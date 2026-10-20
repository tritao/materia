package camkit;

import toolpathkit.path.PathGeometry;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.GeometryOffset;
import toolpathkit.path.Point3;

typedef CamSheetProfileSettings = {
  @:optional var side:String;
  @:optional var direction:CamCutDirection;
  @:optional var stepDown:Float;
  @:optional var plungeFeed:Float;
  @:optional var spindleRpm:Float;
  @:optional var onionSkin:Float;
  @:optional var skinPass:Bool;
  /** Manual centres are distances along the compensated, oriented, entry-rotated loop. */
  @:optional var tabCentres:Array<Float>;
  @:optional var autoTabs:Int;
  /** Minimum retained length along a straight edge, after accounting for cutter diameter. */
  @:optional var tabLength:Float;
  @:optional var tabThickness:Float;
  @:optional var cornerClearance:Float;
  @:optional var leadRadius:Float;
  @:optional var rampLength:Float;
  @:optional var featureRef:String;
}
typedef CamProfileTab = {centre:Float,start:Float,end:Float,length:Float,thickness:Float};
typedef CamSheetProfilePlan = {path:Array<PathGeometry>,length:Float,tabs:Array<CamProfileTab>,leadRadius:Float,rampLength:Float};

/** Exact compensated paths with explicit retained webs. All dimensions are SI. */
class CamSheetProfile {
  public static function reverse(g:PathGeometry):PathGeometry return switch(g) {
    case Line(a,b):Line(b,a);
    case Arc(c,r,a,s):Arc(c,r,a+s,-s);
    case _:throw "Sheet profiles require planar lines and arcs";
  };
  public static function slice(g:PathGeometry,from:Float,to:Float,z0:Float,z1:Float):PathGeometry {
    var length=GeometryTools.length(g),a=GeometryTools.pointAt(g,Math.min(length,from)),b=GeometryTools.pointAt(g,Math.min(length,to));
    return switch(g) {
      case Line(_,_):Line(new Point3(a.x,a.y,z0),new Point3(b.x,b.y,z1));
      case Arc(c,r,start,sweep):
        var angle=start+sweep*from/length,turn=sweep*(to-from)/length;
        if(Math.abs(z0-z1)<1e-12)Arc(new Point3(c.x,c.y,z0),r,angle,turn)
        else Circular(new Point3(c.x,c.y,z0),r,angle,turn,XY,z1-z0);
      case _:throw "Sheet profiles require planar lines and arcs";
    };
  }
  public static function orient(path:Array<PathGeometry>,outside:Bool,direction:CamCutDirection):Array<PathGeometry> {
    var area=0.0;
    for(g in path) {
      var previous=GeometryTools.pointAt(g,0),length=GeometryTools.length(g);
      for(i in 1...17){var next=GeometryTools.pointAt(g,i==16?length:length*i/16);area+=previous.x*next.y-next.x*previous.y;previous=next;}
    }
    if((area>0)==((direction==Climb)!=outside))return path;
    var result=path.copy();result.reverse();return [for(g in result)reverse(g)];
  }
  static function circlePath(contour:CamContour,radius:Float,outside:Bool):Null<Array<PathGeometry>> {
    var source=contour.exactBoundary;
    if(source==null||source.length==0||source[0].center==null)return null;
    var first=source[0],centre=first.arcCenter(),sweep=0.0;
    for(s in source) {
      if(s.center==null||s.arcCenter().distance(centre)>1e-10||Math.abs(s.radius-first.radius)>1e-10)return null;
      sweep+=s.sweep;
    }
    if(Math.abs(Math.abs(sweep)-2*Math.PI)>1e-8)return null;
    var compensated=first.radius+(outside?radius:-radius);
    if(compensated<=1e-10)throw "Circular profile is cutter-sized or smaller; use a drill operation";
    return [Arc(new Point3(centre.x,centre.y,contour.z),compensated,first.startAngle,sweep)];
  }
  public static function plan(contour:CamContour,tool:toolpathkit.tool.Tool,settings:CamSheetProfileSettings):CamSheetProfilePlan {
    if(contour==null||tool==null||settings==null)throw "Sheet profile needs a contour, tool and settings";
    var side=settings.side==null?"outside":settings.side;
    if(side!="outside"&&side!="inside")throw "Sheet profile side must be outside or inside";
    var circular=circlePath(contour,tool.diameter/2,side=="outside");
    var path:Array<PathGeometry> = circular!=null?circular:contour.exactBoundary==null?
      GeometryOffset.profile(contour.vertices,tool.diameter/2,contour.z,side=="outside"):
      CamExactBoundary.offset(contour,tool.diameter/2,contour.z,side=="outside");
    path=orient(path,side=="outside",settings.direction==null?Climb:settings.direction);
    if(path.length==0)throw "Sheet profile has no cutter-centre loop";
    // Enter halfway along the longest straight span, away from a geometric corner.
    var entry=0,longest=0.0;
    for(i in 0...path.length)switch(path[i]){case Line(_,_):if(GeometryTools.length(path[i])>longest){entry=i;longest=GeometryTools.length(path[i]);}case _:}
    var first=path[entry],half=GeometryTools.length(first)/2;
    var rotated=[slice(first,half,2*half,contour.z,contour.z)];
    for(i in 1...path.length)rotated.push(path[(entry+i)%path.length]);
    rotated.push(slice(first,0,half,contour.z,contour.z));path=rotated;
    var length=0.0;for(g in path)length+=GeometryTools.length(g);
    var requested=settings.autoTabs==null?0:settings.autoTabs;
    var manual:Array<Float> = settings.tabCentres==null?[]:settings.tabCentres;
    if(requested<0||requested>1000||(requested>0&&manual.length>0))throw "Invalid automatic/manual tab selection";
    var tabLength=settings.tabLength==null?0.008:settings.tabLength,thickness=settings.tabThickness==null?0.003:settings.tabThickness;
    var clearance=settings.cornerClearance==null?tool.diameter:settings.cornerClearance;
    if(!Math.isFinite(tabLength)||tabLength<=0||!Math.isFinite(thickness)||thickness<=0||!Math.isFinite(clearance)||clearance<0)
      throw "Invalid sheet tab dimensions";
    var halfWidth=(tabLength+tool.diameter)/2,centres:Array<Float> = [];
    if(circular!=null&&(requested>0||manual.length>0)) {
      var radius=switch(circular[0]){case Arc(_,r,_,_):r;case _:0.0;},cutter=tool.diameter/2;
      if(radius<=cutter)throw "Circular cutter envelope cannot leave independent tabs";
      // Bound the cutter's angular reach; retain the requested length even at
      // the smallest radius through the entire curved kerf.
      halfWidth=radius*(tabLength/(2*(radius-cutter))+Math.asin(cutter/radius));
    }
    function allowed(position:Float):Bool {
      if(!Math.isFinite(position)||position<halfWidth+clearance||position>length-halfWidth-clearance)return false;
      var offset=0.0,found=false;
      for(g in path) {
        var span=GeometryTools.length(g),straight=switch(g){case Line(_,_):true;case _:longest==0;};
        if(straight&&position>=offset+halfWidth+clearance&&position<=offset+span-halfWidth-clearance)found=true;
        offset+=span;
      }
      for(other in centres)if(Math.abs(position-other)<2*halfWidth+clearance)return false;
      return found;
    }
    for(position in manual){if(!allowed(position))throw "Manual tab overlaps a corner, entry or another tab";centres.push(position);}
    var steps=Std.int(Math.max(256,requested*128));
    for(i in 0...requested) {
      var desired=length*(i+0.5)/requested,best=-1.0,distance=Math.POSITIVE_INFINITY;
      for(j in 1...steps) {
        var candidate=length*j/steps;
        if(allowed(candidate)&&Math.abs(candidate-desired)<distance){best=candidate;distance=Math.abs(candidate-desired);}
      }
      if(best<0)throw "Requested tabs do not fit away from corners; specify fewer or shorter tabs";
      centres.push(best);
    }
    centres.sort((a,b)->a<b?-1:a>b?1:0);
    var lead=settings.leadRadius==null?0.0:settings.leadRadius,ramp=settings.rampLength==null?0.0:settings.rampLength;
    if(!Math.isFinite(lead)||lead<0||!Math.isFinite(ramp)||ramp<0||ramp>length)throw "Invalid sheet lead or ramp length";
    return {path:path,length:length,tabs:[for(c in centres){centre:c,start:c-halfWidth,end:c+halfWidth,length:tabLength,thickness:thickness}],leadRadius:lead,rampLength:ramp};
  }
  public static function pass(plan:CamSheetProfilePlan,level:Float,prior:Float,bottom:Float,ramp:Bool):Array<PathGeometry> {
    var breaks=[0.0,plan.length],offset=0.0;
    for(g in plan.path){offset+=GeometryTools.length(g);breaks.push(offset);}
    for(tab in plan.tabs){breaks.push(tab.start);breaks.push(tab.end);}
    var run=ramp?plan.rampLength:0.0;
    if(run>0) {
      breaks.push(run);
      if(prior>level)for(tab in plan.tabs){var crossing=run*(prior-bottom-tab.thickness)/(prior-level);if(crossing>0&&crossing<run)breaks.push(crossing);}
    }
    breaks.sort((a,b)->a<b?-1:a>b?1:0);
    function height(s:Float,middle:Float):Float {
      var z=run>0?prior+(level-prior)*Math.min(1,s/run):level;
      for(tab in plan.tabs)if(middle>tab.start-1e-12&&middle<tab.end+1e-12)z=Math.max(z,bottom+tab.thickness);
      return z;
    }
    var output:Array<PathGeometry> = [],piece=0,start=0.0,previous:Null<Point3> = null;
    for(i in 0...breaks.length-1) {
      var a=breaks[i],b=breaks[i+1];if(b-a<1e-11)continue;
      while(piece+1<plan.path.length&&a>=start+GeometryTools.length(plan.path[piece])-1e-11){start+=GeometryTools.length(plan.path[piece]);piece++;}
      var span=GeometryTools.length(plan.path[piece]);
      var g=slice(plan.path[piece],Math.max(0,a-start),Math.min(span,b-start),height(a,(a+b)/2),height(b,(a+b)/2));
      var begin=GeometryTools.pointAt(g,0);
      if(previous!=null&&previous.distanceTo(begin)>1e-10)output.push(Line(previous,begin));
      output.push(g);previous=GeometryTools.pointAt(g,GeometryTools.length(g));
    }
    return output;
  }
  /** Quarter-circle lead, on the waste side and tangent to the finished loop. */
  public static function lead(plan:CamSheetProfilePlan,z:Float,direction:CamCutDirection,out:Bool):PathGeometry {
    var g=plan.path[0],p=GeometryTools.pointAt(g,0),t=GeometryTools.tangentAt(g,0);
    var left=direction==Climb,sign=left?1.0:-1.0,r=plan.leadRadius;
    var c=new Point3(p.x-sign*t.y*r,p.y+sign*t.x*r,z),angle=Math.atan2(p.y-c.y,p.x-c.x),sweep=sign*Math.PI/2;
    return Arc(c,r,out?angle:angle-sweep,sweep);
  }
}
