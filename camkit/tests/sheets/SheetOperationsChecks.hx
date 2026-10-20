import camkit.*;
import camkit.CamSheetProfile.CamSheetProfileSettings;
import camkit.CamSheetOrder.CamSheetOperation;
import stockkit.*;
import toolpathkit.tool.Tool;
import toolpathkit.path.*;

class SheetOperationsChecks {
  static var assertions=0;
  static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw label;}
  static function near(a:Float,b:Float,label:String,tol:Float=1e-10):Void check(Math.abs(a-b)<=tol,'$label ($a vs $b)');
  static function rejects(action:Void->Void,label:String):Void {var failed=false;try action()catch(_:Dynamic)failed=true;check(failed,label);}
  public static function rectangle(x:Float,y:Float,w:Float,h:Float):CamContour return new CamContour([
    new Point3(x,y,0.018),new Point3(x+w,y,0.018),new Point3(x+w,y+h,0.018),new Point3(x,y+h,0.018)]);
  static function settings():CamSheetProfileSettings return {autoTabs:4,tabLength:0.008,tabThickness:0.003,cornerClearance:0.003,
    stepDown:0.006,onionSkin:0.0005,skinPass:true,rampLength:0.025,leadRadius:0.002};
  static function pointAt(path:Array<PathGeometry>,distance:Float):Point3 {
    for(g in path){var length=GeometryTools.length(g);if(distance<=length)return GeometryTools.pointAt(g,distance);distance-=length;}
    throw "test path distance";
  }
  static function planChecks():Void {
    var contour=rectangle(0.02,0.02,0.04,0.05),tool=new Tool(3,0,0.006,null,0.04,0.06),options=settings();
    var plan=CamSheetProfile.plan(contour,tool,options);
    check(plan.tabs.length==4,"four automatic tabs fit away from corners");
    for(tab in plan.tabs){near(tab.end-tab.start,0.014,"tab lift includes cutter diameter");near(tab.thickness,0.003,"tab thickness");}
    options.autoTabs=0;options.tabCentres=[plan.tabs[0].centre,plan.tabs[2].centre];
    check(CamSheetProfile.plan(contour,tool,options).tabs.length==2,"manual tab locations");
    options.tabCentres=[0.0];rejects(()->CamSheetProfile.plan(contour,tool,options),"entry/corner tab refused");
    options=settings();
    var leadIn=CamSheetProfile.lead(plan,0,Climb,false),leadOut=CamSheetProfile.lead(plan,0,Climb,true);
    var tangent=GeometryTools.tangentAt(plan.path[0],0),a=GeometryTools.tangentAt(leadIn,GeometryTools.length(leadIn)),b=GeometryTools.tangentAt(leadOut,0);
    near(a.x*tangent.x+a.y*tangent.y,1,"lead-in tangent");near(b.x*tangent.x+b.y*tangent.y,1,"lead-out tangent");
    var directions:Array<CamCutDirection> = [Climb,Conventional];
    for(outside in [false,true])for(direction in directions) {
      var p=CamSheetProfile.plan(contour,tool,{side:outside?"outside":"inside",direction:direction});var area=0.0;
      for(g in p.path){var previous=GeometryTools.pointAt(g,0),length=GeometryTools.length(g);for(i in 1...33){var next=GeometryTools.pointAt(g,i==32?length:length*i/32);area+=previous.x*next.y-next.x*previous.y;previous=next;}}
      check((area>0)==((direction==Climb)!=outside),"inside/outside climb/conventional winding");
    }
    var circleText="0\nSECTION\n2\nENTITIES\n0\nCIRCLE\n8\nCUT\n10\n50\n20\n50\n40\n30\n0\nENDSEC\n0\nEOF\n";
    var imported=cadkit.drawing.DxfContours.assemble(cadkit.drawing.Dxf.read(circleText).curves);
    var mapped=CamSheetProfiles.fromDxf(imported.parts[0],0.018),circlePlan=CamSheetProfile.plan(mapped.outer,tool,settings());
    var boundary=mapped.outer.exactBoundary;if(boundary==null)throw "missing DXF boundary";
    check(boundary.length==2&&circlePlan.tabs.length==4,"DXF circle retains exact semicircles and accepts four tabs");
    near(circlePlan.length,2*Math.PI*0.033,"DXF-to-CAM units and circular compensation");
    var job=new CamJob(0.025,12000,new Point3(0,0,0.025)).sheetProfile(contour,tool,0,0.05,options).finish(),ramps=0;
    for(op in job.ops)switch(op){case Move(Ramp,g,_,_,_):if(GeometryTools.length(g)>0)ramps++;case _:}
    check(ramps>0,"profile ramp is an authored feed move");
  }
  static function stockChecks():Void {
    var contour=rectangle(0.02,0.02,0.04,0.05),tool=new Tool(3,0,0.006,null,0.04,0.06),options=settings();
    options.leadRadius=0;options.rampLength=0;
    var lattice=StockLattice.covering(0,0,0,0.08,0.096,0.018,0.0005);
    var rough=Stock.box(lattice,0,0,0,0.08,0.096,0.018),finished=Stock.box(lattice,0,0,0,0.08,0.096,0.018);
    rough.setThreads(1);finished.setThreads(1);
    options.skinPass=false;
    rough.cut(CutMoves.fromProgram(new CamJob(0.025,12000,new Point3(0,0,0.025)).sheetProfile(contour,tool,0,0.05,options).finish()));
    options.skinPass=true;
    finished.cut(CutMoves.fromProgram(new CamJob(0.025,12000,new Point3(0,0,0.025)).sheetProfile(contour,tool,0,0.05,options).finish()));
    var changed=0,valid=true;
    for(row in 0...rough.grid.countV) {
      var a=rough.rays(0,row,rough.grid.countU,1),b=finished.rays(0,row,finished.grid.countU,1);
      for(i in 0...a.length) {
        var before=a[i].length==0?0:a[i][0].hi,after=b[i].length==0?0:b[i][0].hi,difference=before-after;
        if(Math.abs(difference)>1e-10){changed++;if(Math.abs(difference-0.0005)>1e-10)valid=false;}
      }
    }
    check(changed>0&&valid,"skin pass removes only the declared 0.5 mm layer");
    check(StockComponents.measure(finished).length==1,"three-millimetre tabs retain the through-profile part");
    var plan=CamSheetProfile.plan(contour,tool,options);
    for(tab in plan.tabs) {
      var p=pointAt(plan.path,tab.centre),i=Std.int(Math.round((p.x-finished.grid.u(0))/finished.grid.spacing)),j=Std.int(Math.round((p.y-finished.grid.v(0))/finished.grid.spacing));
      var rays=finished.rays(i,j,1,1);check(rays[0].length==1,"tab centre contains material");near(rays[0][0].hi,0.003,"native tab thickness");
      for(delta in [-0.0035,0.0035]) {
        var q=pointAt(plan.path,tab.centre+delta),u=Std.int(Math.round((q.x-finished.grid.u(0))/finished.grid.spacing)),v=Std.int(Math.round((q.y-finished.grid.v(0))/finished.grid.spacing));
        var sample=finished.rays(u,v,1,1);check(sample[0].length==1&&Math.abs(sample[0][0].hi-0.003)<1e-10,"native web retains stated length away from the rounded cutter caps");
      }
    }
    check(finished.volume()<rough.volume(),"skin pass removes additional material");
    rough.dispose();finished.dispose();
    // An independent box target, not a replay of the toolpath, verifies volume accounting.
    var surface=Stock.box(lattice,0,0,0,0.08,0.096,0.018),target=Stock.box(lattice,0,0,0,0.08,0.096,0.016);
    var job=new CamJob(0.025,12000,new Point3(0,0,0.025)).surface(new Point3(0,0,0.018),new Point3(0.08,0.096,0.018),tool,0.016,0.05,0.003,0.001);
    var program=job.finish(),before=surface.volume(),report=surface.cut(CutMoves.fromProgram(program));
    near(surface.compare(target).gougeVolume(),0,"surfacing does not gouge box target");near(surface.compare(target).leftoverVolume(),0,"surfacing clears complete target plane");
    near(before-surface.volume(),before-target.volume(),"removed volume equals stock minus independent target",1e-12);
    surface.dispose();target.dispose();
  }
  static function drillingAndOrder():Void {
    var tool=new Tool(3,0,0.006,null,0.04,0.06);
    var job=new CamJob(0.025,12000,new Point3(0,0,0.025));
    job.drill([new Point3(0.01,0.01,0.018)],tool,0.012,0.020,0.005,"pecks",0.002);
    job.withSpindleSpeed(18000).drill([new Point3(0.02,0.02,0.018)],tool,0.016,0.020,0.005,"speed");
    var pecks=0,changes=0,rpm:Array<Float> = [];
    for(op in job.finish().ops)switch(op){case Move(Plunge,_,_,_,span):if(span.featureRef=="pecks")pecks++;case ToolChange(_,_):changes++;case Spindle(Clockwise,s,_):rpm.push(s);case _:}
    check(pecks==3,"three pecks reach 6 mm depth");check(changes==1&&rpm.length==2&&rpm[0]==12000&&rpm[1]==18000,"per-operation speed changes without reloading tool");
    var operations:Array<CamSheetOperation> = [
      {id:"large-out",part:"large",area:0.4,kind:"outside",requires:[],releases:true},
      {id:"small-out",part:"small",area:0.0025,kind:"outside",requires:[],releases:true},
      {id:"small-hole",part:"small",area:0.0025,kind:"inside",requires:[],releases:false},
      {id:"large-hole",part:"large",area:0.4,kind:"inside",requires:[],releases:false}];
    var order=CamSheetOrder.order(operations);
    check([for(op in order.operations)op.id].join(",")=="small-hole,large-hole,small-out,large-out","inside-first then small-before-large release ordering");
    rejects(()->CamSheetOrder.order(operations,op->op.id!="small-out"),"failed holding check prevents an unsafe completed plan");
  }
  public static function main():Void {planChecks();stockChecks();drillingAndOrder();Sys.println('Sheet operation checks passed ($assertions assertions)');}
}
