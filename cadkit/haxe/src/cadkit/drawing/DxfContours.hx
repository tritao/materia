package cadkit.drawing;

import cadkit.drawing.DxfReader.DxfCurve;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.OutlineGeometry;
import cadkit.sheetmetal.WallOutline;

typedef DxfContour = {id:String,layer:String,operation:String,segments:Array<Segment>,area:Float};
typedef DxfGap = {layer:String,position:Point2,nearest:Point2,distance:Float,reason:String};
typedef DxfPart = {outer:DxfContour,holes:Array<DxfContour>,outline:WallOutline};
typedef DxfContourResult = {contours:Array<DxfContour>,parts:Array<DxfPart>,gaps:Array<DxfGap>};

/** Exact line/arc assembly; accepted endpoint gaps become explicit short bridges rather than altered arcs. */
class DxfContours {
  static function reverse(segment:Segment):Segment return segment.isLine()?Segment.line(segment.end,segment.start):
    Segment.arc(segment.arcCenter(),segment.radius,segment.startAngle+segment.sweep,-segment.sweep);
  static function oriented(loop:Array<Segment>,ccw:Bool):Array<Segment> {
    var area=0.0;for(s in loop)area+=s.area();
    if((area>0)==ccw)return loop.copy();
    var result=loop.copy();result.reverse();return [for(s in result)reverse(s)];
  }
  static function bridge(loop:Array<Segment>,next:Segment):Void {
    var end=loop[loop.length-1].end;
    if(end.distance(next.start)>1e-8)loop.push(Segment.line(end,next.start));
    loop.push(next);
  }
  public static function assemble(curves:Array<DxfCurve>,gapTolerance:Float=0.05,?operations:Map<String,String>):DxfContourResult {
    if(!Math.isFinite(gapTolerance)||gapTolerance<1e-8)throw "DXF gap tolerance must be at least 1e-8 mm";
    var out:DxfContourResult={contours:[],parts:[],gaps:[]};
    var layers:Array<String> = [];
    for(curve in curves)if(layers.indexOf(curve.layer)<0)layers.push(curve.layer);
    for(layer in layers) {
      var unused=[for(c in curves)if(c.layer==layer)c.segment];
      while(unused.length>0) {
        var first=unused.shift(),loop:Array<Segment> = [first],failed=false;
        // Extend at both ends so input ordering and line direction do not determine whether a chain closes.
        for(side in 0...2) {
          while(loop[loop.length-1].end.distance(loop[0].start)>gapTolerance) {
            var end=loop[loop.length-1].end,candidates:Array<{index:Int,back:Bool}> = [];
            for(i in 0...unused.length) {
              if(end.distance(unused[i].start)<=gapTolerance)candidates.push({index:i,back:false});
              if(end.distance(unused[i].end)<=gapTolerance)candidates.push({index:i,back:true});
            }
            if(candidates.length!=1) {
              if(candidates.length>1) {
                out.gaps.push({layer:layer,position:end,nearest:end,distance:0,reason:"ambiguous endpoint branch"});failed=true;
              }
              break;
            }
            var choice=candidates[0],next=unused.splice(choice.index,1)[0];
            bridge(loop,choice.back?reverse(next):next);
          }
          if(loop[loop.length-1].end.distance(loop[0].start)<=gapTolerance||failed)break;
          loop.reverse();loop=[for(s in loop)reverse(s)];
        }
        var end=loop[loop.length-1].end,start=loop[0].start,distance=end.distance(start);
        if(distance>gapTolerance||failed) {
          if(!failed) {
            var nearest=start,closest=distance;
            for(s in unused)for(p in [s.start,s.end])if(end.distance(p)<closest){nearest=p;closest=end.distance(p);}
            out.gaps.push({layer:layer,position:end,nearest:nearest,distance:closest,reason:"open contour"});
          }
          continue;
        }
        if(distance>1e-8)loop.push(Segment.line(end,start));
        var area=0.0;for(s in loop)area+=s.area();
        if(Math.abs(area)<=1e-12)throw 'DXF zero-area contour on layer "$layer"';
        // A single closed contour must also be simple.
        var normalized=oriented(loop,true);new WallOutline(normalized);
        out.contours.push({id:'contour-${out.contours.length+1}',layer:layer,
          operation:operations!=null&&operations.exists(layer)?operations.get(layer):layer,segments:normalized,area:Math.abs(area)});
      }
    }
    // Crossings between loops cannot define an unambiguous material hierarchy.
    for(i in 0...out.contours.length)for(j in i+1...out.contours.length) {
      var a=out.contours[i],b=out.contours[j];
      if(a.operation!=b.operation)continue;
      for(s in a.segments)for(t in b.segments)if(OutlineGeometry.intersections(s,t).length>0)
        throw 'DXF contours ${a.id} and ${b.id} touch or intersect';
    }
    var parents:Array<Int> = [],depths:Array<Int> = [];
    for(i in 0...out.contours.length) {
      var child=out.contours[i],parent=-1;
      for(j in 0...out.contours.length)if(i!=j) {
        var candidate=out.contours[j];
        if(candidate.operation==child.operation&&candidate.area>child.area&&
          OutlineGeometry.contains(candidate.segments,child.segments[0].start)&&
          (parent<0||candidate.area<out.contours[parent].area))parent=j;
      }
      parents.push(parent);
    }
    for(i in 0...out.contours.length) {
      var depth=0,p=parents[i];while(p>=0){depth++;p=parents[p];}depths.push(depth);
    }
    for(i in 0...out.contours.length)if(depths[i]%2==0) {
      var outer=out.contours[i],holes=[for(j in 0...out.contours.length)if(parents[j]==i)out.contours[j]];
      var outline=new WallOutline(outer.segments,[for(hole in holes)oriented(hole.segments,false)]);
      out.parts.push({outer:outer,holes:holes,outline:outline});
    }
    return out;
  }
}
