package camkit;

import materia.sheet.SheetPolygon;
import materia.sheet.SheetPolygon.SheetPoint;
import materia.sheet.SheetPolygon.SheetShape;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.OutlineGeometry;
import cadkit.modeling.Curve;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;

typedef CamNestVariant = {angle:Float,shape:SheetShape,pieces:Array<Array<SheetPoint>>};
typedef CamNestPlaced = {id:String,requirementId:String,x:Float,y:Float,variant:CamNestVariant,shape:SheetShape,pieces:Array<Array<SheetPoint>>};
typedef CamNfpStats = {pairs:Int,candidates:Int,decompositions:Int};

/** Configuration obstacles are unions of convex Minkowski sums, with exact rounded clearance offsets. */
class CamSheetNfp {
  public static function hull(points:Array<SheetPoint>):Array<SheetPoint> {
    var sorted=points.copy();sorted.sort((a,b)->a.x!=b.x?(a.x<b.x?-1:1):a.y<b.y?-1:a.y>b.y?1:0);
    var unique:Array<SheetPoint> = [];for(p in sorted)if(unique.length==0||p.x!=unique[unique.length-1].x||p.y!=unique[unique.length-1].y)unique.push(p);
    if(unique.length<3)throw "Degenerate no-fit polygon";
    var lower:Array<SheetPoint> = [],upper:Array<SheetPoint> = [];
    for(p in unique){while(lower.length>1&&SheetPolygon.cross(lower[lower.length-2],lower[lower.length-1],p)<=0)lower.pop();lower.push(p);}
    unique.reverse();for(p in unique){while(upper.length>1&&SheetPolygon.cross(upper[upper.length-2],upper[upper.length-1],p)<=0)upper.pop();upper.push(p);}
    lower.pop();upper.pop();return lower.concat(upper);
  }
  public static function convex(shape:SheetShape):Bool {
    if(shape.length!=1)return false;
    var loop=shape[0];for(i in 0...loop.length)if(SheetPolygon.cross(loop[i],loop[(i+1)%loop.length],loop[(i+2)%loop.length])<0)return false;return true;
  }
  public static function decompose(shape:SheetShape,stats:CamNfpStats):Array<Array<SheetPoint>> {
    if(convex(shape))return [shape[0]];
    var wires:Array<Curve> = [],face:Null<Sketch> = null;
    try {
      for(loop in shape)wires.push(Curve.polyline([for(p in loop)new Vector(p.x,p.y,0)],true));
      face=Sketch.face(wires[0],[for(i in 1...wires.length)wires[i]]);
      var mesh=face.shape.tessellate(0.001,0.1),pieces:Array<Array<SheetPoint>> = [],area=0.0;
      for(i in 0...Std.int(mesh.indexCount/3)) {
        var triangle:Array<SheetPoint> = [];
        for(j in 0...3){var vertex=mesh.indices.getInt32((i*3+j)*4);triangle.push(SheetPolygon.point(mesh.vertices.getDouble(vertex*24),mesh.vertices.getDouble(vertex*24+8)));}
        var size=SheetPolygon.area(triangle);if(Math.abs(size)<1e-9)continue;
        if(size<0)triangle.reverse();pieces.push(triangle);area+=Math.abs(size);
      }
      if(pieces.length==0||Math.abs(area-SheetPolygon.materialArea(shape))>Math.max(0.01,SheetPolygon.materialArea(shape)*1e-8))
        throw "Native convex decomposition does not conserve polygon material area";
      for(w in wires)w.close();face.close();stats.decompositions++;return pieces;
    }catch(error:Dynamic){for(w in wires)w.close();if(face!=null)face.close();throw error;}
  }
  static function rounded(polygon:Array<SheetPoint>,spacing:Float):Array<Segment> {
    var result:Array<Segment> = [];
    for(i in 0...polygon.length) {
      var a=polygon[(i+polygon.length-1)%polygon.length],b=polygon[i],c=polygon[(i+1)%polygon.length];
      var prior=Math.atan2(-(b.x-a.x),b.y-a.y),next=Math.atan2(-(c.x-b.x),c.y-b.y),sweep=(next-prior)%(2*Math.PI);
      if(sweep<0)sweep+=2*Math.PI;
      if(spacing>0&&sweep>1e-12)result.push(Segment.arc(new Point2(b.x,b.y),spacing,prior,sweep));
      var start=new Point2(b.x+spacing*Math.cos(next),b.y+spacing*Math.sin(next));
      var end=new Point2(c.x+spacing*Math.cos(next),c.y+spacing*Math.sin(next));
      result.push(Segment.line(start,end));
    }
    return result;
  }
  public static function obstacles(placed:Array<CamNestPlaced>,moving:CamNestVariant,spacing:Float,stats:CamNfpStats):Array<Segment> {
    var edges:Array<Segment> = [];
    for(part in placed)for(a in part.pieces)for(b in moving.pieces) {
      var sum:Array<SheetPoint> = [];
      for(p in a)for(q in b)sum.push(SheetPolygon.point(p.x-q.x,p.y-q.y));
      for(edge in rounded(hull(sum),spacing))edges.push(edge);
      stats.pairs++;
    }
    return edges;
  }
  public static function candidates(placed:Array<CamNestPlaced>,variant:CamNestVariant,width:Float,height:Float,margin:Float,spacing:Float,stats:CamNfpStats):Array<SheetPoint> {
    var b=SheetPolygon.bounds(variant.shape),lowX=margin-b.minX,lowY=margin-b.minY,highX=width-margin-b.maxX,highY=height-margin-b.maxY;
    if(highX<lowX-SheetPolygon.EPS||highY<lowY-SheetPolygon.EPS)return [];
    var points:Array<SheetPoint> = [],seen=new Map<String,Bool>();
    function add(p:Point2):Void {
      // Grid neighbours of analytic contacts keep round-off from discarding a feasible boundary placement.
      var x=SheetPolygon.snap(p.x),y=SheetPolygon.snap(p.y);
      for(dx in [0.0,-0.001,0.001])for(dy in [0.0,-0.001,0.001]) {
        var q=SheetPolygon.point(x+dx,y+dy);
        if(q.x<lowX-SheetPolygon.EPS||q.y<lowY-SheetPolygon.EPS||q.x>highX+SheetPolygon.EPS||q.y>highY+SheetPolygon.EPS)continue;
        var key='${Math.round(q.x*1000)},${Math.round(q.y*1000)}';if(!seen.exists(key)){seen.set(key,true);points.push(q);}
      }
    }
    var boundary:Array<Segment> = [];
    if(highX-lowX>1e-9&&highY-lowY>1e-9) {
      var corners=[new Point2(lowX,lowY),new Point2(highX,lowY),new Point2(highX,highY),new Point2(lowX,highY)];
      for(i in 0...4){add(corners[i]);boundary.push(Segment.line(corners[i],corners[(i+1)%4]));}
    } else {
      add(new Point2(lowX,lowY));add(new Point2(highX,highY));
      if(highX-lowX>1e-9||highY-lowY>1e-9)boundary.push(Segment.line(new Point2(lowX,lowY),new Point2(highX,highY)));
    }
    var edges=obstacles(placed,variant,spacing,stats);
    var bounded:Array<Segment> = [];
    for(edge in edges) {
      var minX=Math.POSITIVE_INFINITY,minY=Math.POSITIVE_INFINITY,maxX=Math.NEGATIVE_INFINITY,maxY=Math.NEGATIVE_INFINITY;
      for(p in edge.extrema()){minX=Math.min(minX,p.x);maxX=Math.max(maxX,p.x);minY=Math.min(minY,p.y);maxY=Math.max(maxY,p.y);add(p);}
      if(maxX>=lowX-1e-8&&maxY>=lowY-1e-8&&minX<=highX+1e-8&&minY<=highY+1e-8)bounded.push(edge);
      for(side in boundary)for(p in OutlineGeometry.intersections(edge,side))add(p);
    }
    // Intersections expose the boundary of the union, including cavities induced by holes.
    for(i in 0...bounded.length)for(j in i+1...bounded.length)
      for(p in OutlineGeometry.intersections(bounded[i],bounded[j]))add(p);
    points.sort((a,b)->a.y!=b.y?(a.y<b.y?-1:1):a.x<b.x?-1:a.x>b.x?1:0);
    stats.candidates+=points.length;return points;
  }
  public static function place(id:String,requirementId:String,variant:CamNestVariant,p:SheetPoint):CamNestPlaced return {
    id:id,requirementId:requirementId,x:p.x,y:p.y,variant:variant,
    shape:SheetPolygon.transform(variant.shape,0,p.x,p.y),
    pieces:[for(piece in variant.pieces)SheetPolygon.transform([piece],0,p.x,p.y)[0]]};
}
