import nativekit.scene.gizmo.*;
import nativekit.scene.gizmo.ScaleHandle;

class ScaleGizmoTests {
  static function check(value:Bool,message:String):Void {if(!value) throw message;}
  static function rejects(action:Void->Void):Void {var failed=false;try action() catch(_:Dynamic) failed=true;check(failed,"Expected scale rejection");}
  public static function run():Void {
    var pivot=new GizmoVec3(0,0,0),camera=new GizmoCamera(new GizmoVec3(10,10,10),pivot,new GizmoVec3(0,0,1),800,600);
    for(basis in [GizmoBasis.world(),GizmoBasis.fromQuaternion(0,0,Math.sin(Math.PI/12),Math.cos(Math.PI/12))]) {
      var layout=new ScaleGizmo(camera,pivot,basis);check(layout.shapes().length==4,"Missing scale handles");
      for(shape in layout.shapes()) {
        var points=shape.points(),x=0.0,y=0.0;for(point in points){x+=point.x/points.length;y+=point.y/points.length;}
        check(layout.pick(x,y)==shape.handle,"Scale draw/pick disagree");
        var drag=layout.begin(shape.handle,x,y,0.25);check(drag.update(x,y)==1,"Off-grid scale jumped on grab");
        var center=camera.project(pivot);if(center==null) throw "Missing scale center";
        var dx=layout.pixels/Math.sqrt(2),dy= -dx;
        if(shape.handle!=Uniform){dx=points[1].x-center.x;dy=points[1].y-center.y;}
        var factor=drag.update(x+0.6*dx,y+0.6*dy);check(factor!=null && Math.abs(factor-1.5)<0.000001,"Scale factor or snap is wrong");
        var factors=drag.factors(1.5);var expected=switch shape.handle {case ScaleX:[1.5,1.0,1.0];case ScaleY:[1.0,1.5,1.0];case ScaleZ:[1.0,1.0,1.5];case Uniform:[1.5,1.5,1.5];};
        for(i in 0...3) check(factors[i]==expected[i],"Scale changed another axis");
        check(drag.update(x-2*dx,y-2*dy)==null && drag.lastFactor==1.5,"Negative factor replaced valid preview");
        check(drag.update(Math.NaN,y)==null,"Nonfinite scale accepted");
        check(drag.update(x,y)==1,"Scale accumulated pointer movement");drag.finish();rejects(()->drag.update(x,y));
      }
      var detached=layout.shapes();detached.pop();check(layout.shapes().length==4,"Scale leaked shape collection");
      var points=layout.shapes()[0].points();points.pop();check(layout.shapes()[0].points().length==2,"Scale leaked point collection");
      rejects(()->layout.begin(Uniform,0,0));
    }
    var normal=new ScaleGizmo(camera,pivot,GizmoBasis.world());
    var distant=new ScaleGizmo(new GizmoCamera(new GizmoVec3(100,100,100),pivot,new GizmoVec3(0,0,1),800,600),pivot,GizmoBasis.world());
    for(i in 0...normal.shapes().length){var a=normal.shapes()[i].points(),b=distant.shapes()[i].points();for(j in 0...a.length) check(Math.abs(a[j].x-b[j].x)<0.000001 && Math.abs(a[j].y-b[j].y)<0.000001,"Zoom changed logical scale size");}
    check(new ScaleGizmo(camera,new GizmoVec3(1000,1000,1000),GizmoBasis.world()).shapes().length==0,"Clipped scale was pickable");
    Sys.println("Scale draw/pick, oriented factors, snapping, zoom, invalid proposals and frozen lifecycle passed");
  }
}
