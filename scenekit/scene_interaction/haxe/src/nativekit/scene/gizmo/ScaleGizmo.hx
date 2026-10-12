package nativekit.scene.gizmo;
import nativekit.scene.gizmo.ScaleHandle;

/** Positive local-axis and uniform scale factors, using frozen screen geometry. */
class ScaleGizmo {
  public final camera:GizmoCamera;
  public final pivot:GizmoVec3;
  public final basis:GizmoBasis;
  public final pixels:Float;
  final geometry:Array<ScaleShape> = [];
  public function new(camera:GizmoCamera,pivot:GizmoVec3,basis:GizmoBasis,pixels:Float=90) {
    if(camera==null || pivot==null || basis==null || !Math.isFinite(pixels) || pixels<24 || pixels>512) throw "Invalid scale layout";
    this.camera=camera;this.pivot=pivot;this.basis=basis;this.pixels=pixels;
    var center=camera.project(pivot); if(center==null || !camera.contains(center.x,center.y)) return;
    var size=camera.worldLength(pivot,pixels),eye=camera.eye.subtract(pivot).normalized();
    for(handle in [ScaleX,ScaleY,ScaleZ]) {
      var axis=switch handle {case ScaleX:basis.x;case ScaleY:basis.y;default:basis.z;};
      if(Math.abs(axis.dot(eye))>0.99) continue;
      var start=camera.project(pivot.add(axis.scaled(size*0.16))),tip=camera.project(pivot.add(axis.scaled(size)));
      if(start==null || tip==null) continue;
      var dx=tip.x-center.x,dy=tip.y-center.y;if(dx*dx+dy*dy<576) continue;
      geometry.push(new ScaleShape(handle,[start,tip]));
    }
    geometry.push(new ScaleShape(Uniform,[new GizmoPoint(center.x-6,center.y-6,center.depth),new GizmoPoint(center.x+6,center.y-6,center.depth),new GizmoPoint(center.x+6,center.y+6,center.depth),new GizmoPoint(center.x-6,center.y+6,center.depth)]));
  }
  public function shapes():Array<ScaleShape> return geometry.copy();
  public function pick(x:Float,y:Float):Null<ScaleHandle> {
    if(!camera.contains(x,y)) return null;
    for(shape in geometry) if(shape.handle==Uniform && shape.distance(x,y)==0) return Uniform;
    var best:Null<ScaleHandle> = null,distance=8.0;
    for(shape in geometry) {var value=shape.distance(x,y);if(value!=null && value<distance){best=shape.handle;distance=value;}}
    return best;
  }
  public function begin(handle:ScaleHandle,x:Float,y:Float,snapFactor:Float=0):ScaleDrag {
    if(handle==null || pick(x,y)!=handle) throw "Scale must begin on a visible picked handle";
    var center=camera.project(pivot);if(center==null) throw "Missing scale pivot";
    var dx=1/Math.sqrt(2),dy= -dx,span=pixels;
    if(handle!=Uniform) for(shape in geometry) if(shape.handle==handle) {
      var points=shape.points(),tip=points[1];dx=tip.x-center.x;dy=tip.y-center.y;span=Math.sqrt(dx*dx+dy*dy);dx/=span;dy/=span;
    }
    return new ScaleDrag(handle,x,y,dx,dy,span,snapFactor);
  }
  public static function label(handle:ScaleHandle):String return switch handle {case ScaleX:"X";case ScaleY:"Y";case ScaleZ:"Z";case Uniform:"All";};
}
