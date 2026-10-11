import nativekit.scene.PerspectiveCamera;
import nativekit.scene.gizmo.*;
import nativekit.scene.gizmo.TranslationHandle;
import beartooth.document.world.*;
import sys.io.File;
class CameraCompatibilityTests {
  static function check(value:Bool, message:String):Void { if (!value) throw message; }
  static function main():Void {
    var camera = new PerspectiveCamera();
    for (angle in [[0.0,0.0],[-0.96,0.56],[1.2,1.48],[-2.0,-1.3]]) {
      camera.setAngle(angle[0],angle[1]); camera.frame(3,2,1,5,6,4,1.3333333333333333);
      var eye = camera.eyePosition();
      var snapshot = new GizmoCamera(new GizmoVec3(eye[0],eye[1],eye[2]),new GizmoVec3(camera.targetX,camera.targetY,camera.targetZ),new GizmoVec3(0,0,1),800,600,PerspectiveCamera.FOV_Y,camera.clipNear,camera.clipFar);
      for (coordinate in [[400.0,300.0],[200.0,180.0],[700.0,500.0]]) {
        var ray = camera.screenRay(coordinate[0],coordinate[1],800,600), proposed = snapshot.rayDirection(coordinate[0],coordinate[1]);
        check(Math.abs(proposed.x-ray.directionX)<1e-12 && Math.abs(proposed.y-ray.directionY)<1e-12 && Math.abs(proposed.z-ray.directionZ)<1e-12,"Frozen gizmo rays disagree with pinned PerspectiveCamera");
        var point = new GizmoVec3(ray.originX+ray.directionX*camera.distance,ray.originY+ray.directionY*camera.distance,ray.originZ+ray.directionZ*camera.distance);
        var actual = camera.project(point.x,point.y,point.z,800,600), projected = snapshot.project(point);
        if (actual==null || projected==null) throw "Expected projected compatibility point";
        check(Math.abs(actual.x-projected.x)<0.0001 && Math.abs(actual.y-projected.y)<0.0001,"Frozen gizmo projection disagrees with pinned PerspectiveCamera");
      }
    }
    var source = 'Place "gesture" { entity "Parent" #c101 { type = "entity" Transform { position = [10, 0, 0] rotationDegrees = [0, 0, 90] scale = [2, 3, 4] } entity "Child" #c102 { type = "entity" Transform { position = [1, 2, 3] } } } }';
    var host = new BtSceneHost(source,new BtTransformBinding(File.getContent(Sys.programPath()+".descriptors.json")));
    host.select("c102",0); var initial = host.entities()[1];
    camera.frame(initial.x,initial.y,initial.z,6,6,6,800.0/600.0);
    var eye = camera.eyePosition(), snapshot = new GizmoCamera(new GizmoVec3(eye[0],eye[1],eye[2]),new GizmoVec3(camera.targetX,camera.targetY,camera.targetZ),new GizmoVec3(0,0,1),800,600);
    var layout = new TranslationGizmo(snapshot,new GizmoVec3(initial.x,initial.y,initial.z));
    var start:Null<GizmoPoint> = null;
    for (shape in layout.shapes()) if (shape.handle == AxisX) {
      var points = shape.points(); start = new GizmoPoint((points[0].x+points[1].x)*0.5,(points[0].y+points[1].y)*0.5,points[0].depth);
    }
    if (start == null) throw "Expected consumer X handle";
    var drag = layout.begin(AxisX,start.x,start.y), token = host.beginPositionPreview("c102",0);
    var delta = drag.update(start.x+20,start.y); if (delta == null) throw "Expected consumer displacement";
    host.updateWorldDisplacement(token,[delta.x,delta.y,delta.z]);
    var visible = host.displayEntities()[1];
    check(host.source == source && host.revision == 0 && Math.abs(visible.x-initial.x-delta.x)<0.00001 && Math.abs(visible.y-initial.y)<0.00001,"Solver displacement did not feed isolated Documents preview");
    check(Math.abs(visible.localY-(2-delta.x/3))<0.00001,"Consumer drag did not convert rotated/nonuniform parent");
    check(drag.update(Math.NaN,0)==null && host.displayEntities()[1].localY==visible.localY,"Rejected solver update changed document preview");
    drag.finish(); host.commitPositionPreview(token);
    check(host.revision==1 && Math.abs(host.entities()[1].x-visible.x)<0.00001,"Gesture did not commit one matching World pose");
    host.undo(1); check(host.source==source,"Consumer gesture undo lost exact source"); host.dispose();
    Sys.println("SceneKit translation solver to Documents parent-aware preview/commit/undo compatibility passed");
    Sys.println("Pinned SceneKit PerspectiveCamera and frozen gizmo ray/projection compatibility passed");
  }
}
