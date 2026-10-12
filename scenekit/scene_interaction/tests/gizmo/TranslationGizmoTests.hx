import nativekit.scene.gizmo.*;
import nativekit.scene.gizmo.TranslationHandle;

class TranslationGizmoTests {
  static function check(value:Bool, message:String):Void { if (!value) throw message; }
  static function close(value:Float, expected:Float):Bool return Math.abs(value-expected) < 0.000001;
  static function rejects(action:Void -> Void):Void { var failed = false; try action() catch (_:Dynamic) failed = true; check(failed, "Expected gizmo rejection"); }
  static function camera(distance:Float = 10, width:Float = 800, height:Float = 600):GizmoCamera return new GizmoCamera(new GizmoVec3(distance,distance,distance), new GizmoVec3(0,0,0), new GizmoVec3(0,0,1), width,height);
  static function shape(layout:TranslationGizmo, handle:TranslationHandle):TranslationShape {
    for (value in layout.shapes()) if (value.handle == handle) return value; throw "Missing test handle";
  }
  static function center(shape:TranslationShape):GizmoPoint {
    var x = 0.0, y = 0.0, depth = 0.0, vertices = shape.points();
    for (point in vertices) { x += point.x/vertices.length; y += point.y/vertices.length; depth += point.depth/vertices.length; }
    return new GizmoPoint(x,y,depth);
  }
  static function main():Void {
    ScaleGizmoTests.run();
    var rotationView = camera(), rotationPivot=new GizmoVec3(0,0,0);
    for(rotationBasis in [GizmoBasis.world(),GizmoBasis.fromQuaternion(0,0,Math.sin(Math.PI/12),Math.cos(Math.PI/12))]) {
    var rotation=new RotationGizmo(rotationView,rotationPivot,75,rotationBasis);
    check(rotation.shapes().length==3,"Missing rotation rings");
    for(shape in rotation.shapes()) {
      var start:Null<GizmoPoint> = null;
      for(point in shape.points()) if(rotation.pick(point.x,point.y)==shape.handle) {start=point;break;}
      if(start==null) throw "No pickable rotation ring";
      var drag=rotation.begin(shape.handle,start.x,start.y,15);
      check(drag.update(start.x,start.y)==0,"Rotation grab jumped");
      var axis=rotation.basis.axis(shape.handle),ray=rotationView.rayDirection(start.x,start.y);
      var hit=rotationView.eye.add(ray.scaled(axis.dot(rotationPivot.subtract(rotationView.eye))/axis.dot(ray))).subtract(rotationPivot);
      for(angle in [45.0,170.0,190.0,270.0,350.0,370.0]) {
        var rad=angle*Math.PI/180;
        var target=rotationView.project(rotationPivot.add(hit.scaled(Math.cos(rad))).add(axis.cross(hit).scaled(Math.sin(rad))));
        if(target==null) throw "Missing rotation test projection";
        var proposed=drag.update(target.x,target.y);check(proposed!=null && close(proposed,Math.fround(angle/15)*15),"Rotation sign, unwrap or snap failed");
      }
      var last=drag.lastDegrees;check(drag.update(Math.NaN,0)==null && drag.lastDegrees==last,"Invalid rotation replaced valid angle");
      drag.finish();rejects(()->drag.update(start.x,start.y));
    }
    rejects(()->rotation.begin(AxisX,400,300));
    var detached=rotation.shapes();detached.pop();check(rotation.shapes().length==3,"Rotation leaked shapes");
    }
    var view = camera(), pivot = new GizmoVec3(0,0,0), layout = new TranslationGizmo(view,pivot);
    check(layout.shapes().length == 6, "Expected three axes and three plane pads");
    var leaked = layout.shapes(); leaked.pop(); check(layout.shapes().length == 6, "Layout leaked collection");
    var vertices = layout.shapes()[0].points(); vertices.pop(); check(layout.shapes()[0].points().length == 2, "Primitive leaked points");
    for (handle in [AxisX,AxisY,AxisZ,PlaneXY,PlaneXZ,PlaneYZ]) {
      var origin = center(shape(layout, handle));
      check(layout.pick(origin.x,origin.y) == handle, "Draw/pick disagree at handle center");
      for (_ in 0...10) check(layout.pick(origin.x,origin.y) == handle, "Picking is nondeterministic");
      var drag = layout.begin(handle,origin.x,origin.y);
      var zero = drag.update(origin.x,origin.y); check(zero != null && close(zero.x,0) && close(zero.y,0) && close(zero.z,0), "Grab caused movement");
      var axes = switch handle { case AxisX,AxisY,AxisZ: [TranslationGizmo.axis(handle)]; default: TranslationGizmo.planeAxes(handle); };
      var expected = axes[0].scaled(0.5); if (axes.length == 2) expected = expected.add(axes[1].scaled(0.3));
      var startWorld = pivot;
      // Reconstruct the actual initial plane hit, independently, using its ray and known constraint.
      var normal = axes.length == 1 ? view.eye.normalized().subtract(axes[0].scaled(axes[0].dot(view.eye.normalized()))).normalized() : axes[0].cross(axes[1]);
      var ray = view.rayDirection(origin.x,origin.y), t = normal.dot(pivot.subtract(view.eye))/normal.dot(ray);
      startWorld = view.eye.add(ray.scaled(t));
      var target = view.project(startWorld.add(expected)); if (target == null) throw "Test motion not projected";
      var proposed = drag.update(target.x,target.y);
      check(proposed != null && close(proposed.x,expected.x) && close(proposed.y,expected.y) && close(proposed.z,expected.z), "Axis/plane constraint returned wrong World motion");
      drag.update(origin.x,origin.y); check(close(drag.lastDelta.x,0) && close(drag.lastDelta.y,0) && close(drag.lastDelta.z,0), "Drag accumulated previous updates");
      drag.update(target.x,target.y);
      var saved = drag.lastDelta; check(drag.update(Math.NaN,0) == null && drag.lastDelta == saved, "Invalid update discarded last valid proposal");
      check(drag.finish() == saved && drag.closed, "Finish did not return last valid proposal"); rejects(() -> drag.update(0,0)); rejects(() -> drag.finish());
    }
    rejects(() -> new GizmoBasis(new GizmoVec3(2,0,0),new GizmoVec3(0,1,0),new GizmoVec3(0,0,1)));
    rejects(() -> GizmoBasis.fromQuaternion(0,0,0,0));
    rejects(() -> GizmoBasis.fromQuaternion(Math.NaN,0,0,1));
    var basis = GizmoBasis.fromQuaternion(0,0,Math.sin(Math.PI/8),Math.cos(Math.PI/8));
    var localCamera = new GizmoCamera(new GizmoVec3(0,14,10),pivot,new GizmoVec3(0,0,1),800,600);
    var oriented = new TranslationGizmo(localCamera,pivot,90,8,basis);
    check(oriented.shapes().length == 6, "Missing oriented handles");
    for (handle in [AxisX,AxisY,AxisZ,PlaneXY,PlaneXZ,PlaneYZ]) {
      var grab = center(shape(oriented,handle));
      var axes = switch handle { case AxisX,AxisY,AxisZ: [basis.axis(handle)]; default: basis.planeAxes(handle); };
      var eye = localCamera.eye.normalized();
      var normal = axes.length == 1 ? eye.subtract(axes[0].scaled(axes[0].dot(eye))).normalized() : axes[0].cross(axes[1]);
      var ray = localCamera.rayDirection(grab.x,grab.y);
      var start = localCamera.eye.add(ray.scaled(normal.dot(pivot.subtract(localCamera.eye))/normal.dot(ray)));
      var movement = axes[0].scaled(0.7); if (axes.length == 2) movement = movement.add(axes[1].scaled(-0.3));
      var target = localCamera.project(start.add(movement)); if (target == null) throw "Missing oriented target";
      var drag = oriented.begin(handle,grab.x,grab.y,0.5);
      var zero = drag.update(grab.x,grab.y); check(zero != null && close(zero.dot(zero),0), "Oriented grab jumped");
      var delta = drag.update(target.x,target.y);
      var expected = axes[0].scaled(0.5); if (axes.length == 2) expected = expected.add(axes[1].scaled(-0.5));
      check(delta != null && close(delta.x,expected.x) && close(delta.y,expected.y) && close(delta.z,expected.z), "Oriented snapping used World components");
      drag.cancel();
    }
    var followed = layout.translatedShapes(new GizmoVec3(0.5,0.0,0.0));
    check(followed.length == layout.shapes().length && followed[0].handle == layout.shapes()[0].handle, "Following geometry changed captured handles");
    var followedTip = followed[0].points()[1], expectedTip = view.project(new GizmoVec3(layout.worldSize+0.5,0,0));
    check(expectedTip != null && close(followedTip.x,expectedTip.x) && close(followedTip.y,expectedTip.y), "Following geometry ignored absolute World displacement");
    var planeOrigin = center(shape(layout,PlaneXY)), planeDrag = layout.begin(PlaneXY,planeOrigin.x,planeOrigin.y);
    planeDrag.update(planeOrigin.x+10,planeOrigin.y); var valid = planeDrag.lastDelta;
    var tangent = Math.tan(50*Math.PI/360), horizonY = view.height*0.5*(1+view.forward.z/view.up.z/tangent);
    check(planeDrag.update(400,horizonY) == null && planeDrag.lastDelta == valid, "Near-parallel ray replaced valid displacement");
    check(planeDrag.update(400,-100000) == null && planeDrag.lastDelta == valid, "Behind-eye intersection replaced valid displacement");
    check(planeDrag.update(planeOrigin.x,planeOrigin.y) != null, "Constraint did not recover after invalid ray"); planeDrag.cancel();
    var offGrid = new TranslationGizmo(view,new GizmoVec3(0.17,0.29,0.41)), offGridOrigin = center(shape(offGrid,AxisX));
    var offGridDrag = offGrid.begin(AxisX,offGridOrigin.x,offGridOrigin.y,0.5);
    var offGridZero = offGridDrag.update(offGridOrigin.x,offGridOrigin.y);
    check(offGridZero != null && offGridZero.x == 0, "Snapping moved an off-grid pivot on grab"); offGridDrag.cancel();
    var nearView = new GizmoCamera(new GizmoVec3(0,-10,0),pivot,new GizmoVec3(0,0,1),800,600,50,1,20);
    check(new TranslationGizmo(nearView,new GizmoVec3(0,-9.5,0)).shapes().length == 0 && new TranslationGizmo(nearView,new GizmoVec3(0,20,0)).shapes().length == 0, "Depth clipping produced pickable handles");
    check(layout.pick(-1,0) == null && layout.pick(400,300) == null, "Outside/center pixel selected a handle");
    var scaled = new TranslationGizmo(camera(100),pivot);
    for (index in 0...layout.shapes().length) {
      var a = layout.shapes()[index].points(), b = scaled.shapes()[index].points();
      for (vertex in 0...a.length) check(close(a[vertex].x,b[vertex].x) && close(a[vertex].y,b[vertex].y), "Zoom changed logical handle size");
    }
    var highDpi = new TranslationGizmo(camera(10,1600,1200),pivot);
    var a = center(shape(layout,AxisX)), b = center(shape(highDpi,AxisX));
    check(Math.abs((a.x-400)-(b.x-800)) < 3 && Math.abs((a.y-300)-(b.y-600)) < 3, "Viewport sizing changed logical handle extent");
    var front = new GizmoCamera(new GizmoVec3(0,-10,0),pivot,new GizmoVec3(0,0,1),800,600), frontLayout = new TranslationGizmo(front,pivot);
    for (value in frontLayout.shapes()) check(value.handle != AxisY && value.handle != PlaneXY && value.handle != PlaneYZ, "Edge-on constraints remained visible/pickable");
    check(new TranslationGizmo(front,new GizmoVec3(0,-20,0)).shapes().length == 0, "Behind-eye pivot generated handles");
    check(new TranslationGizmo(front,new GizmoVec3(100,0,0)).shapes().length == 0, "Offscreen pivot generated handles");
    var snapOrigin = center(shape(layout,AxisX));
    var snapped = layout.begin(AxisX,snapOrigin.x,snapOrigin.y,0.5);
    var continuous = layout.begin(AxisX,snapOrigin.x,snapOrigin.y);
    var p = new GizmoPoint(snapOrigin.x+15,snapOrigin.y,1), movement = continuous.update(p.x,p.y), quantized = snapped.update(p.x,p.y);
    check(movement != null && quantized != null && close(quantized.x, Math.fround(movement.x/0.5)*0.5), "Snap did not quantize displacement increment");
    snapped.cancel(); rejects(() -> snapped.update(0,0));
    rejects(() -> layout.begin(AxisX,400,300)); rejects(() -> layout.begin(AxisX,snapOrigin.x,snapOrigin.y,-1));
    rejects(() -> new GizmoCamera(pivot,pivot,new GizmoVec3(0,0,1),800,600));
    rejects(() -> new GizmoCamera(new GizmoVec3(0,0,10),pivot,new GizmoVec3(0,0,1),800,600));
    rejects(() -> new TranslationGizmo(view,pivot,Math.NaN));
    Sys.println("Translation gizmo draw/pick, frozen axis/plane constraints, screen sizing, snapping, invalid updates and lifecycle tests passed");
  }
}
