package tests;

import nativekit.scene.PerspectiveCamera;

/** Off-screen exploded parts must not consume the visible model's depth precision. */
@:access(app.EditorPerspectiveViewport) class CameraClipRangeTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
  static function viewportBounds():Void {
    var scene = new app.EditorScene([]);
    scene.createRectangle();
    var id = scene.selectedId;
    scene.setDimensions(id, 1, 1, 1);
    var host = new haxeon.ui.host.UiHostContext(null, new haxeon.platform.event.EventPump(), () -> {}, () -> {});
    var viewport = new app.EditorPerspectiveViewport("clip-bounds-test", scene, host);
    try {
      viewport.updateViewportSize(800, 600);
      viewport.camera.setAngle(0.3, 0);
      viewport.camera.frame(0, 0, 0, 4, 3, 2, 4.0 / 3.0);
      viewport.fitCameraClipRange();
      var originalNear = viewport.camera.clipNear, originalFar = viewport.camera.clipFar;
      var back = viewport.camera.viewDirection();
      scene.setPositionXY(id, back[0], back[1]);
      viewport.fitCameraClipRange();
      var movedNear = viewport.camera.clipNear;
      check(movedNear < originalNear - 0.5, "scene edits update displayed clip bounds");
      scene.setVisible(id, false);
      viewport.fitCameraClipRange();
      check(viewport.camera.clipFar > originalFar * 10, "hidden geometry leaves the clip range");
      scene.setVisible(id, true);
      viewport.fitCameraClipRange();
      check(Math.abs(viewport.camera.clipNear - movedNear) < 1e-6, "showing geometry restores its clip bounds");
      viewport.setSimulationState(true, [{id:id, position:[0, 0, 0], rotation:[0, 0, 0, 1]}], 1);
      viewport.fitCameraClipRange();
      check(Math.abs(viewport.camera.clipNear - originalNear) < 1e-6, "runtime poses replace authored clip bounds");
      viewport.setSimulationState(true, [{id:id, position:[for (value in back) value * 2], rotation:[0, 0, 0, 1]}], 2);
      viewport.fitCameraClipRange();
      check(viewport.camera.clipNear < movedNear - 0.5, "new runtime revisions update clip bounds");
      viewport.setSimulationState(false, [], 3);
      viewport.camera.orbit(30, 20);
      viewport.camera.pan(20, -10, 600);
      viewport.fitCameraClipRange();
      var world = scene.info(id).bounds(), expected = new PerspectiveCamera();
      expected.setAngle(viewport.camera.yaw, viewport.camera.pitch);
      expected.setTarget(viewport.camera.targetX, viewport.camera.targetY, viewport.camera.targetZ, viewport.camera.distance);
      expected.fitClipRange([[world.minX, world.minY, world.minZ, world.maxX, world.maxY, world.maxZ]], 4.0 / 3.0);
      check(Math.abs(viewport.camera.clipNear - expected.clipNear) < 1e-6
        && Math.abs(viewport.camera.clipFar - expected.clipFar) < 1e-6,
        "camera motion refits clipping against the current world bounds");
    } catch (error:Dynamic) {
      viewport.dispose(); scene.dispose(); throw error;
    }
    viewport.dispose(); scene.dispose();
  }
  /** A close-up on a hole rim in an exploded assembly: the exploded frame's box encloses the eye. */
  static function explodedCloseUp():Void {
    var camera = new PerspectiveCamera();
    camera.setAngle(0.6, 0.35);
    // Pivot on a 6 mm hole rim of a bracket exploded 0.4 m off a 2 m frame (metres).
    camera.setTarget(0.4, 0.1, 0.55, 0.08);
    var bracket = [0.37, 0.07, 0.53, 0.43, 0.13, 0.57];
    var frame = [-1.0, -0.4, 0.0, 1.0, 0.4, 0.9];
    var fasteners = [0.2, -0.1, 0.45, 0.6, 0.3, 0.75];
    camera.fitClipRange([bracket, frame, fasteners], 16.0 / 9.0);
    var step = camera.depthStepAt(camera.distance);
    // The previous distance/10000 fallback gave ~48 um steps here (0.6 um now); edges lost to faces at hole rims.
    check(step <= camera.distance * PerspectiveCamera.PIVOT_RELATIVE_DEPTH_STEP * 1.001,
      'exploded close-up keeps pivot depth precision (step ${step} m)');
    check(camera.clipNear < camera.distance * 0.01, "exploded close-up keeps geometry well ahead of the pivot");
    var eye = camera.eyePosition(), back = camera.viewDirection(), farthest = 0.0;
    for (corner in 0...8) {
      var depth = 0.0;
      for (axis in 0...3) depth -= (frame[(corner >> axis & 1) * 3 + axis] - eye[axis]) * back[axis];
      farthest = Math.max(farthest, depth);
    }
    check(camera.clipFar >= farthest, "exploded close-up keeps the far side of the frame");
  }

  public static function main():Int {
    viewportBounds();
    var camera = new PerspectiveCamera();
    for (yaw in [0.0, 0.7, 13.241878906249955]) for (pitch in [-0.8, 0.0, 0.46328857421874864]) {
      camera.setAngle(yaw, pitch);
      camera.setTarget(-0.838, 0.073, 0.176, 0.05);
      var eye = camera.eyePosition(), back = camera.viewDirection(), right = [-Math.sin(yaw), Math.cos(yaw), 0.0];
      function box(depth:Float, side:Float, radius:Float):Array<Float> {
        var center = [for (i in 0...3) eye[i] - back[i] * depth + right[i] * side];
        return [
          center[0] - radius,
          center[1] - radius,
          center[2] - radius,
          center[0] + radius,
          center[1] + radius,
          center[2] + radius
        ];
      }
      var visible = box(20, 0, 0.5), crossing = box(0, 100, 0.5), distant = box(1000, 10000, 0.5);
      for (aspect in [0.5, 1.0, 2.0]) {
        camera.fitClipRange([visible], aspect);
        var near = camera.clipNear, far = camera.clipFar;
        camera.fitClipRange([visible, crossing, distant, box(-10, 0, 0.5)], aspect);
        check(
          Math.abs(camera.clipNear - near) < 1e-9 && Math.abs(camera.clipFar - far) < 1e-9,
          "off-screen/behind-eye boxes preserve the visible depth range"
        );
      }
      camera.fitClipRange([visible]);
      var visibleNear = camera.clipNear;
      camera.fitClipRange([visible, crossing]);
      check(camera.clipNear < visibleNear * 0.5, "fixture straddles the eye in depth without entering the view");
      check(camera.depthStepAt(camera.distance) <= camera.distance * PerspectiveCamera.PIVOT_RELATIVE_DEPTH_STEP * 1.001,
        "boxes straddling the eye keep the pivot's depth precision");
      var side = box(10, 8, 0.2);
      camera.fitClipRange([visible, side], 1);
      var narrowNear = camera.clipNear;
      camera.fitClipRange([visible, side], 2);
      check(camera.clipNear < narrowNear * 0.75, "wide aspect retains side geometry");
      camera.fitClipRange([visible, side], 1, 90);
      check(camera.clipNear < narrowNear * 0.75, "inspection lens FOV retains side geometry");
      camera.fitClipRange([visible, box(10, Math.tan(25 * Math.PI / 180) * 10, 1)], 1);
      check(camera.clipNear < narrowNear * 0.75, "boxes crossing a side plane stay included");
      camera.fitClipRange([visible, box(0, 0, 0.5)], 1);
      check(camera.clipNear < camera.distance * 0.01, "camera inside an on-screen box retains geometry ahead of the pivot");
      camera.fitClipRange([crossing], 1);
      check(camera.clipNear > 0 && camera.clipFar > camera.clipNear, "fully off-screen scene keeps valid clipping");
    }
    // The saved column-mold camera, its off-screen bed, and two visible former bounds.
    camera.setAngle(13.241878906249955, 0.46328857421874864);
    camera.setTarget(-0.8380172100843716, 0.0726464048317293, 0.1761653914041284, 0.05);
    camera.fitClipRange(
      [
        [-0.266, -0.879, -0.022, 0.266, 0.046, -0.004],
        [-1.0304913847, -0.0899, 0.0271, -0.9506913847, -0.0101, 0.0929],
        [-1.0304913847, -0.0899, 0.1671, -0.9506913847, -0.0101, 0.2329]
      ],
      1085.0 / 689.0
    );
    check(camera.clipNear > 0.1 && camera.clipFar < 0.5, "saved mold view keeps its useful depth precision");
    explodedCloseUp();
    Sys.println('Camera clip range: $assertions assertions passed');
    return 0;
  }
}
