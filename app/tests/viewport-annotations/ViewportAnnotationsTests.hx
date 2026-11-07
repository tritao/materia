import app.EditorScene;
import app.EditorPerspectiveViewport;
import app.ViewportAnnotations;
import app.ViewportRobotAnnotations;
import app.SensorConfiguration;
import haxeon.platform.event.EventPump;
import haxeon.ui.host.UiHostContext;
import haxeon.ui.Canvas;
import haxeon.ui.DisplayList;
import robotkit.core.SensorFrame;
import haxe.Int64;

/** Exercise actual annotation painting, posed occlusion and authored inspection. */
@:access(app.EditorPerspectiveViewport)
class ViewportAnnotationsTests {
  static var assertions = 0;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value)
      throw message;
  }

  static function commands(viewport:EditorPerspectiveViewport, collision:Bool = false):Int {
    var canvas = new Canvas();
    if (collision) viewport.paintCollisions(canvas, 800, 600);
    else viewport.paintSensors(canvas, 800, 600);
    var list = DisplayList.create();
    canvas.update(list);
    var count = list.info().commandCount;
    list.dispose();
    return count;
  }

  public static function main():Int {
    var scene = new EditorScene([
      {
        id: "occluder",
        label: "Enclosure",
        type: "rectangle",
        x: 0.0,
        y: 0.0,
        z: 0.0,
        width: 1.0,
        height: 1.0,
        depth: 1.0,
        collisionEnabled: false,
        dynamicBody: false,
        mass: 1.0,
        red: 0.5,
        green: 0.5,
        blue: 0.5,
        visible: true
      }
    ]);
    var host = new UiHostContext(null, new EventPump(), function() {}, function() {});
    var viewport = new EditorPerspectiveViewport("annotations-test", scene, host);
    viewport.viewportWidth = 800;
    viewport.viewportHeight = 600;
    viewport.camera.frame(0, 0, 0, 3, 3, 3, 800 / 600);
    var frame = new SensorFrame("probe", "lidar", "probe-frame", Int64.ofInt(1), Int64.ofInt(0), [0.25], null, "base", [0.0, 0.0, 0.0],
      [0.0, 0.0, 0.0, 1.0]);
    var robots:Array<app.ApplicationSimulation.SimulationRobotVisual> = [
      {
        id: "robot",
        position: [0.0, 0.0, 0.0],
        rotation: [0.0, 0.0, 0.0, 1.0],
        sensors: [frame],
        links: [{id: "base", position: [0.0, 0.0, 0.0], rotation: [0.0, 0.0, 0.0, 1.0]}]
      }
    ];
    try {
      viewport.setSimulationState(true, [], 1, robots);
      check(commands(viewport) == 0, "Play alone must not reveal sensor or link markers");
      viewport.setAnnotations(new ViewportAnnotations(true, true));
      check(commands(viewport) == 0, "enclosed sensor mounts respect occlusion");
      viewport.setAnnotations(new ViewportAnnotations(true, true, false, false, true));
      check(commands(viewport) > 0, "explicit show-through reveals enclosed mounts and axes");
      viewport.setAnnotations(new ViewportAnnotations(true, false, false, false, true, "other", "probe"));
      check(commands(viewport) == 0, "sensor identity includes its owning robot");
      viewport.setAnnotations(new ViewportAnnotations(true, false, false, false, true, "robot", "probe"));
      check(commands(viewport) > 0, "inspection reveals only the selected sensor without a global toggle");
      var key = viewport.paintKey();
      viewport.setAnnotations(new ViewportAnnotations());
      check(viewport.paintKey() != key, "selection changes invalidate retained annotation paint");
      check(commands(viewport) == 0, "leaving inspection restores a clean viewport");
      viewport.setAnnotations(new ViewportAnnotations(true, true, true, true, true));
      viewport.setSimulationOverlays(false);
      check(commands(viewport) == 0, "the overlay master switch hides mounts, frames and readings");
      viewport.setSimulationOverlays(true);
      viewport.setAnnotations(new ViewportAnnotations(false, true, true, true, true));
      check(commands(viewport) == 0, "disabled policy suppresses all annotation categories");
      viewport.setAnnotations(new ViewportAnnotations(true, false, false, true, true));
      check(commands(viewport) > 0, "link frames can be enabled independently");
      viewport.setAnnotations(new ViewportAnnotations(true, false, true, false, true));
      check(commands(viewport) > 0, "readings can be enabled without mount markers");

      var sensors = new SensorConfiguration();
      var authored = ViewportRobotAnnotations.authored(sensors);
      check(authored.length == 1 && authored[0].sensors.length == 1, "authored mounts exist before Play");
      check(authored[0].sensors[0].reading == null, "authored inspection never fabricates observations");
      viewport.setSimulationState(false, [], 2);
      viewport.setAnnotationRobots(authored, "authored:1");
      viewport.setAnnotations(new ViewportAnnotations(true, false, true, false, true));
      check(commands(viewport) == 0, "no readings are drawn before measurements exist");
      viewport.setAnnotations(new ViewportAnnotations(true, true, false, false, true));
      check(commands(viewport) > 0, "authored mounts and orientation work independently of Play");
      var beforeMountEdit = viewport.paintKey();
      viewport.setAnnotationRobots(authored, "authored:2");
      check(viewport.paintKey() != beforeMountEdit, "authored mount edits invalidate retained paint without camera movement");
      sensors.dispose();
      var configured = authored[0].sensors[0];
      var beforeReading = ViewportRobotAnnotations.measured([
        {
          id: authored[0].id,
          position: authored[0].position,
          rotation: authored[0].rotation,
          links: authored[0].links,
          sensors: []
        }
      ], authored);
      check(beforeReading[0].sensors[0].id == configured.id && beforeReading[0].sensors[0].reading == null,
        "live mounts are inspectable before their first observation");
      check(!ViewportAnnotations.panelVisible(haxeon.ui.docking.DockNode.Tabs(["sensors", "machine"], "machine"), "sensors"),
        "a background Sensors tab does not reveal markers");
      check(ViewportAnnotations.panelVisible(haxeon.ui.docking.DockNode.Tabs(["sensors", "machine"], "sensors"), "sensors"),
        "a visible Sensors tab enables contextual inspection");
      var measured = ViewportRobotAnnotations.measured(robots);
      check(measured[0].sensors[0].reading == frame, "measured annotation keeps the immutable observation");
      viewport.setCollisions([new app.SceneCollision.SceneCollisionPair("occluder", "other", 0.001,
        [0.0, 0.0, 0.0], [0.1, 0.0, 0.0], false)]);
      viewport.setAnnotations(new ViewportAnnotations());
      scene.select("scene");
      check(commands(viewport, true) == 0, "ordinary viewing hides amber proximity markers");
      scene.select("occluder");
      check(commands(viewport, true) == 0, "selected proximity markers respect geometry occlusion");
      viewport.setAnnotations(new ViewportAnnotations(true, false, false, false, true));
      check(commands(viewport, true) > 0, "selection reveals the related closest points with explicit show-through");
      scene.select("scene");
      check(commands(viewport, true) == 0, "deselection clears collision inspection aids");
      viewport.setAnnotations(new ViewportAnnotations(true, false, false, false, true, null, null, true));
      check(commands(viewport, true) > 0, "Collision details explicitly reveals every pair");
      viewport.setSimulationOverlays(false);
      check(commands(viewport, true) == 0, "the master overlay switch also hides collision markers");
      viewport.setSimulationOverlays(true);
      check(viewport.collisionPairCount() == 1, "hiding annotation details retains collision diagnostics");
      check(ViewportAnnotations.unobstructed(1, null, 0.001), "empty space is visible");
      check(ViewportAnnotations.unobstructed(1, 0.9999, 0.001), "surface roundoff does not hide a mount");
      check(!ViewportAnnotations.unobstructed(1, 0.5, 0.001), "a nearer surface hides a mount");
    } catch (error:Dynamic) {
      viewport.dispose();
      scene.dispose();
      throw error;
    }
    viewport.dispose();
    scene.dispose();
    Sys.println('Viewport annotations: $assertions assertions passed');
    return 0;
  }
}
