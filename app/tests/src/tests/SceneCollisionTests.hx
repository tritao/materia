package tests;

import app.AppSettings;
import app.EditorPerspectiveViewport;
import app.EditorScene;
import app.Main.ReferenceEditorApp;
import app.SceneCollision;
import collisionkit.native.NativeCollisionWorld;
import haxeon.platform.NativeKitEvents;
import haxeon.ui.properties.PropertyValue;
import haxeon.ui.host.UiHostContext;
import nativekit.scene.SceneView;
import nativekit.scene.Transform;

/** The editor's collision world over plain scene objects (COLLISION.md CL8a) and how it is shown (CL8b). */
@:access(app.Main.ReferenceEditorApp)
@:access(app.EditorPerspectiveViewport)
class SceneCollisionTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;

  static function box(id:String, x:Float):app.SceneObjectData
    return {id: id, label: id, type: "rectangle", x: x, y: 0.0, z: 0.05, width: 0.1, height: 0.1, depth: 0.1,
      collisionEnabled: true, dynamicBody: false, mass: 1.0, red: 0.5, green: 0.5, blue: 0.5, visible: true};

  public static function main():Int {
    try {
      run();
      Sys.println("Scene collision tests passed");
      return 0;
    } catch (error:Dynamic) {
      Sys.println('Scene collision tests failed: $error');
      return 1;
    }
  }

  static function run():Void {
    world();
    shown();
    editor();
  }

  static function world():Void {
    // Two 100 mm cubes, the second 80 mm along X: they overlap by 20 mm.
    var scene = new EditorScene([box("a", 0.0), box("b", 0.08)]);
    var collision = new SceneCollision(() -> new NativeCollisionWorld());
    var pairs = collision.query(scene, null, 0.01);
    check(pairs.length == 1 && pairs[0].a == "a" && pairs[0].b == "b" && pairs[0].colliding(),
      'overlapping boxes collide (${pairs.length} pairs)');
    check(Math.abs(pairs[0].distance + 0.02) < 1e-6, 'the overlap is 20 mm deep (${pairs[0].distance})');
    check(collision.query(scene, null, 0.01) == pairs, "an unchanged scene is not queried again");

    // 5 mm apart: near, not colliding; the world is only re-posed.
    scene.setPosition("b", 0, 0.105);
    pairs = collision.query(scene, null, 0.01);
    check(pairs.length == 1 && !pairs[0].colliding() && Math.abs(pairs[0].distance - 0.005) < 1e-6,
      'boxes 5 mm apart are near (${pairs.length} pairs)');
    check(Math.abs(pairs[0].pointB[0] - pairs[0].pointA[0] - 0.005) < 1e-6, "the closest points span the gap");

    // Face to face, as a box laid on another: touching, shown as near, not colliding.
    scene.setPosition("b", 0, 0.1);
    pairs = collision.query(scene, null, 0.01);
    check(pairs.length == 1 && !pairs[0].colliding() && Math.abs(pairs[0].distance) < 1e-6,
      'flush boxes touch without colliding (${pairs.length} pairs)');
    check(collision.query(scene, null, 0.01, 0.0).length == 1, "a zero contact tolerance still finds the pair");

    // 50 mm apart: clear.
    scene.setPosition("b", 0, 0.15);
    check(collision.query(scene, null, 0.01).length == 0, "boxes 50 mm apart are clear");
    check(collision.builds == 1, 'moving objects never rebuilds the world (${collision.builds} builds)');

    // Turning collision off on one object removes the pair and rebuilds once.
    scene.setPosition("b", 0, 0.08);
    check(collision.query(scene, null, 0.01).length == 1, "moved back, the boxes collide again");
    for (item in scene.items()) if (item.id == "b") item.collisionEnabled = false;
    scene.setPosition("b", 0, 0.081);
    check(collision.query(scene, null, 0.01).length == 0 && collision.builds == 2,
      "an object without collision is left out, by a rebuild");
    collision.dispose();
  }

  /** CL8b: red and amber tints in the render view, the summary line, and the viewport's repaint key. */
  static function shown():Void {
    var scene = new EditorScene([box("a", 0.0), box("b", 0.08), box("c", 0.3), box("d", 0.405)]);
    var collision = new SceneCollision(() -> new NativeCollisionWorld());
    var pairs = collision.query(scene, null, 0.01);
    check(pairs.length == 2 && pairs[0].colliding() && !pairs[1].colliding(), "one colliding pair and one near pair");
    check(SceneCollision.summary(pairs, id -> id.toUpperCase()) == "1 collision, 1 near · A / B 20 mm deep",
      'the toolbar line names the closest pair (${SceneCollision.summary(pairs, id -> id)})');
    check(SceneCollision.summary([], id -> id) == null, "nothing to report, no line");
    function tinted(?with:Array<app.SceneCollision.SceneCollisionPair>):Int
      return scene.configureRenderView(new SceneView(), Transform.identity(), null, null, -1, with).materialOverrideValues().length;
    scene.select("scene");
    check(tinted(pairs) == 4, 'all four objects are tinted (${tinted(pairs)})');
    scene.select("a");
    check(tinted(pairs) == 3, "the selected object keeps its selection highlight");
    check(tinted() == 0, "without pairs nothing is tinted");

    var host = new UiHostContext(null, new NativeKitEvents(), function() {}, function() {});
    var viewport = new EditorPerspectiveViewport("collision-test", scene, host);
    var before = viewport.presentationKey();
    viewport.setCollisions(pairs);
    var shownKey = viewport.presentationKey();
    check(shownKey != before && viewport.collisionPairCount() == 2, "new pairs change the viewport's key");
    viewport.setCollisions(pairs);
    check(viewport.presentationKey() == shownKey, "the same query result keeps the rendered image");
    collision.dispose();
  }

  /** CL8b in the editor: the setting turns it on and off, and a simulation hides it. */
  static function editor():Void {
    var app = new ReferenceEditorApp();
    var ids:Array<String> = [];
    for (x in [0.0, 0.08]) {
      check(app.scene.createRectangle(), "the editor creates a box");
      var id = app.scene.selectedId;
      app.scene.setDimensions(id, 0.1, 0.1, 0.1);
      app.scene.setPosition(id, 0, x);
      app.scene.setPosition(id, 1, 0.0);
      ids.push(id);
    }
    app.scene.select(null);
    var pairs = app.editingCollisions();
    check(pairs.length == 1 && app.collisionSummary != null && app.collisionSummary.indexOf("1 collision") == 0,
      'the editor finds the overlap (${app.collisionSummary})');
    app.preferences.store.set(AppSettings.COLLISIONS_VISIBLE, PropertyValue.Bool(false));
    check(!app.collisionsVisible && app.editingCollisions().length == 0 && app.collisionSummary == null,
      "turned off, nothing is shown");
    app.commands.execute("scene.toggle-collisions");
    check(app.collisionsVisible && app.editingCollisions().length == 1, "the command turns it back on");
    app.preferences.store.set(AppSettings.COLLISION_NEAR, PropertyValue.Float(0.0));
    app.scene.setPosition(ids[1], 0, 0.105);
    check(app.editingCollisions().length == 0, "a 0 mm near distance shows only collisions");
    app.dispose();
  }
}
