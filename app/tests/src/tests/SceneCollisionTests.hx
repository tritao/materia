package tests;

import app.EditorScene;
import app.SceneCollision;
import collisionkit.native.NativeCollisionWorld;

/** The editor's collision world over plain scene objects (COLLISION.md CL8a). */
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
}
