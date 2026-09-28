package tests;

import app.Main.ReferenceEditorApp;
import app.examples.TwoRobotSetupScript;
import FontCollection;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import sys.FileSystem;

/** A humanoid character preview takes part in the application simulation as a person. */
class HumanSimulationTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;

  public static function main():Int {
    try {
      run();
      Sys.println("Human simulation tests passed");
      return 0;
    } catch (error:Dynamic) {
      Sys.println("Human simulation tests failed: " + Std.string(error));
      return 1;
    }
  }

  public static function run():Void {
    var directory = "build/human-simulation-test";
    if (!FileSystem.exists(directory)) FileSystem.createDirectory(directory);
    var fonts = FontCollection.create();
    fonts.add(requirePath(["uikit/vendor/harfbuzz/perf/fonts/Roboto-Regular.ttf",
      "../../uikit/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf",
      "../../../uikit/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf"]));
    var editor = new ReferenceEditorApp(fonts, directory + "/workspace.json", null, null, null,
      TwoRobotSetupScript.REFERENCE);
    editor.enableCharacterPreview(requirePath(["animkit/assets/quaternius/worker.glb",
      "../../animkit/assets/quaternius/worker.glb", "../animkit/assets/quaternius/worker.glb"]));
    editor.tick();
    check(editor.simulation.pending(editor.sensors, editor.scene), "a new person requires a rebuild");
    check(editor.simulation.rebuild(editor.sensors, editor.scene, editor.session),
      "the simulation builds with a person: " + editor.simulation.error);
    var session = editor.simulation.activeSession();
    check(session != null, "the application simulation has a session");
    var live:SimSession = cast session;

    // The walk starts 2.5 m along +X; a ray along +Y at chest height meets the person.
    var start = torsoDistance(live);
    check(start > 2.4 && start < 3.0, 'the person does not stand at the start of the walk: $start');

    // Frames without simulation ticks leave the person where the physics has them.
    for (_ in 0...5) editor.tick();
    check(Math.abs(torsoDistance(live) - start) < 1e-9, "the person moved without the simulation advancing");

    // Half a second of ticks: the person walks on along the circle, towards +Y.
    for (_ in 0...50) {
      editor.simulation.step();
      editor.tick();
    }
    var walked = torsoDistance(live);
    check(walked > start + 0.3, 'the person did not walk with simulation time: $start then $walked');
    editor.dispose();
  }

  static function torsoDistance(session:SimSession):Float
    return session.raycast(new SimPose(2.5, -3.0, 1.0), 0.0, 1.0, 0.0, 10.0);

  static function requirePath(candidates:Array<String>):String {
    for (candidate in candidates) if (FileSystem.exists(candidate)) return candidate;
    throw "Missing test file: " + candidates[0];
  }
}
