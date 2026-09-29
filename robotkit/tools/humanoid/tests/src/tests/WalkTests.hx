package tests;

import haxe.Int64;
import humanoid.WalkRun;
import humanoid.WalkRun.WalkResult;
import humanoid.WalkRun.WalkScenario;
import robotkit.world.McapRecordingReader;
import robotkit.world.RobotCommand;
import robotkit.world.JointTarget;
import robotkit.world.JointTargetMode;

/**
 * H4 acceptance: Unitree's pretrained G1 policy, run through the ordinary
 * RobotKit runtime in MuJoCo on IMU and encoder observations, stands, walks at
 * 1 m/s for 30 s, turns, survives pushes, and records to MCAP and replays.
 *
 * The bounds are what the policy does in Unitree's own plain-MuJoCo run
 * (tools/humanoid/policies/reference_unitree_g1.py), which agrees with these
 * runs to a few percent; they are not tighter than the policy itself.
 */
class WalkTests {
  public static function run():Void {
    stands();
    walksOneMetrePerSecondForThirtySeconds();
    turns();
    survivesPushes();
    recordsToMcapAndReplays();
    Sys.println("Walk tests passed");
  }

  static function scenario(seconds:Float, vx:Float, vy:Float, wz:Float, ?record:String):WalkScenario
    return {
      model: Files.robotkit("tests/fixtures/humanoid/unitree-g1-12dof.robot.json"),
      spec: Files.robotkit("tools/humanoid/policies/unitree-g1/policy.json"),
      seconds: seconds, commands: [{from: 0.0, vx: vx, vy: vy, wz: wz}], pushes: [], record: record,
      truthObservations: false, report: 0.0, warmup: 0.0
    };

  static function require(condition:Bool, what:String):Void if (!condition) throw what;

  static function summary(result:WalkResult):String {
    var last = result.samples[result.samples.length - 1];
    return 'end (${round(last.x)}, ${round(last.y)}, z ${round(last.z)}), yaw ${round(last.yaw)}, worst tilt ${round(result.worstTilt)}, '
      + 'fell ${result.fell}${result.fell ? " at " + result.fallTime : ""}';
  }

  static function round(value:Float):Float return Math.round(value * 1000.0) / 1000.0;

  static function uprightAtEnd(result:WalkResult, what:String):Void {
    var last = result.samples[result.samples.length - 1];
    require(!result.fell && last.z > 0.72 && last.tilt < 0.25, '$what: not upright at the end: ${summary(result)}');
  }

  /** The IMU-based gravity estimate must track the true one closely: it is what the policy sees. */
  static function estimateIsClose(result:WalkResult, what:String):Void
    require(result.worstEstimateError < 0.02, '$what: the gravity estimate was ${result.worstEstimateError} rad off');

  static function stands():Void {
    var result = WalkRun.run(scenario(10.0, 0.0, 0.0, 0.0));
    uprightAtEnd(result, "stand");
    var last = result.samples[result.samples.length - 1];
    require(result.worstTilt < 0.25 && Math.sqrt(last.x * last.x + last.y * last.y) < 1.0, 'stand drifted: ${summary(result)}');
    estimateIsClose(result, "stand");
    Sys.println('stand: ${summary(result)}');
  }

  static function walksOneMetrePerSecondForThirtySeconds():Void {
    var result = WalkRun.run(scenario(30.0, 1.0, 0.0, 0.0));
    uprightAtEnd(result, "walk");
    var mean = WalkRun.meanVelocity(result, 5.0, 30.0);
    // Plain MuJoCo reaches 0.87 m/s for this command; the run must be walking, not shuffling.
    require(mean.vx > 0.75 && mean.vx < 1.15, 'walk: mean forward speed ${round(mean.vx)} m/s for a 1 m/s command');
    require(Math.abs(mean.vy) < 0.3, 'walk: mean sideways speed ${round(mean.vy)} m/s');
    var last = result.samples[result.samples.length - 1];
    require(last.x > 20.0, 'walk: covered only ${round(last.x)} m in 30 s');
    estimateIsClose(result, "walk");
    Sys.println('walk 1 m/s, 30 s: mean vx ${round(mean.vx)} m/s, ${summary(result)}');
  }

  static function turns():Void {
    // The policy turns at about a third of the commanded rate, in plain MuJoCo too.
    for (turn in [{wz: 0.5, sign: 1.0}, {wz: -0.5, sign: -1.0}]) {
      var walking = WalkRun.run(scenario(10.0, 0.5, 0.0, turn.wz));
      uprightAtEnd(walking, 'turn ${turn.wz} while walking');
      var yaw = walking.samples[walking.samples.length - 1].yaw;
      require(yaw * turn.sign > 1.0 && yaw * turn.sign < 2.6, 'walking turn ${turn.wz}: heading changed by ${round(yaw)} rad');
      estimateIsClose(walking, "walking turn");
      var inPlace = WalkRun.run(scenario(10.0, 0.0, 0.0, turn.wz));
      uprightAtEnd(inPlace, 'turn ${turn.wz} in place');
      var inPlaceYaw = inPlace.samples[inPlace.samples.length - 1].yaw;
      require(inPlaceYaw * turn.sign > 0.8 && inPlaceYaw * turn.sign < 2.6, 'in-place turn ${turn.wz}: heading changed by ${round(inPlaceYaw)} rad');
      Sys.println('turn ${turn.wz}: walking ${round(yaw)} rad, in place ${round(inPlaceYaw)} rad in 10 s');
    }
  }

  /**
   * A push is a horizontal force on the pelvis for 0.2 s: 100 N adds about 0.6 m/s to a 33 kg robot. The
   * policy shrugs that off in every direction, standing or walking. 200 N forward or sideways is more than
   * it survives standing, in Unitree's plain MuJoCo run as well (see the plan).
   */
  static function survivesPushes():Void {
    for (push in [[100.0, 0.0], [-100.0, 0.0], [0.0, 100.0], [0.0, -100.0]]) {
      for (walking in [false, true]) {
        var s = scenario(9.0, walking ? 0.5 : 0.0, 0.0, 0.0);
        s.pushes.push({at: 4.0, force: [push[0], push[1], 0.0], duration: 0.2});
        var result = WalkRun.run(s);
        uprightAtEnd(result, 'push ${push} ${walking ? "walking" : "standing"}');
        Sys.println('push $push ${walking ? "walking" : "standing"}: ${summary(result)}');
      }
    }
    // The push really lands: an unpushed robot ends elsewhere than a pushed one.
    var quiet = WalkRun.run(scenario(6.0, 0.0, 0.0, 0.0));
    var pushed = scenario(6.0, 0.0, 0.0, 0.0);
    pushed.pushes.push({at: 3.0, force: [-200.0, 0.0, 0.0], duration: 0.2});
    var shoved = WalkRun.run(pushed);
    uprightAtEnd(shoved, "backward 200 N push");
    var q = quiet.samples[quiet.samples.length - 1], p = shoved.samples[shoved.samples.length - 1];
    require(p.x < q.x - 0.5, 'a 200 N push moved the robot only ${round(q.x - p.x)} m');
  }

  static function recordsToMcapAndReplays():Void {
    var path = '/tmp/robotkit-${Sys.getPid()}-g1-walk.mcap';
    var walk = scenario(6.0, 0.5, 0.0, 0.0, path);
    var result = WalkRun.run(walk);
    uprightAtEnd(result, "recorded walk");
    var recording = McapRecordingReader.load(path);
    // One snapshot per 2 ms tick, one servo batch per 20 ms control period and one to start.
    require(recording.snapshots.length == 3000, 'recorded ${recording.snapshots.length} snapshots');
    require(recording.commands.length == 301, 'recorded ${recording.commands.length} commands');
    var first:Array<JointTarget> = [], last:Array<JointTarget> = [];
    var seen = false;
    for (command in recording.commands)
      switch command {
        case JointTargets(targets, _):
          for (target in targets) require(target.mode == JointTargetMode.Servo, "a recorded joint target lost its servo mode");
          if (!seen) first = targets;
          seen = true;
          last = targets;
        case _: throw "unexpected command in a policy recording";
      }
    require(first.length == 12 && first[3].stiffness == 150.0 && first[3].damping == 4.0 && first[3].target == 0.3,
      "the recorded stand command must carry the default pose and its gains");
    require(last[0].target != first[0].target || last[3].target != first[3].target, "the policy's targets were not recorded");
    // Replaying the recorded servo targets into a fresh simulation, with no policy, gives the same run.
    var replayed = WalkRun.replay(walk, recording);
    require(replayed.steps == 3000, 'replayed ${replayed.steps} steps');
    require(replayed.worstJoint < 1e-9 && replayed.worstTime == 0.0,
      'replay differs from the recording by ${replayed.worstJoint} rad');
    Sys.println('recorded ${recording.snapshots.length} snapshots and ${recording.commands.length} servo batches; replay differs by ${replayed.worstJoint} rad');
    for (leftover in [path, path + ".incomplete.status"])
      if (sys.FileSystem.exists(leftover)) sys.FileSystem.deleteFile(leftover);
  }
}
