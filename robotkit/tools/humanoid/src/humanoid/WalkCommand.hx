package humanoid;

/**
 * `walk <robot.json> <policy.json> <seconds> <vx> <vy> <wz> [--push <at-s> <fx> <fy> <duration-s>]
 *  [--record <file.mcap>] [--warmup <s>] [--truth]`: runs a policy on a humanoid and prints where it goes.
 * Exits nonzero when the robot falls.
 */
class WalkCommand {
  public static function run(args:Array<String>):Void {
    if (args.length < 6) {
      Sys.println("usage: walk <robot.json> <policy.json> <seconds> <vx> <vy> <wz> [--push <at-s> <fx> <fy> <duration-s>] "
        + "[--record <file.mcap>] [--warmup <s>] [--truth]");
      Sys.exit(2);
    }
    var scenario:WalkScenario = {
      model: args[0], spec: args[1], seconds: Std.parseFloat(args[2]),
      commands: [{from: 0.0, vx: Std.parseFloat(args[3]), vy: Std.parseFloat(args[4]), wz: Std.parseFloat(args[5])}],
      pushes: [], record: null, truthObservations: false, report: 1.0, warmup: 0.0
    };
    var i = 6;
    while (i < args.length) {
      switch args[i] {
        case "--push":
          scenario.pushes.push({at: Std.parseFloat(args[i + 1]), force: [Std.parseFloat(args[i + 2]), Std.parseFloat(args[i + 3]), 0.0],
            duration: Std.parseFloat(args[i + 4])});
          i += 5;
        case "--record":
          scenario.record = args[i + 1];
          i += 2;
        case "--warmup":
          scenario.warmup = Std.parseFloat(args[i + 1]);
          i += 2;
        case "--truth":
          scenario.truthObservations = true;
          i++;
        case other:
          Sys.println('unknown option $other');
          Sys.exit(2);
      }
    }
    var result = WalkRun.run(scenario);
    var last = result.samples[result.samples.length - 1];
    Sys.println('ran ${last.t} s: ended at (${last.x}, ${last.y}, ${last.z}), yaw ${last.yaw}, worst tilt ${result.worstTilt} rad, '
      + 'worst gravity-estimate error ${result.worstEstimateError} rad (mean bias ${result.meanEstimateError}), ${result.clampedTargets} targets clamped into joint travel, fell: ${result.fell}');
    if (result.fell) {
      Sys.println('FAIL: fell at t=${result.fallTime}');
      Sys.exit(1);
    }
    Sys.println("PASS: stayed up");
  }
}
