package tests;

import robotkit.policy.OnnxPolicy;

/** The ONNX binding: a tiny model with an explicit state, then the G1 policy against Python's answers. */
class OnnxPolicyTests {
  public static function run():Void {
    affineModelCarriesItsState();
    rejectsBadInputs();
    unitreeG1MatchesPython();
    Sys.println("OnnxPolicy tests passed");
  }

  static function near(a:Float, b:Float, tolerance:Float):Bool return Math.abs(a - b) <= tolerance;

  static function affineModelCarriesItsState():Void {
    var policy = OnnxPolicy.load(Files.robotkit("policy/tests/fixtures/affine_state.onnx"));
    if (policy.inputs.length != 2 || policy.input("s").offset != 3 || policy.output("s_out").elements != 2)
      throw "affine model tensors were not described";
    var state = [0.0, 0.0];
    for (step in 1...4) {
      var a = [1.0, 2.0 * step, -1.0];
      var result = policy.run(["a" => a, "s" => state]);
      var y0 = a[0] * 1.0 + a[1] * 0.5 - a[2] * 2.0 + 0.1, y1 = a[0] * 2.0 - a[1] + a[2] * 0.25 - 0.2;
      var y = result.get("y");
      if (!near(y[0], y0, 1e-6) || !near(y[1], y1, 1e-6)) throw 'affine y wrong at step $step: $y';
      state = [state[0] + y0, state[1] + y1];
      var fedBack = result.get("s_out");
      if (!near(fedBack[0], state[0], 1e-5) || !near(fedBack[1], state[1], 1e-5)) throw "affine state wrong";
      state = fedBack;
    }
    policy.dispose();
  }

  static function rejectsBadInputs():Void {
    var policy = OnnxPolicy.load(Files.robotkit("policy/tests/fixtures/affine_state.onnx"));
    expectThrow(() -> policy.run(["a" => [1.0, 2.0, 3.0]]), "missing input");
    expectThrow(() -> policy.run(["a" => [1.0, 2.0], "s" => [0.0, 0.0]]), "wrong size");
    policy.dispose();
    expectThrow(() -> policy.run(["a" => [1.0, 2.0, 3.0], "s" => [0.0, 0.0]]), "disposed policy");
    expectThrow(() -> OnnxPolicy.load("no-such-model.onnx"), "missing file");
  }

  static function expectThrow(action:() -> Void, what:String):Void {
    var threw = false;
    try action() catch (_:Dynamic) threw = true;
    if (!threw) throw 'expected $what to be rejected';
  }

  /** Golden values from onnxruntime in Python (tools/humanoid/policies/make_test_fixtures.py). */
  static function unitreeG1MatchesPython():Void {
    var policy = OnnxPolicy.load(Files.robotkit("tools/humanoid/policies/unitree-g1/policy.onnx"));
    var first = [2.70954037, -0.696337819, 0.151529938, 1.7566756, 2.12212324, 0.383644611, -2.40137696,
      1.48327303, 1.93907154, -0.27001524, 0.95063132, 1.15230751];
    var last = [3.22580767, 1.93022501, -0.342123568, 2.09378767, 3.17845392, 0.889675498, -1.39973164,
      1.8377111, 1.68238902, 1.5478754, 0.570321441, 1.00086355];
    var h = [for (_ in 0...64) 0.0], c = [for (_ in 0...64) 0.0];
    var action:Array<Float> = [];
    for (step in 0...100) {
      var obs = [for (i in 0...47) 0.5 * Math.sin(0.1 * (step * 47 + i))];
      var result = policy.run(["obs" => obs, "h_in" => h, "c_in" => c]);
      action = result.get("action");
      h = result.get("h_out");
      c = result.get("c_out");
      var expected = step == 0 ? first : step == 99 ? last : null;
      if (expected != null)
        for (i in 0...12)
          if (!near(action[i], expected[i], 1e-3)) throw 'G1 action[$i] at step $step: ${action[i]} vs ${expected[i]}';
    }
    if (!near(h[0], 0.0761331394, 1e-3)) throw "G1 recurrent state drifted from Python";
    policy.dispose();
  }
}
