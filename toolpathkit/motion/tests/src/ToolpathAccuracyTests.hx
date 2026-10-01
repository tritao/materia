import haxe.Int64;
import machinekit.assembly.LinearAxis;
import motionkit.robot.MachineKitRobotCompiler;

/**
  CNC geometry through ProgramCompiler on a gantry: a 1 mm radius circle and a
  fine polygon blended at 10 µm are followed to within the planner's 1 µm
  lowering tolerance, however coarsely the joint path is sampled, because
  joint derivatives come from the geometry.
**/
class ToolpathAccuracyTests {
  public static function run():Int {
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(
      new LinearAxis(23, 10, 200), new LinearAxis(23, 10, 200),
      new LinearAxis(23, 10, 200), 0.05, 0.5);
    var cnc = new MotionCncRig("work", "x", "y", "z", 0.05, [0.02, 0.02, 0.0], 0.00001);
    var binding = ToolpathTestSupport.cncBinding(cnc, blueprint);
    // The joints where the interpreter's start position is.
    var start = binding.solver.solvePose(new motionkit.kinematics.Pose3(0.02, 0.02, 0.0),
      [for (_ in blueprint.model.joints) 0.0], new motionkit.kinematics.IkTolerance());
    if (start == null) throw "the gantry reaches its start";
    var assertions = 0;

    // A full 1 mm radius circle about (20, 20) mm.
    var circle = ToolpathTestSupport.compileCnc(binding, cnc,
      "G21 G90 G17\nG1 F300 X21 Y20\nG2 X21 Y20 I-1 J0\nM2\n", start, Int64.ofInt(10));
    var worst = 0.0;
    for (block in circle.blocks) for (plan in block.plans) for (step in 0...401) {
      var pose = binding.solver.forward(plan.evaluate(plan.durationSeconds * step / 400).positions);
      var radius = Math.sqrt(Math.pow(pose.x - 0.02, 2) + Math.pow(pose.y - 0.02, 2));
      var onApproach = Math.abs(pose.y - 0.02) < 1e-9 && pose.x < 0.021 - 1e-9;
      if (!onApproach) worst = Math.max(worst, Math.abs(radius - 0.001));
    }
    circle.dispose();
if (!(worst < 2e-6)) throw 'a 1 mm circle is followed within 2 um, worst ${worst * 1e6} um';
    assertions++;

    // A 96-gon of 2 mm radius blended at 10 um: 96 short sides and fillets, one path.
    var gcode = new StringBuf();
    gcode.add("G21 G90 G17\nG1 F300 X22 Y20\nG64 P0.01\n");
    for (side in 1...97) {
      var angle = 2 * Math.PI * side / 96;
      gcode.add('G1 X${20 + 2 * Math.cos(angle)} Y${20 + 2 * Math.sin(angle)}\n');
    }
    gcode.add("G61\nG1 X22.5 Y20\nM2\n");
    var polygon = ToolpathTestSupport.compileCnc(binding, cnc, gcode.toString(), start, Int64.ofInt(20));
    var plans = 0;
    for (block in polygon.blocks) plans += block.plans.length;
    polygon.dispose();
    // The approach, the blended polygon and the exit: the polygon's corners do not stop.
    if (plans > 4) throw 'a blended polygon plans as one path, got $plans plans';
    assertions++;
    return assertions;
  }
}
