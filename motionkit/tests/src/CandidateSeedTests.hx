import motionkit.robot.ManipulatorKinematics;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.IkTolerance;
import motionkit.path.OrientationPolicy;
import robotkit.manipulation.KinematicGroup;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Frame;

class CandidateSeedTests {
  static var checks = 0;
  static function check(value:Bool, message:String):Void {
    checks++;
    if (!value) throw message;
  }
  public static function main():Void {
    for (count in [3,6,7]) {
      var model = new RobotModel('seed-coverage-$count');
      var links = [for (i in 0...count+1) model.addLink(new Link('link-$i'))];
      for (i in 0...count) {
        var joint = model.addJoint(new Joint('joint-$i', JointType.Revolute, links[i], links[i+1]));
        joint.limits.lower = -1; joint.limits.upper = 1;
      }
      var frame = model.addFrame(new Frame("tip", links[count]));
      var solver = new SeedRecorder(new KinematicGroup(model, links[0].id, frame.id));
      var requested = count == 3 ? 1 : 12;
      var limit = count == 3 ? 27 : count == 6 ? 96 : 32;
      solver.sampleCandidates(new Pose3(0,0,0,0,0,0,1), requested, new IkTolerance());
      check(solver.seeds.length == limit + 2*count, "the bounded seed budget and near-end probes are applied");
      var unique = new Map<String,Bool>();
      for (i in 0...limit) unique.set(solver.seeds[i].join(":"),true);
      var distinct = 0;
      for (_ in unique.keys()) distinct++;
      check(distinct == limit, "Product seeds never repeat; the three-joint lattice stays complete");
      for (joint in 0...count) {
        var low = 0, middle = 0, high = 0;
        for (i in 0...limit) {
          var value = solver.seeds[i][joint];
          if (value == -0.5) low++; else if (value == 0) middle++; else if (value == 0.5) high++;
          else throw "Product seed is outside the original three levels";
        }
        check(low > 0 && middle > 0 && high > 0, 'Budgeted discovery covers every level of joint $joint');
      }
    }
    var sevenAxis = new RobotModel("seed-coverage-seven-axis");
    var sevenLinks = [for (i in 0...8) sevenAxis.addLink(new Link('seven-link-$i'))];
    for (i in 0...7) {
      var joint = sevenAxis.addJoint(new Joint('seven-joint-$i', JointType.Revolute, sevenLinks[i], sevenLinks[i+1]));
      joint.limits.lower = -1; joint.limits.upper = 1;
    }
    var sevenFrame = sevenAxis.addFrame(new Frame("seven-tip", sevenLinks[7]));
    var screen = new SeedRecorder(new KinematicGroup(sevenAxis, sevenLinks[0].id, sevenFrame.id));
    screen.sampleCandidates(new Pose3(0,0,0,0,0,0,1), 1, new IkTolerance());
    check(screen.seeds.length == 8 + 14, "A redundant-arm feasibility screen uses eight seeds and endpoint probes");
    Sys.println('Candidate seed tests passed ($checks assertions)');
  }
}

private class SeedRecorder extends ManipulatorKinematics {
  public final seeds:Array<Array<Float>> = [];
  public function new(group:KinematicGroup) super(group);
  override public function solvePose(target:Pose3, seed:Array<Float>, tolerance:IkTolerance,
      ?freedom:OrientationPolicy):Null<Array<Float>> {
    seeds.push(seed.copy());
    return null;
  }
}
