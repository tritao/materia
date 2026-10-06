import motionkit.robot.RevoluteReachBound;
import motionkit.robot.ManipulatorKinematics;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import robotkit.manipulation.KinematicGroup;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Frame;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

class RevoluteReachBoundTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }

  public static function main():Void {
    var model = new RobotModel("reach-bound");
    var links = [for (i in 0...6) model.addLink(new Link('link-$i'))];
    var kinds = [JointType.Fixed, JointType.Revolute, JointType.Fixed,
      JointType.Continuous, JointType.Fixed];
    for (i in 0...5) {
      var joint = model.addJoint(new Joint('joint-$i', kinds[i], links[i], links[i+1]));
      joint.parentFramePosition = [0.1, 0.02, 0.03];
      joint.childFramePosition = [-0.02, 0.01, 0.04];
      joint.parentFrameRotation = Quat.fromRollPitchYaw(0.1, 0.2, 0.3).toArray();
      joint.childFrameRotation = Quat.fromRollPitchYaw(-0.2, 0.1, -0.3).toArray();
    }
    var flange = model.addFrame(new Frame("flange", links[5]));
    flange.position = [0.08, -0.02, 0.05];
    var group = new KinematicGroup(model, links[0].id, flange.id, null,
      new Transform3(new Vec3(0.04, 0.03, 0.02), Quat.fromRollPitchYaw(0.2, -0.1, 0.4)));
    var bound = RevoluteReachBound.of(group);
    check(bound != null, "revolute chain has a bound");
    for (i in 0...41) for (j in 0...41) {
      var pose = group.tcpPose([-Math.PI + i*Math.PI/20, -Math.PI + j*Math.PI/20]);
      check(!bound.excludes(pose, 0, 0, true), "fixed spans and rotated joint frames preserve full reach");
      check(!bound.excludes(pose, 0, 0, false), "free orientation never excludes generated TCP");
      var spun = new Transform3(pose.translation,
        pose.rotation.multiply(Quat.fromRollPitchYaw(0,0,0.7)));
      check(!bound.excludes(spun, 0, 0, false, true), "free tool spin retains generated TCP");
    }
    var far = new Transform3(new Vec3(10, 0, 0), Quat.identity());
    check(bound.excludes(far, 0.001, 0.001, true), "distant full pose excluded");
    check(bound.excludes(far, 0.001, 0.001, false), "distant free pose excluded");
    var solver = new ManipulatorKinematics(group);
    var tolerance = new IkTolerance(0.00005, 0.001);
    check(solver.solvePose(new Pose3(10,0,0,0,0,0,1), [0,0], tolerance) == null,
      "adapter rejects unreachable pose");
    check(solver.lastFailure == "Tool pose is outside the revolute chain reach bound",
      "adapter reports geometric rejection");
    check(solver.sampleCandidates(new Pose3(10,0,0,0,0,0,1), 12, tolerance).length == 0,
      "global discovery rejects impossible seeds");

    var simple = new RobotModel("orientation-bound");
    var base = simple.addLink(new Link("base"));
    var elbow = simple.addLink(new Link("elbow"));
    var tip = simple.addLink(new Link("tip"));
    simple.addJoint(new Joint("shoulder", JointType.Revolute, base, elbow));
    var wrist = simple.addJoint(new Joint("wrist", JointType.Revolute, elbow, tip));
    wrist.parentFramePosition = [0.6,0,0];
    var tool = simple.addFrame(new Frame("tool", tip));
    tool.position = [0.4,0,0];
    var simpleGroup = new KinematicGroup(simple, base.id, tool.id);
    var simpleBound = RevoluteReachBound.of(simpleGroup);
    var reversed = new Transform3(new Vec3(1,0,0), Quat.fromRollPitchYaw(0,0,Math.PI));
    check(simpleBound.excludes(reversed, 0, 0, true), "orientation places wrist outside reach");
    check(!simpleBound.excludes(reversed, 0, 0, false), "free orientation retains reachable TCP");
    check(!simpleBound.excludes(reversed, 0, 0, false, true), "free spin retains perpendicular tool offset");
    check(!simpleBound.excludes(simpleGroup.tcpPose([0,0]), 0, 0, true), "straight boundary accepted");
    var outside = new Transform3(new Vec3(1.0001,0,0), Quat.identity());
    check(!simpleBound.excludes(outside, 0.0002, 0, true), "position tolerance enlarges bound");
    var axialTool = simple.addFrame(new Frame("axial-tool", tip));
    axialTool.position = [0,0,0.4];
    var axial = RevoluteReachBound.of(new KinematicGroup(simple, base.id, axialTool.id));
    var axisAway = new Transform3(new Vec3(0.8,0,0), Quat.fromRollPitchYaw(0,-Math.PI/2,0));
    check(!axial.excludes(axisAway, 0, 0, false), "unrestricted orientation retains TCP sphere");
    check(axial.excludes(axisAway, 0, 0, false, true), "fixed tool axis excludes unreachable wrist");
    var sliding = new RobotModel("sliding");
    var rail = sliding.addLink(new Link("rail"));
    var carriage = sliding.addLink(new Link("carriage"));
    sliding.addJoint(new Joint("slide", JointType.Prismatic, rail, carriage));
    var slidingTool = sliding.addFrame(new Frame("sliding-tool", carriage));
    check(RevoluteReachBound.of(new KinematicGroup(sliding, rail.id, slidingTool.id)) == null,
      "prismatic chain keeps unrestricted solver behavior");
    check(RevoluteReachBound.of(new KinematicGroup(simple, base.id, tool.id, tool.id)) == null,
      "moving work frame keeps unrestricted solver behavior");
    projectedPrefixes();
    Sys.println('Revolute reach bound tests passed ($assertions assertions)');
  }

  static function projectedPrefixes():Void {
    for (variant in 0...3) {
      var model = new RobotModel('projected-prefix-$variant');
      var links = [for (i in 0...7) model.addLink(new Link('link-$i'))];
      var offsets = [[0.0,0,0.1], [0.0,0.135,0], [0.0,-0.12,0.425],
        [0.0,0,0.392], [0.0,0.109,0], [0.0,0,0.095]];
      var axes = [[0.0,0,1], [0.0,1,0], [0.0,1,0], [1.0,0,0], [0.0,1,0], [0.0,0,1]];
      if (variant == 1) axes[2] = [0.3,0.95,0.2];
      for (i in 0...6) {
        var joint = model.addJoint(new Joint('joint-$i', JointType.Revolute, links[i], links[i+1]));
        joint.axis = Vec3.fromArray(axes[i]).normalized().toArray();
        joint.parentFramePosition = offsets[i];
        if (variant == 2) {
          joint.childFramePosition = [0.01,-0.02,0.03];
          joint.childFrameRotation = Quat.fromRollPitchYaw(0.1,0.2,-0.3).toArray();
        }
      }
      var frame = model.addFrame(new Frame("tip", links[6]));
      frame.position = [0,0.08,0];
      var group = new KinematicGroup(model, links[0].id, frame.id);
      var bound = RevoluteReachBound.of(group);
      for (sample in 0...2000) {
        var q = [for (joint in 0...6) Math.sin((sample+1)*(joint+2)*0.617)*Math.PI];
        var actual = group.tcpPose(q);
        check(!bound.excludes(actual,0,0,true), 'Projected prefix retains full FK pose ($variant)');
        check(!bound.excludes(actual,0,0,false,true), 'Projected prefix retains axis FK pose ($variant)');
      }
      if (variant == 0)
        check(bound.excludes(new Transform3(new Vec3(1.1,0.08,0.1), Quat.identity()),0,0,true),
          "Axial offsets cancel: reject a pose inside the old triangle-only sphere");
    }
  }
}
