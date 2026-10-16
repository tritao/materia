import materia.project.SceneArtifact;
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import robotkit.manipulation.Manipulator;
import motionkit.robot.EaikKinematics;
import motionkit.robot.ManipulatorKinematics;
import motionkit.path.PoseMath;
import materia.assembly.AssemblyFrames;

class ReferenceArmsPreview {
  static final ROOT:String = ReferenceAssetLocation.DIRECTORY + "/";
  static function check(value:Bool, message:String):Void {if (!value) throw message;}
  public static function main():Void {
    if (Sys.args().indexOf("--braked-preview")>=0) {BrakedReferencePreview.run();return;}
    if (Sys.args().indexOf("--passive-dynamics")>=0) {PassiveDynamicsReview.run();return;}
    if (Sys.args().indexOf("--manufacturer-checks")>=0) {CommercialManufacturerChecks.run();return;}
    if (Sys.args().indexOf("--frozen-control")>=0) {FrozenReferenceControl.run();return;}
    if (Sys.args().indexOf("--arm-design")>=0) {ArmDesignReview.run();return;}
    if (Sys.args().indexOf("--original-review")>=0) {OriginalArmReview.run();return;}
    if (Sys.args().indexOf("--registration-previews")>=0) {CellRegistration.previews();return;}
    if (Sys.args().indexOf("--registration-properties")>=0) {CellRegistration.properties();return;}
    if (Sys.args().indexOf("--registration")>=0) {CellRegistration.run();return;}
    if (Sys.args().indexOf("--comparison")>=0) {ComparisonReview.run();return;}
    if (Sys.args().indexOf("--straight-clearance")>=0) {StraightCurveReview.run();return;}
    if (Sys.args().indexOf("--straight-boundaries")>=0) {StraightBoundaryReview.run();return;}
    var models = [for (arg in Sys.args()) if (!StringTools.startsWith(arg,"--")) arg];
    if (models.length == 0) models = ["ur5e","fairino5_v6"];
    for (id in models) {
      check(~/^[a-zA-Z0-9_]+$/.match(id),"Invalid model ID");
      var source = new ReferenceRobot(ROOT + id);
      // Exercise the exact saved format consumed by the application.
      var bytes = SceneArtifact.encode(source.scene);
      sys.io.File.saveBytes(ROOT+id+"/"+id+".scene",bytes);
      sys.io.File.saveBytes(ROOT+id+"/"+id+".mtrg",bytes);
      var scene = SceneArtifact.decode(bytes);
      var converted = AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition,AssemblyPhysicalPartView.fromSceneArtifact(scene));
      var planningWindows:Array<String> = [];
      for (joint in converted.model.joints) if (joint.type == robotkit.model.JointType.Continuous) {
        // Analytic turn enumeration needs a finite study window. The saved source
        // scene retains a continuous joint with no invented mechanical stop.
        joint.limits.lower = -Math.PI; joint.limits.upper = Math.PI;
        joint.limits.assumptions.push({quantity:"position",label:"Reference comparison planning window +/-pi; continuous source joint"});
        planningWindows.push(joint.name);
      }
      var frame = [for (f in converted.model.frames) if (f.name == source.tipOccurrence + " robot flange") f];
      check(frame.length == 1,"Reference needs exactly one tip frame");
      var arm = new Manipulator(converted.model,converted.model.links[0].id,frame[0].id);
      check(arm.group.count() == 6,"Reference must compile to six moving joints");
      var numeric = new ManipulatorKinematics(arm), solver = new ReferenceKinematics(arm);
      var roundTrips = 0, maxBranches = 0, numericFallbacks = 0;
      var state = new cadkit.modeling.AssemblyState(scene.assemblyDefinition);
      var moving = [for (j in source.manifest.joints) if (j.type != "fixed") j];
      for (sample in 0...source.manifest.probes.length) {
        var probe = source.manifest.probes[sample], q = probe.q;
        for (i in 0...6) state.setJoint(moving[i].id,q[i]);
        for (link in source.manifest.links) {
          var actual = source.linkPose(state,link.id);
          var expected = [for (row in probe.links) if (row.id == link.id) row.pose][0];
          var point = AssemblyFrames.transformPoint(actual,0.017,-0.011,0.007);
          var wanted = AssemblyFrames.transformPoint(expected,0.017,-0.011,0.007);
          check(Math.abs(point.x-wanted.x)<1e-8 && Math.abs(point.y-wanted.y)<1e-8 && Math.abs(point.z-wanted.z)<1e-8,
            '$id: source URDF joint jog differs for ${link.id}');
          var a = AssemblyFrames.toRotationMatrix(actual), b = AssemblyFrames.toRotationMatrix(expected);
          for (i in 0...9) check(Math.abs(a[i]-b[i])<1e-8,"Source link orientation differs");
        }
        var target = numeric.forward(q);
        var f = source.manifest.flangeOffset == null ? probe.tip : AssemblyFrames.compose(probe.tip,source.manifest.flangeOffset);
        var independent = new motionkit.kinematics.Pose3(f.x,f.y,f.z,f.qx,f.qy,f.qz,f.qw);
        check(PoseMath.distance(target,independent)<1e-8 && PoseMath.angle(target,independent)<1e-7,
          '$id: source URDF FK differs at sample $sample');
        var candidates:Array<motionkit.robot.AnalyticIk.AnalyticBranch> = solver.exact == null ? [] : solver.exact.branches(target,q);
        if (candidates.length == 0) {
          var seed = [for (i in 0...6) q[i] + 0.025*Math.sin(i+1)];
          var solved = solver.solvePose(target,seed,new motionkit.kinematics.IkTolerance(1e-6,1e-6,400,0.005));
          check(solved != null,'$id: analytic and numerical inverse lost sample $sample: ${solver.lastFailure}');
          var pose = numeric.forward(solved);
          check(PoseMath.distance(target,pose)<1e-6 && PoseMath.angle(target,pose)<1e-6,"Numerical fallback fails FK");
          for (i in 0...6) {
            var bounds = arm.group.limitsOf(i);
            check(solved[i]>=bounds.lower-1e-8 && solved[i]<=bounds.upper+1e-8,"Numerical fallback violates limits");
          }
          numericFallbacks++;
        }
        maxBranches = Std.int(Math.max(maxBranches,candidates.length));
        for (candidate in candidates) {
          var pose = numeric.forward(candidate.q);
          check(PoseMath.distance(target,pose)<1e-6 && PoseMath.angle(target,pose)<1e-6,"IK branch fails independent compiled FK");
          for (i in 0...6) {
            var bounds = arm.group.limitsOf(i);
            check(candidate.q[i]>=bounds.lower-1e-8 && candidate.q[i]<=bounds.upper+1e-8,"IK branch violates limits");
          }
        }
        roundTrips++;
      }
      var report = {model:id,roundTrips:roundTrips,maxBranches:maxBranches,
        solver:solver.exact == null ? "numeric-fallback" : "EAIK with numerical fallback",unsupportedDiagnostic:solver.unsupportedDiagnostic,
        numericFallbacks:numericFallbacks,continuousPlanningWindows:planningWindows,
        sceneBytes:bytes.length,tipStatus:source.manifest.tipStatus,hardwareConnected:false};
      sys.io.File.saveContent(ROOT+id+"/validation.json",haxe.Json.stringify(report,null,"  "));
      Sys.println(haxe.Json.stringify(report));
      if (Sys.args().indexOf("--cartesian")>=0) ReferenceCartesianReview.run(id);
      var review = ReferenceArmsProject.artifact(id);
      sys.io.File.saveBytes(ROOT+id+"/"+id+"-review.mtrg",review);
    }
  }
}
