import materia.project.SceneArtifact;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import robotkit.manipulation.Manipulator;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.PathRequest;
import motionkit.kinematics.IkTolerance;
import motionkit.path.PoseMath;

/** Common synthetic TCP line: source frames, existing IK, no hardware performance claim. */
class ReferenceCartesianReview {
  static function check(condition:Bool,message:String):Void {if (!condition) throw message;}

  public static function run(id:String):Void {
    var directory = ReferenceAssetLocation.DIRECTORY+"/"+id;
    var source = new ReferenceRobot(directory);
    var scene = ReferenceArmsProject.scene(id);
    var converted = AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition,AssemblyPhysicalPartView.fromSceneArtifact(scene));
    var tip = [for (f in converted.model.frames) if (f.name == source.tipOccurrence + " robot flange") f][0];
    var arm = new Manipulator(converted.model,converted.model.links[0].id,tip.id,
      new Transform3(new Vec3(0,0,0.15),Quat.identity()));
    var solver = new ReferenceKinematics(arm);
    var tolerance = new IkTolerance(1e-6,1e-6,400,0.005);
    var start = new Pose3(0.45,0.15,0.45,1,0,0,0);
    var initial = solver.solvePose(start,[0.2,-0.8,1.0,0.1,-0.6,0.2],tolerance);
    check(initial != null,'$id cannot reach the common comparison start');
    var poses = [for (i in 0...21) new Pose3(start.x+0.001*i,start.y,start.z,start.qx,start.qy,start.qz,start.qw)];
    var moving = [for (j in source.manifest.joints) if (j.type != "fixed") j];
    var velocity = [for (j in moving) (cast j.limits.velocity:Float)];
    var request = new PathRequest([for (i in 0...21) i*0.001],poses,initial,tolerance,[for (_ in 0...6) 0.1],velocity);
    var path = solver.solvePath(request), samples:Array<Array<Float>> = [];
    var positionError = 0.0, angleError = 0.0, maximumJump = 0.0;
    for (i in 0...path.length) {
      var q = path[i]; check(q != null,'$id lost Cartesian sample $i');
      var actual = solver.forward(q);
      positionError = Math.max(positionError,PoseMath.distance(actual,poses[i]));
      angleError = Math.max(angleError,PoseMath.angle(actual,poses[i]));
      check(positionError<=1e-6 && angleError<=1e-6,"Cartesian FK residual exceeds tolerance");
      for (j in 0...6) {
        var limits = arm.group.limitsOf(j);
        check(q[j]>=limits.lower-1e-8 && q[j]<=limits.upper+1e-8,"Cartesian path exceeds limits");
        if (i>0) maximumJump = Math.max(maximumJump,Math.abs(q[j]-samples[i-1][j]));
      }
      samples.push(q.copy());
    }
    check(maximumJump<=0.1,"Cartesian path changed branch discontinuously");
    var builder = new robotkit.collision.CollisionSceneBuilder(arm,initial,id,0.0005);
    var clearance = builder.clearance(() -> new collisionkit.native.NativeCollisionWorld());
    var violations:Array<{sample:Int,a:String,b:String,distance:Float,required:Float}> = [];
    try {
      for (i in 0...samples.length) {
        var violation = clearance.violation(samples[i]);
        if (violation != null) violations.push({sample:i,a:violation.a,b:violation.b,distance:violation.distance,required:violation.required});
      }
    } catch (error:Dynamic) {clearance.world.dispose(); throw error;}
    clearance.world.dispose();
    var report = {model:id,task:"common 20 mm line, fixed downward TCP orientation",tcpOffsetMetres:0.15,
      tipStatus:source.manifest.tipStatus,samples:samples,maxPositionError:positionError,maxOrientationError:angleError,
      maxJointJump:maximumJump,numericFallbacks:solver.numericFallbacks,collisionViolations:violations,
      clearanceScope:"sampled enclosing source collision hulls; articulation pair policy; no tool body, fixtures or swept proof",
      dynamicQualification:false};
    sys.io.File.saveContent(directory+"/cartesian-report.json",haxe.Json.stringify(report,null,"  "));
    // A prebuilt review artifact plays the checked joint samples slowly. Linear
    // key playback is an illustration, not a jerk-limited hardware trajectory.
    var state = new cadkit.modeling.AssemblyState(scene.assemblyDefinition);
    for (j in 0...6) state.setJoint(moving[j].id,samples[0][j]);
    scene.assemblyState = state.record();
    var loop = samples.copy();
    for (i in 1...samples.length) loop.push(samples[samples.length-1-i].copy());
    scene.project = {dynamicParts:[],motions:[for (j in 0...6) {joint:moving[j].id,loop:true,
      // Application runtime coordinates are relative to the saved initial pose.
      keys:[for (i in 0...loop.length) {time:i*0.3,position:loop[i][j]-samples[0][j]}]}]};
    sys.io.File.saveBytes(directory+"/"+id+"-cartesian.mtrg",SceneArtifact.encode(scene));
    Sys.println('$id Cartesian review: ${samples.length} samples, position error=$positionError m, orientation error=$angleError rad, ${violations.length} sampled hull violations');
  }
}
