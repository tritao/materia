import materia.project.SceneArtifact;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import robotkit.model.JointType;
import motionkit.robot.JointPathTiming;
import motionkit.robot.JointMotionPolicy;
import machinekit.control.erob.ERobDriveStub;

/** Existing model/timing stack + simulation-only eRob boundary, with explicit commissioning gaps. */
class FrozenReferenceControl {
  static final DIRECTORY=ReferenceAssetLocation.DIRECTORY+"/../../zeroerr-arm";
  static function read(path:String):Dynamic return haxe.Json.parse(sys.io.File.getContent(path));
  static function check(ok:Bool,message:String):Void {if(!ok) throw message;}
  public static function run():Void {
    var profile=read(DIRECTORY+"/reference-control-profile.json");
    var build=DIRECTORY+"/build/reference-control";
    if(!sys.FileSystem.exists(build)) sys.FileSystem.createDirectory(build);
    var stub=new ERobDriveStub(),denied=false;
    try ERobDriveStub.requireHardwareReady(profile) catch(_:Dynamic) denied=true;
    check(denied,"Uncommissioned profile reached hardware");
    stub.configure(6);stub.command(0.0,[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]);
    var rejected=false;
    try stub.command(0.0,[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]) catch(_:Dynamic) rejected=true;
    check(rejected,"Stub accepted stale command");stub.close();
    if(Sys.args().indexOf("--replay-approved")>=0) {
      var approved=read(build+"/approved-commands.json");
      check(approved.simulationOnly==true && approved.passed==true,"Missing approved simulation trajectory");
      check(approved.profileSha256==haxe.crypto.Sha256.encode(sys.io.File.getContent(DIRECTORY+"/reference-control-profile.json")),"Changed drive profile");
      check(approved.timedSourceSha256==haxe.crypto.Sha256.encode(sys.io.File.getContent(build+"/timed-paths.json")),"Changed timed trajectory");
      for(name in Reflect.fields(approved.inputs))
        check(Reflect.field(approved.inputs,name)==haxe.crypto.Sha256.encode(sys.io.File.getContent(DIRECTORY+"/"+name)),"Changed approved dynamics/curve dependency: "+name);
      var runs:Array<Dynamic> = approved.runs,feedbacks:Array<Dynamic> = [],total=0;
      for(run in runs) {
        stub=new ERobDriveStub();stub.configure(6);
        var samples:Array<Dynamic> = run.commands;
        for(sample in samples) stub.command(sample.t,sample.q,sample.velocity,sample.torqueNm);
        feedbacks.push({id:run.id,feedback:stub.feedback()});total+=samples.length;stub.close();
      }
      sys.io.File.saveContent(build+"/replay.json",haxe.Json.stringify({commands:total,runs:feedbacks,hardwareConnected:false,stubOnly:true}));
      Sys.println('Replayed $total approved timed SI commands through eRob stub in four independent runs');return;
    }
    var scene=ComparisonProject.source("zeroerr-original");
    var converted=AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition,AssemblyPhysicalPartView.fromSceneArtifact(scene));
    var joints=[for(j in converted.model.joints) if(j.type!=JointType.Fixed) j];
    check(joints.length==6,"Frozen reference requires six joints");
    var maxima:Array<Float> = profile.maximumVelocityDegS;
    var acceleration:Array<Float> = profile.rampDerivedAccelerationCeilingDegS2;
    var window:Array<Float> = profile.simulationJointWindowRad;
    var rates:Array<motionkit.robot.JointMotionPolicy.JointMotionRates> = [];
    for(i in 0...6) {
      joints[i].limits.lower=window[0];joints[i].limits.upper=window[1];
      joints[i].limits.velocity=Math.min(maxima[i],profile.requestedVelocityDegS)*Math.PI/180;
      joints[i].limits.maxAcceleration=Math.min(acceleration[i],profile.requestedAccelerationDegS2)*Math.PI/180;
      joints[i].limits.assumptions.push({quantity:"position",label:"Diagnostic simulation window; vendor assembled/harness limits pending"});
      joints[i].limits.assumptions.push({quantity:"velocity",label:Std.string(profile.velocityBasis)});
      rates.push({velocity:joints[i].limits.requireVelocity(),acceleration:joints[i].limits.requireAcceleration(),jerk:profile.requestedJerkDegS3*Math.PI/180});
    }
    var source=read(ReferenceAssetLocation.DIRECTORY+"/arm-design/zeroerr-original-native.json");
    var tasks:Array<Dynamic> = source.paths,output:Array<Dynamic> = [];
    for(task in tasks) {
      check(task.complete==true && task.continuous.clear==true,"Missing original joint-path clearance");
      var path:Array<Array<Float>> = task.path;
      for(q in path) for(i in 0...6) check(q[i]>=window[0] && q[i]<=window[1],"Path outside diagnostic window");
      var timing=new JointPathTiming(rates),trajectory=timing.trajectory(path),samples:Array<Dynamic> = [];
      var duration=trajectory.durationSeconds(),count=Std.int(Math.ceil(duration/profile.commandPeriodSeconds));
      for(n in 0...count+1) {
        var t=Math.min(duration,n*profile.commandPeriodSeconds),state=trajectory.evaluate(t);
        samples.push({t:t,q:state.positions,velocity:state.velocities,acceleration:state.accelerations});
      }
      output.push({id:task.id,durationSeconds:duration,samples:samples,clearanceBasis:"Existing original-arm straight joint-edge certificate; timing stops at each knot"});
      trajectory.dispose();
    }
    sys.io.File.saveContent(build+"/timed-paths.json",haxe.Json.stringify({simulationOnly:true,
      profileSha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(DIRECTORY+"/reference-control-profile.json")),
      sourceSha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(ReferenceAssetLocation.DIRECTORY+"/arm-design/zeroerr-original-native.json")),
      timingProducerSha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(ReferenceAssetLocation.DIRECTORY+"/../FrozenReferenceControl.hx")),
      paths:output,hardwareConnected:false}));
    Sys.println("Timed four original-arm paths using JointPathTiming; hardware remains unavailable");
  }
}
