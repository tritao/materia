import materia.project.SceneArtifact;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.SimulationHarness;
import robotkit.runtime.SimulationSpace;
import robotkit.model.JointType;

/** Isolate dissipation in the exact loaded geometry; do not modify saved scene or vendor parameters. */
class PassiveDynamicsReview {
  static function digest(path:String):String {
    var hash=haxe.crypto.Sha256.make(sys.io.File.getBytes(path)),out=new StringBuf();
    for(i in 0...hash.length) out.add(StringTools.hex(hash.get(i),2).toLowerCase());
    return out.toString();
  }

  public static function run():Void {
    var root=ReferenceAssetLocation.DIRECTORY;
    var path=root+"/arm-design/zeroerr-original.mtrg";
    var reports:Array<Dynamic> = [];
    for(test in [{id:"zero_joint_losses",damping:0.0,dt:0.01},
                 {id:"diagnostic_damping_1",damping:1.0,dt:0.01},
                 {id:"diagnostic_damping_5",damping:5.0,dt:0.01},
                 {id:"zero_joint_losses_half_timestep",damping:0.0,dt:0.005},
                 {id:"zero_joint_losses_1ms",damping:0.0,dt:0.001},
                 {id:"zero_joint_losses_half_ms",damping:0.0,dt:0.0005}]) {
      var scene=SceneArtifact.decode(sys.io.File.getBytes(path));
      var converted=AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition,AssemblyPhysicalPartView.fromSceneArtifact(scene),scene.assemblyState);
      var moving=[for(j in converted.model.joints) if(j.type!=JointType.Fixed) j];
      if(moving.length!=6) throw "Expected six original joints";
      var original=[for(j in moving) {id:j.id,passive:j.passive,damping:j.damping,frictionLoss:j.frictionLoss,armature:j.armature}];
      for(j in moving) {if(!j.passive) throw "Expected passive exported joint";j.damping=test.damping;}
      converted.model.collisionApproximation=robotkit.model.CollisionApproximation.None;
      for(link in converted.model.links) link.collisionShapes=[];
      var blueprint=RobotRuntimeCompiler.compile(converted.model,converted.profile,1);
      blueprint.linkCollisionShapes.resize(0);
      var harness=new SimulationHarness(test.dt,1,SimulationSpace.MUJOCO);
      var samples:Array<Dynamic> = [],failure:Null<String> = null;
      var robot=harness.simulation.addRobot(blueprint);
      var duration=60.0,count=Std.int(duration/test.dt),stride=Std.int(0.1/test.dt);
      try {
        for(step in 0...count) {
          harness.step();
          if(step%stride==0 || step==count-1) {
            var snapshot=robot.snapshot(),q=snapshot.q.toArray(),velocity=snapshot.dq.toArray();
            for(value in q.concat(velocity)) if(!Math.isFinite(value)) throw "Nonfinite passive state";
            var maxVelocity=0.0;for(value in velocity) maxVelocity=Math.max(maxVelocity,Math.abs(value));
            samples.push({t:(step+1)*test.dt,q:q,velocity:velocity,maxVelocityRadS:maxVelocity,faultCode:snapshot.faultCode});
          }
        }
      } catch(error:Dynamic) failure=Std.string(error);
      harness.dispose();
      var lateMaximum=0.0,earlyMaximum=0.0;
      for(sample in samples) {
        if(sample.t>=50) lateMaximum=Math.max(lateMaximum,sample.maxVelocityRadS);
        if(sample.t<=10) earlyMaximum=Math.max(earlyMaximum,sample.maxVelocityRadS);
      }
      var totalMass=0.0;for(link in converted.model.links) totalMass+=link.mass;
      reports.push({id:test.id,sourceJointProperties:original,diagnosticDampingNmPerRadS:test.damping,timestepSeconds:test.dt,
        seconds:duration,early10SecondsMaxVelocityRadS:earlyMaximum,last10SecondsMaxVelocityRadS:lateMaximum,
        convertedSceneMassKg:totalMass,brakes:blueprint.brakes.length,actuators:converted.model.actuators.length,
        failure:failure,samples:samples});
      Sys.println(test.id+": early max="+earlyMaximum+" rad/s; late max="+lateMaximum+" rad/s; failure="+failure);Sys.stdout().flush();
    }
    var directory=root+"/passive-dynamics";if(!sys.FileSystem.exists(directory)) sys.FileSystem.createDirectory(directory);
    sys.io.File.saveContent(directory+"/review.json",haxe.Json.stringify({
      scope:"MuJoCo 60-second contact-free isolation using the loaded original scene state. No floor/self contacts; diagnostic damping values are not vendor data. No saved geometry/scene changes or hardware commands.",
      sourceSha256:digest(path),
      producerSha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(root+"/../PassiveDynamicsReview.hx")),
      nativeModuleSha256:digest(Sys.programPath()),cases:reports},null,"  "));
  }
}
