import robotkit.model.*;
import robotkit.model.CollisionShape.ContactSurface;
import robotkit.runtime.*;
import nativekit.sim.*;
import robotkit.core.JointTarget;
import robotkit.spatial.Quat;

/** Physical bilateral-contact, release and stop/hold checks in MuJoCo, without CAD or a mission stub. */
class TendingElectricGripperPhysicsChecks {
  public static function main():Void {
    var solver:SimWorldOptions={integrator:2,frictionCone:2,frictionImpedanceRatio:1000,
      noslipIterations:10,solverTolerance:1e-12};
    var harness=new SimulationHarness(0.005,10,SimulationSpace.MUJOCO,solver);
    try {
      var model=new RobotModel("electric parallel jaw physics contract"),base=model.addLink(new Link("base"));
      model.collisionApproximation=CollisionApproximation.None; // Only the authored pads collide; no fallback spheres.
      var wrist=model.addLink(new Link("wrist"));wrist.mass=0.2;wrist.inertiaTensor=[0.001,0,0,0,0.001,0,0,0,0.001];
      var roll=model.addJoint(new Joint("wrist-roll",JointType.Revolute,base,wrist));
      roll.axis=[1,0,0];roll.limits=new JointLimits(-1,1,1,5,5);
      var left=model.addLink(new Link("left")),right=model.addLink(new Link("right"));
      for(link in [left,right]) {
        link.mass=0.1;link.centerOfMass=[0,0,0.03];
        link.inertiaTensor=[0.0000433333333333,0,0,0,0.0000308333333333,0,0,0,0.0000141666666667];
        var shape=new CollisionShape(Box(0.005,0.02,0.03),[0,0,0.03]);
        shape.surface=SimulatedContactSurfaces.gripperFinger(0.8);link.collisionShapes=[shape];
      }
      for(i in 0...2) {
        var joint=model.addJoint(new Joint(i==0 ? "jaw" : "follower",JointType.Prismatic,wrist,i==0 ? left : right));
        joint.parentFramePosition=[i==0 ? -0.018 : 0.018,0,0];joint.axis=[i==0 ? 1.0 : -1.0,0,0];
        joint.limits=new JointLimits(-0.03,0,0.05,70,1);
      }
      model.addCoupling(new JointCoupling("mirror","jaw","follower",1,0));
      var blueprint=RobotRuntimeCompiler.compile(model,new robotkit.profile.RobotProfile());blueprint.processModelJoints.push(1);
      blueprint.joints[0].servoStiffness=1000;blueprint.joints[0].servoDamping=20;
      blueprint.joints[1].servoStiffness=10000;blueprint.joints[1].servoDamping=100;
      var runtime=harness.simulation.addRobot(blueprint);
      var stock=harness.space.session.createObject(MotionType.Dynamic,SimShape.box(0.03,0.03,0.03),
        new SimPose(0,0,0.03),0.1944,null,SimulatedContactSurfaces.environmentPart(),
        new SimInertialProperties([0,0,0],[0.00011664,0,0,0,0.00011664,0,0,0,0.00011664]));
      var floor=harness.spawnPlane();harness.simulation.setJointPositions(0,[0,-0.025,-0.025]);
      runtime.submitTargets([JointTarget.servo(0,0,0,1000,20,0)],1);
      var grip=new SimulatedElectricGripper(harness.simulation,0,1,[2,3],-0.025,0,70);
      grip.command(true);var confirmed=false;
      var earlyContact=false;
      for(_ in 0...2000) {
        harness.step();
        if(_==0 && Math.abs(harness.simulation.jointSample(0,2).get_position()+0.025)>0.0001)throw "Passive follower was not seeded open";
        var bilateral=grip.grippedObjects(harness.simulation.robotContacts(runtime)).indexOf(stock.handle.rawValue())>=0;
        if(bilateral && !grip.commandComplete())earlyContact=true;
        if(bilateral && grip.commandComplete()){confirmed=true;break;}
      }
      if(!earlyContact)throw "Test did not exercise transient contact before completed clamping";
      if(!confirmed)throw "Physical bilateral grasp was not confirmed";
      var worldContacts=harness.space.session.contacts();
      if(worldContacts.length==0)throw "Captured world contact feedback is empty";
      var closed=grip.opening();
      if(Math.abs(closed+0.017)>0.001)throw "Bilateral grasp has the wrong physical width";
      grip.stopAndHold();
      var refused=false;try grip.command(false) catch(_:Dynamic)refused=true;
      if(!refused || !grip.closed)throw "Stop/hold opened the electric gripper";
      var heldPose=harness.simulation.objectPose(stock);
      harness.stop();floor.teleport(new SimPose(0,0,-1));
      // Measure steady holding from a loaded reference after the support-to-grip transfer.
      for(_ in 0...100)harness.step();
      var loadedPose=harness.simulation.objectPose(stock);
      var lx=loadedPose.x-heldPose.x,ly=loadedPose.y-heldPose.y,lz=loadedPose.z-heldPose.z;
      var loadTransferMotion=Math.sqrt(lx*lx+ly*ly+lz*lz);heldPose=loadedPose;
      for(_ in 0...1000)harness.step();
      var afterHold=harness.simulation.objectPose(stock);
      var dx=afterHold.x-heldPose.x,dy=afterHold.y-heldPose.y,dz=afterHold.z-heldPose.z;
      var holdDrift=Math.sqrt(dx*dx+dy*dy+dz*dz);
      if(holdDrift>0.00001)throw "Frictional hold creeps under gravity: "+holdDrift+" m; wrist="+harness.simulation.jointSample(0,0).get_position();
      if(grip.grippedObjects(harness.simulation.robotContacts(runtime)).indexOf(stock.handle.rawValue())<0)throw "Stop lost bilateral grasp";
      var initialWrist=harness.simulation.linkPose(0,1).rotation;
      var initialRelative=new Quat(initialWrist[0],initialWrist[1],initialWrist[2],initialWrist[3]).conjugate()
        .multiply(new Quat(afterHold.qx,afterHold.qy,afterHold.qz,afterHold.qw));
      for(i in 1...2001) {
        var t=i*0.005,angle=0.25*(1-Math.cos(2*Math.PI*t/10)),rate=0.25*(2*Math.PI/10)*Math.sin(2*Math.PI*t/10);
        runtime.submitTargets([JointTarget.servo(0,angle,rate,1000,20,-(0.2+0.1944)*9.81*0.03*Math.sin(angle))],1000+i);
        harness.step();
        if(grip.grippedObjects(harness.simulation.robotContacts(runtime)).indexOf(stock.handle.rawValue())<0) {
          Sys.println(haxe.Json.stringify({tick:i,wrist:harness.simulation.jointSample(0,0).get_position(),opening:grip.opening(),
            safety:runtime.snapshot().safety,stock:harness.simulation.objectPose(stock),contacts:[for(c in harness.simulation.robotContacts(runtime))
              {link:c.linkIndex,object:c.otherObject,distance:c.distance,force:c.force.norm(),active:c.active}]}));
          throw "Wrist roll lost the physical grasp";
        }
      }
      var transported=harness.simulation.objectPose(stock);
      var finalWrist=harness.simulation.linkPose(0,1).rotation;
      var relative=new Quat(finalWrist[0],finalWrist[1],finalWrist[2],finalWrist[3]).conjugate()
        .multiply(new Quat(transported.qx,transported.qy,transported.qz,transported.qw));
      var dot=Math.abs(relative.x*initialRelative.x+relative.y*initialRelative.y+relative.z*initialRelative.z+relative.w*initialRelative.w);
      var rollDrift=2*Math.acos(Math.min(1,dot))*180/Math.PI;
      Sys.println("Wrist roll drift degrees: "+rollDrift);
      if(rollDrift>0.1)throw "Wrist-roll drift exceeds the handling model's orientation budget";
      // Separate instance is an explicit new run, never automatic fault recovery.
      var release=new SimulatedElectricGripper(harness.simulation,0,1,[2,3],-0.025,0,70,0.005,0);release.command(false);
      var released=false;
      for(_ in 0...2000){harness.step();if(release.released(harness.simulation.robotContacts(runtime),stock.handle.rawValue())){released=true;break;}}
      if(!released)throw "Physical release was not confirmed";
      if(runtime.snapshot().safety==RobotKitRuntimeConstants.RK_SAFETY_FAULT)throw "Gripper physics runtime faulted";
      release.dispose();grip.dispose();
      var releasedPosition=release.opening();
      harness.stop();stock.teleport(new SimPose(2,0,0.03));
      var empty=new SimulatedElectricGripper(harness.simulation,0,1,[2,3],-0.025,-0.017,70,0.005,releasedPosition);empty.command(true);
      for(_ in 0...500) {
        harness.step();if(empty.grippedObjects(harness.simulation.robotContacts(runtime)).length!=0)throw "Empty close falsely confirmed a grasp";
      }
      empty.dispose();
      var result={passed:true,backend:"MuJoCo",solver:solver,missingStockNotGrasped:true,bilateralContact:true,stopRetainedGrasp:true,releaseConfirmed:true,
        wristRollDriftDegrees:rollDrift,loadTransferMotionMm:loadTransferMotion*1000,gravityHoldSeconds:5,gravityHoldDriftMm:holdDrift*1000,passiveFollowerSeeded:true,clampRampComplete:grip.commandComplete(),transientContactBeforeClamp:earlyContact,measuredGraspCoordinateM:closed,worldContacts:worldContacts.length,scope:"Parallel-jaw physics contract; simplified pads, simulation force/friction assumptions"};
      sys.io.File.saveContent("build/electric-gripper-physics-checks.json",haxe.Json.stringify(result,null,"  "));
      Sys.println(haxe.Json.stringify(result));
      harness.dispose();
    } catch(error:Dynamic){harness.dispose();throw error;}
  }
}
