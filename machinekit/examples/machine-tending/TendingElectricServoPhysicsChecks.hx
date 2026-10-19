import robotkit.runtime.*;
import robotkit.model.*;
import robotkit.execution.*;
import haxe.Int64;

/** Analytic runtime trajectories with model gravity bias, using real MuJoCo observations. */
class TendingElectricServoPhysicsChecks {
  public static function main():Void {
    var harness=new SimulationHarness(0.005,10,SimulationSpace.MUJOCO);
    try {
      var model=new RobotModel("gravity-compensated CSP"),base=model.addLink(new Link("base")),load=model.addLink(new Link("load"));
      load.mass=1;var joint=model.addJoint(new Joint("lift",JointType.Prismatic,base,load));
      joint.axis=[0,0,1];joint.limits=new JointLimits(0,1,0.2,50,1);
      var blueprint=RobotRuntimeCompiler.compile(model,new robotkit.profile.RobotProfile());
      blueprint.joints[0].servoStiffness=1000;blueprint.joints[0].servoDamping=63.245553;
      var runtime=harness.simulation.addRobot(blueprint);harness.simulation.setJointPositions(0,[0.1]);
      runtime.submitTargets([robotkit.core.JointTarget.servo(0,0.1,0,1000,63.245553,9.81)],1);
      for(_ in 0...10)harness.step();
      harness.simulation.setServoFeedforward(0,0,9.81);
      var refused=false;try harness.simulation.setServoFeedforward(0,0,51) catch(_:Dynamic)refused=true;
      if(!refused)throw "Servo feedforward exceeded the model effort rating";
      var start=harness.simulation.jointSample(0,0).get_position();
      var delta=0.2-start,T=3.0;
      var segment=new TrajectorySegment(Int64.ofInt(0),Int64.fromFloat(3000000000.0),[[start,0,0,10*delta/Math.pow(T,3),-15*delta/Math.pow(T,4),6*delta/Math.pow(T,5)]]);
      runtime.submitPlan(new ExecutionPlanSubmission(Int64.ofInt(2),Int64.ofInt(blueprint.revision),Int64.ofInt(blueprint.calibrationRevision),
        RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE,[start],[0],[0],[segment],null,null,[0.00001],[0.0001],[0.01],true,[],true),2);
      for(_ in 0...650)harness.step();
      var measured=harness.simulation.jointSample(0,0).get_position();
      if(Math.abs(measured-0.2)>0.00001 || runtime.snapshot().safety!=RobotKitRuntimeConstants.RK_SAFETY_READY)
        throw "Analytic gravity-compensated trajectory did not settle";
      harness.reset();harness.simulation.setJointPositions(0,[0.1]);
      runtime.submitTargets([robotkit.core.JointTarget.servo(0,0.1,0,1000,63.245553,9.81)],3);
      for(_ in 0...10)harness.step();
      var resetStart=harness.simulation.jointSample(0,0).get_position();
      runtime.submitPlan(new ExecutionPlanSubmission(Int64.ofInt(4),Int64.ofInt(blueprint.revision),Int64.ofInt(blueprint.calibrationRevision),
        RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE,[resetStart],[0],[0],
        [new TrajectorySegment(Int64.ofInt(0),Int64.ofInt(1000000000),[[resetStart]])],null,null,[0.00001],[0.0001],[0.01],true,[],true),4);
      for(_ in 0...250)harness.step();
      if(harness.simulation.jointSample(0,0).get_position()>0.095)throw "Simulation reset retained a stale gravity bias";
      harness.reset();harness.simulation.setJointPositions(0,[0.1]);
      runtime.submitTargets([robotkit.core.JointTarget.servo(0,0.1,0,1000,63.245553,9.81)],5);
      for(_ in 0...10)harness.step();harness.simulation.setServoFeedforward(0,0,9.81);
      runtime.submitPlan(new ExecutionPlanSubmission(Int64.ofInt(6),Int64.ofInt(blueprint.revision),Int64.ofInt(blueprint.calibrationRevision),
        RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE,[0.1],[0],[0],
        [new TrajectorySegment(Int64.ofInt(0),Int64.ofInt(2000000000),[[0.1,0,0,0.125,-0.09375,0.01875]])],null,null,[0.00001],[0.0001],[0.01],true,[],true),6);
      for(_ in 0...100)harness.step();var beforeHold=harness.simulation.jointSample(0,0).get_position();runtime.submitHold(7);
      for(_ in 0...400)harness.step();
      var stopped=harness.simulation.jointSample(0,0);
      if(stopped.get_position()>=0.19 || stopped.get_position()<beforeHold-0.00001 || Math.abs(stopped.get_velocity())>0.0001)throw "Analytic motion did not hold before its goal";
      // Contact search must discard its unused tail, not resume it on release.
      runtime.submitAbort(8);for(_ in 0...20)harness.step();
      var cleared=runtime.snapshot();
      if(cleared.trajectoryActive || cleared.trajectoryQueueDepth!=0 || cleared.safety!=RobotKitRuntimeConstants.RK_SAFETY_READY)
        throw "Aborted contact search retained its future trajectory";
      var anchor=cleared.setpoint.get(0),move=0.01;
      runtime.submitPlan(new ExecutionPlanSubmission(Int64.ofInt(9),Int64.ofInt(blueprint.revision),Int64.ofInt(blueprint.calibrationRevision),
        RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE,[anchor],[0],[0],
        [new TrajectorySegment(Int64.ofInt(0),Int64.ofInt(1000000000),[[anchor,0,0,10*move,-15*move,6*move]])],
        null,null,[0.00001],[0.0001],[0.01],true,[],true),9);
      for(_ in 0...250)harness.step();
      if(Math.abs(harness.simulation.jointSample(0,0).get_position()-anchor-move)>0.00001)
        throw "Withdrawal plan after contact-stop cancellation did not execute";
      var result={passed:true,contactStopDiscardsTailAndAcceptsNewPlan:true,controlledHoldStopsBeforeGoal:true,resetClearsBias:true,backend:"MuJoCo",measured:measured,effortLimitEnforced:true,analyticTrajectory:true,longDurationNanoseconds:true};
      sys.io.File.saveContent("build/electric-servo-physics-checks.json",haxe.Json.stringify(result,null,"  "));Sys.println(haxe.Json.stringify(result));
      harness.dispose();
    } catch(error:Dynamic){harness.dispose();throw error;}
  }
}
