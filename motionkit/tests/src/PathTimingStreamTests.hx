import RobotKitRuntime;
import motionkit.planner.JointPathSamples;
import motionkit.planner.PathTimingLimits;
import motionkit.planner.PathTimingStream.TimingStreamStep;
import motionkit.planner.ToppraPathTiming;
import motionkit.trajectory.Trajectory;
import motionkit.robot.TimedMotionBuffer;
import motionkit.robot.TimedMotionTransforms.MotionFilterSettings;
import motionkit.trajectory.ShaperImpulses;
import haxe.Int64;

class PathTimingStreamTests extends MotionKitTestSupport {
  function exercise():Void {
    var geometry = new JointPathSamples([0.0,0.5,1.0], [[0.0],[0.5],[1.0]],
      [[1.0],[1.0],[1.0]], [[0.0],[0.0],[0.0]]);
    var stream = new ToppraPathTiming().prepareStream(geometry,new PathTimingLimits([2.0],[1.0]),4);
    geometry.q[1][0] = 99;
    switch stream.step() {
      case Ready(stale):
        switch stream.step() {
          case Ready(current):
            var refused = false;
            try stale.commitTiming() catch (_:Dynamic) refused = true;
            check(refused,"superseded Haxe preview cannot commit");
            stale.dispose();
            var timed = current.peek();
            near(timed.trajectory.evaluate(0.0).positions[0],0.0,"preview owns its input snapshot",1e-10);
            near(timed.trajectory.durationSeconds(),2.0,"stream preserves the analytic triangular clock",2e-9);
            var middle = Trajectory.nanoseconds(1.0);
            var left = timed.trajectory.sliceTicks(Trajectory.nanoseconds(0.0),middle);
            var right = timed.trajectory.sliceTicks(middle,Trajectory.nanoseconds(timed.trajectory.durationSeconds()));
            left.controlAcceleration = [0.6]; right.controlAcceleration = [0.4];
            var joined = Trajectory.concatenate([left,right]);
            left.dispose(); right.dispose();
            near(joined.controlAcceleration[0],0.4,"native concatenation preserves the strictest explicit stop policy",1e-12);
            for (i in 0...21) {
              var reference = timed.trajectory.evaluate(i/10), actual = joined.evaluate(i/10);
              near(actual.positions[0],reference.positions[0],"native ranges preserve position",1e-10);
              near(actual.velocities[0],reference.velocities[0],"native ranges preserve velocity",1e-10);
              near(actual.accelerations[0],reference.accelerations[0],"native ranges preserve acceleration",1e-10);
            }
            joined.dispose();
            var owned = current.commitTiming();
            stream.dispose();
            near(owned.distanceToTime(1.0),2.0,"committed time map survives stream disposal",2e-9);
            check(owned.bindingConstraints.length>0,"lazy bindings survive stream disposal");
            owned.releaseDistanceMap(); owned.trajectory.dispose(); current.dispose();
          case _: throw "Expected a replacement preview";
        }
      case _: throw "Expected a ready single-window preview";
    }
  }
  function motors():Void {
    var path = new JointPathSamples([0.0,0.5,1.0],[[0.0,0.0],[0.5,0.5],[1.0,1.0]],
      [[1.0,1.0],[1.0,1.0],[1.0,1.0]],[[0.0,0.0],[0.0,0.0],[0.0,0.0]]);
    var backend = new ToppraPathTiming();
    var motors = new motionkit.robot.MotorSpaceConstraints([[1.0,1.0],[1.0,-1.0]],[1.0,1.0]);
    var limits = new PathTimingLimits([2.0,2.0],[1.0,1.0]);
    var ordinary = motors.time(backend,path,limits);
    var augmented = new JointPathSamples(path.s,
      [for (q in path.q) q.concat([q[0]+q[1],q[0]-q[1]])],
      [for (q in path.qPrime) q.concat([q[0]+q[1],q[0]-q[1]])],
      [for (q in path.qDoublePrime) q.concat([q[0]+q[1],q[0]-q[1]])]);
    var referenceStream = backend.prepareStream(augmented,
      new PathTimingLimits([2.0,2.0,1.0,1.0],[1.0,1.0,1e12,1e12]),4);
    var reference:motionkit.planner.TimedPath = null;
    switch referenceStream.step() {
      case Ready(candidate): reference = candidate.commitTiming();
      case _: throw "Expected augmented motor reference";
    }
    referenceStream.dispose();
    var stream = motors.prepareStream(backend,path,limits,4);
    switch stream.step() {
      case Ready(candidate):
        var owned = candidate.commitTiming();
        stream.dispose();
        check(owned.trajectory.jointCount() == 2,"stream retains only physical motor-constrained joints");
        for (i in 0...101) {
          var seconds = owned.trajectory.durationSeconds()*i/100;
          var actual = owned.trajectory.evaluate(seconds), expected = reference.trajectory.evaluate(seconds);
          near(actual.positions[0],expected.positions[0],"selected streaming motor timing preserves the augmented solution",1e-8);
          check(Math.abs(actual.velocities[0]+actual.velocities[1]) <= 1.0+1e-8,"shared CoreXY motor rate is bounded");
        }
        near(owned.distanceToTime(1.0),reference.distanceToTime(1.0),"selected result owns its time map",1e-8);
        check(owned.bindingConstraints.length > 0,"selected result preserves lazy binding diagnostics");
        var failed = false;
        try owned.takeSelectedJoints([2]) catch (_:Dynamic) failed = true;
        check(failed,"invalid physical joint selection is rejected atomically");
        near(owned.distanceToTime(1.0),reference.distanceToTime(1.0),"failed selection keeps its time map",1e-8);
        owned.releaseDistanceMap(); owned.trajectory.dispose();
      case _: throw "Expected single-window CoreXY timing result";
    }
    motors.validate(ordinary.trajectory);
    near(ordinary.trajectory.evaluate(ordinary.trajectory.durationSeconds()).positions[1],1.0,"ordinary motor selection keeps its endpoint",1e-9);
    ordinary.releaseDistanceMap(); ordinary.trajectory.dispose();
    reference.releaseDistanceMap(); reference.trajectory.dispose();
  }
  function authoredGeometryBound():Void {
    var limits=new motionkit.trajectory.ValidationLimits(6,Int64.ofInt(1),Int64.ofInt(0));
    var compiler=new motionkit.robot.ProgramCompiler(new MotionKitTestSupport.PlanarSolver(),limits,"work",
      [for (_ in 0...6) 1.0],[for (_ in 0...6) 2.0],[for (_ in 0...6) 10.0],
      motionkit.robot.StartTolerances.uniform(6,0.02,0.02,0.02));
    var geometry=motionkit.path.GeometricPath.lines([new motionkit.path.PathPoint(0,0.001,0),
      new motionkit.path.PathPoint(0.1,0.001,0)]);
    var path=new motionkit.path.PosePath("work",[new motionkit.path.PoseLine(
      new motionkit.path.PoseWaypoint(new motionkit.kinematics.Pose3(0),0.002,0.02),
      new motionkit.path.PoseWaypoint(new motionkit.kinematics.Pose3(0.1),0.002,0.02),
      motionkit.path.OrientationPolicy.Fixed,0.02,0.02)]);
    path.withAuthoredGeometry(geometry,0.002,0.001);
    function compile() return compiler.compile(new motionkit.program.MotionProgram([
      motionkit.program.MotionOp.FollowPath(path,"work",0.02,[])]),[for (_ in 0...6) 0.0],Int64.ofInt(100));
    var bounded=compile();
    check(@:privateAccess geometry.grid==null,"a sufficient authored certificate avoids a whole-path spatial index");
    var duration=bounded.blocks[0].plans[0].durationSeconds;bounded.dispose();
    path=new motionkit.path.PosePath("work",path.primitives).withAuthoredGeometry(geometry,0.002);
    var queried=compile();
    check(@:privateAccess geometry.grid!=null,"uncertified geometry retains the original exact query");
    near(queried.blocks[0].plans[0].durationSeconds,duration,"authored proof does not change the planned clock",0);
    queried.dispose();
  }
  function coupledJointEntry():Void {
    var velocity=[1.0,1.0,1.4,0.5,1.0,1.0],acceleration=[4.0,1.0,1.5,4.0,1.0,1.0],jerk=[20.0,2.0,3.0,20.0,2.0,2.0];
    var limits=new motionkit.trajectory.ValidationLimits(6,Int64.ofInt(1),Int64.ofInt(0));
    for (i in 0...6) {limits.velocity(i,velocity[i]);limits.acceleration(i,acceleration[i]);limits.jerk(i,jerk[i]);}
    var ids=["x","y","sum","difference","held1","held2"];
    var couplings=[new robotkit.model.JointCoupling("x-sum","x","sum",1,0),
      new robotkit.model.JointCoupling("y-sum","y","sum",1,0),
      new robotkit.model.JointCoupling("x-difference","x","difference",1,0),
      new robotkit.model.JointCoupling("y-difference","y","difference",-1,0)];
    var compiler=new motionkit.robot.ProgramCompiler(new MotionKitTestSupport.PlanarSolver(),limits,"work",
      velocity,acceleration,jerk,motionkit.robot.StartTolerances.uniform(6,0.02,0.02,0.02),
      null,0.01,0.5,0.005,0.02,null,null,null,ids,couplings);
    var origin=[for (_ in 0...6) 0.0],goal=[1.0,1.0,2.0,0.0,0.0,0.0];
    var entry=compiler.generateEntry(origin,goal);
    var program=new motionkit.program.MotionProgram([motionkit.program.MotionOp.MoveJ(
      motionkit.program.MoveTarget.JointTarget(goal),new motionkit.MotionOptions(),motionkit.program.Blend.ExactStop)]);
    var compiled=compiler.compile(program,origin,Int64.ofInt(100));
    var planned=compiled.blocks[0].plans[0];
    for (i in 0...101) {
      var a=entry.evaluate(entry.durationSeconds()*i/100),b=planned.evaluate(planned.durationSeconds*i/100);
      near(a.positions[0],a.positions[1],"generated coupled entry retains one progress law",1e-8);
      near(b.positions[0],b.positions[1],"coupled joint moves retain one progress law",1e-8);
      near(a.positions[2],2*a.positions[0],"entry coupling remains exact throughout",1e-8);
      near(b.positions[3],0.0,"opposite motor sum stays still throughout",1e-8);
    }
    entry.dispose();compiled.dispose();
  }
  function execution():Void {
    for (mode in 0...5) {
      var fixture=buildContractArmFixture();
      for (joint in fixture.model.joints) joint.limits.maxAcceleration=4.0;
      var harness=new robotkit.runtime.SimulationHarness(0.001);
      var blueprint=robotkit.runtime.RobotRuntimeCompiler.compile(fixture.model,new robotkit.profile.RobotProfile());
      blueprint.channels.push(new robotkit.execution.ProcessChannelDeclaration("tool.enabled",
        robotkit.execution.ProcessEventValue.Digital(false)));
      var runtime=harness.simulation.addRobot(blueprint);
      var robot=new robotkit.simulation.SimulatedRobot("streamed-arm",runtime,fixture.model.name,
        [for (link in fixture.model.links) link.name],[for (joint in fixture.model.joints) joint.name]);
      var limits=new motionkit.trajectory.ValidationLimits(6,Int64.ofInt(1),Int64.ofInt(0));
      var compiler=new motionkit.robot.ProgramCompiler(new MotionKitTestSupport.PlanarSolver(),limits,"work",
        [for (_ in 0...6) 2.0],[for (_ in 0...6) 4.0],[for (_ in 0...6) 20.0],
        motionkit.robot.StartTolerances.uniform(6,0.02,0.02,0.02),null,0.05);
      compiler.maximumPathPlanSeconds=0.05;
      var path=new motionkit.path.PosePath("work",[new motionkit.path.PoseLine(
        new motionkit.path.PoseWaypoint(new motionkit.kinematics.Pose3(0.0),0.005,0.02),
        new motionkit.path.PoseWaypoint(new motionkit.kinematics.Pose3(0.1),0.005,0.02),
        motionkit.path.OrientationPolicy.Fixed,0.1,0.1)]);
      var program=new motionkit.program.MotionProgram([
        motionkit.program.MotionOp.FollowPath(path,"work",0.1,
          [new motionkit.event.PathEvent(0.05,"tool.enabled",motionkit.event.EventValue.Digital(true))]),
        motionkit.program.MotionOp.SetOutput("tool.enabled",motionkit.event.EventValue.Digital(false)),
        motionkit.program.MotionOp.Dwell(0.01)]);
      var origin=[for (_ in 0...6) 0.0];
      if (mode==2) {
        var compiled=compiler.compile(program,origin,Int64.ofInt(900));
        var owned=compiled.takeBlocks(),first=owned[0].plans[0];
        check(!first.endsAtRest,"a publication cut explicitly requires a successor");
        for (block in owned) for (plan in block.plans) if (plan!=first) plan.dispose();
        var executor=new motionkit.robot.PlanExecutor(robot);
        executor.start(first,false);
        var faulted=false;
        for (tick in 0...300) {
          if (!faulted) try executor.update() catch (_:Dynamic) faulted=true;
          harness.step(Int64.ofInt(tick+1));
        }
        check(faulted && robot.snapshot().faultCode==RobotKitRuntimeConstants.RK_FAULT_TRAJECTORY_UNDERFLOW,
          "a missing moving successor faults through the runtime's bounded stop");
        for (velocity in robot.snapshot().velocities.toArray()) near(velocity,0.0,"starvation reaches rest",1e-6);
        executor.clear();first.dispose();
      } else {
        var motion=new motionkit.robot.ManipulatorMotion(robot,compiler,_ -> null,() -> runtime.pollEvents());
        if (mode==0) {
          var compiled=compiler.compile(program,origin,Int64.ofInt(900));
          check(compiled.blocks[0].plans.length>2,"runtime test has multiple moving publication cuts");
          motion.runCompiled(compiled);compiled.dispose();
        } else motion.run(program);
        var previous=0.0,samples=0,sawBarrier=false;
        var held=false,resumed=false,heldTicks=0,stopped=false;
        for (tick in 0...3000) {
          if (motion.progress().barrier!=null) sawBarrier=true;
          motion.update(0.001);harness.step(Int64.ofInt(tick+1));
          var current=robot.snapshot().setpointPositions.get(0);
          if (previous>0.01 && current<0.09 && current>0.01) {
            if (mode<2) check(current-previous>0.00005,"moving publication cuts do not insert a physical stop");
            samples++;
          }
          if (mode==3) {
            if (!held && current>0.02) {motion.hold();held=true;}
            if (held && !resumed && motion.sessionState()==motionkit.robot.SessionState.Held && ++heldTicks>=20) {
              motion.resume();resumed=true;
            }
          }
          if (mode==4 && !stopped && current>0.02) {motion.stop();stopped=true;}
          previous=current;
          if (motion.completed || motion.failure!=null || (stopped && !motion.running)) break;
          if (mode!=0 && mode!=1) Sys.sleep(0.001);
        }
        if (mode==4) {
          check(stopped && !motion.running && !motion.completed && motion.failure==null,
            'controlled stop releases queued successors without a fault: ${motion.failure}');
          check(robot.snapshot().positions.get(0)<0.1,"controlled stop discards the unpublished suffix");
          for (velocity in robot.snapshot().velocities.toArray()) near(velocity,0.0,"cancelled stream reaches rest",1e-6);
        } else {
          check(motion.completed && motion.failure==null,'continuous execution completes: ${motion.failure}');
          check(samples>100 && sawBarrier,"continuous execution retains the final exact-stop barrier");
          near(robot.snapshot().positions.get(0),0.1,"continuous execution reaches the certified end",1e-4);
          check([for (event in motion.firedEvents()) if (event.channel=="tool.enabled") event].length==2,
            "moving publication cuts fire each event exactly once");
          if (mode==3) check(held && resumed,"a stream with queued successors holds and resumes");
        }
        motion.shutdownPlanning();
      }
      harness.dispose();
    }
  }
  function compilation():Void {
    var solver = new MotionKitTestSupport.WristBranchSolver(false);
    var limits = new motionkit.trajectory.ValidationLimits(6,Int64.ofInt(1),Int64.ofInt(1));
    for (i in 0...6) { limits.velocity(i,2.0); limits.acceleration(i,4.0); limits.jerk(i,20.0); }
    var compiler = new motionkit.robot.ProgramCompiler(solver,limits,"work",[for (_ in 0...6) 2.0],
      [for (_ in 0...6) 4.0],[for (_ in 0...6) 20.0],
      motionkit.robot.StartTolerances.uniform(6,0.02,0.02,0.02),null,0.05);
    var path = new motionkit.path.PosePath("work",[new motionkit.path.PoseLine(
      new motionkit.path.PoseWaypoint(new motionkit.kinematics.Pose3(0.0),0.005,0.02),
      new motionkit.path.PoseWaypoint(new motionkit.kinematics.Pose3(0.1),0.005,0.02),
      motionkit.path.OrientationPolicy.Fixed,0.1,0.1)]);
    var program = new motionkit.program.MotionProgram([motionkit.program.MotionOp.FollowPath(path,"work",0.1,
      [new motionkit.event.PathEvent(0.05,"tool.enabled",motionkit.event.EventValue.Digital(true),0.02)])]);
    var start = [0.0,0.0,0.0,0.0,0.1,0.0];
    var whole = compiler.compile(program,start,Int64.ofInt(700));
    compiler.maximumPathPlanSeconds = 0.1;
    var blocks = new motionkit.robot.ProgramBlocks();
    var work = compiler.begin(program,start,Int64.ofInt(800),0,1.0,blocks);
    var steps=0, firstReady=-1;
    while (work.step()) {
      if (blocks.blocks[0].plans.length > 0 && firstReady < 0) firstReady=steps;
      if (++steps > 1000) throw "Streaming compilation failed to make bounded progress";
    }
    check(firstReady >= 0 && firstReady < steps-1,"compiler publishes before the full path finishes");
    var plans = blocks.blocks[0].plans;
    for (i in 0...plans.length) {
      var times=blocks.blocks[0].pathTimes[i],distances=blocks.blocks[0].pathDistances[i];
      check(times.length>=2 && times.length==distances.length,"every streaming chunk locates progress between source samples");
      near(times[0],0.0,"streamed progress starts at the chunk clock boundary",1e-12);
      near(times[times.length-1],plans[i].durationSeconds,"streamed progress ends at the chunk clock boundary",1e-12);
    }
    check(plans.length > 1,"compiler emits bounded continuous windows");
    var elapsed=0.0, events=0;
    var previous:motionkit.trajectory.TrajectoryState = null;
    for (plan in plans) {
      check(plan.durationSeconds <= 0.1+1e-9,"publication obeys the owning planner's duration budget");
      var initial=plan.evaluate(0.0);
      if (previous != null) for (joint in 0...6) {
        near(initial.positions[joint],previous.positions[joint],"published position seam",1e-10);
        near(initial.velocities[joint],previous.velocities[joint],"published velocity seam",1e-8);
        near(initial.accelerations[joint],previous.accelerations[joint],"published acceleration seam",1e-6);
      }
      for (i in 0...11) {
        var local = plan.durationSeconds*i/10;
        var actual = plan.evaluate(local), reference = whole.blocks[0].plans[0].evaluate(elapsed+local);
        near(actual.positions[0],reference.positions[0],"draining and chunking use the same stream",1e-10);
        near(actual.velocities[0],reference.velocities[0],"chunked velocity matches drained stream",1e-8);
      }
      for (event in plan.events) {
        events++;
        near(elapsed+Int64.toFloat(event.timeNs)*1e-9,
          Int64.toFloat(whole.blocks[0].plans[0].events[0].timeNs)*1e-9,"distance event keeps its global clock and lead",2e-9);
      }
      previous=plan.evaluate(plan.durationSeconds); elapsed+=plan.durationSeconds;
    }
    near(elapsed,whole.blocks[0].plans[0].durationSeconds,"stream drain preserves total duration",2e-9);
    check(events==1,"streaming emits each authored event once");
    work.dispose(); blocks.dispose(); whole.dispose();
  }
  function context():Void {
    var source = Trajectory.fromSegments([{timeFromStartNs:Trajectory.nanoseconds(0),
      durationNs:Trajectory.nanoseconds(2),coefficients:[[0.0,0.0,0.0,1.25,-0.9375,0.1875]]}]);
    var advanced = source.advanceJoint(0,0.03,0.04);
    var whole = advanced.shape([[0.0,0.02]],[[0.5,0.5]]);
    advanced.dispose();
    for (range in 0...3) {
      var begin = range == 0 ? 0.0 : 0.5, end = range == 2 ? 2.0 : 1.5;
      var raw = source.sliceTicks(Trajectory.nanoseconds(begin),Trajectory.nanoseconds(end));
      var pressure = raw.advanceJointContext(0,0.03,0.04,Trajectory.nanoseconds(begin),range == 2);
      var shaped = pressure.shape([[0.0,0.02]],[[0.5,0.5]]);
      pressure.dispose(); raw.dispose();
      var first = range == 0 ? 0 : 40, last = range == 2 ? 1520 : Std.int((end-begin-0.04)*1000);
      for (i in first...(last+1)) {
        var reference = whole.evaluate(begin+i/1000), actual = shaped.evaluate(i/1000);
        near(actual.positions[0],reference.positions[0],"pressure and shaping retain neighboring context",1e-10);
        near(actual.velocities[0],reference.velocities[0],"context velocity matches whole-motion transforms",1e-8);
        near(actual.accelerations[0],reference.accelerations[0],"context acceleration matches whole-motion transforms",1e-5);
      }
      shaped.dispose();
    }
    source.dispose(); whole.dispose();
  }
  function buffer():Void {
    var source = Trajectory.fromSegments([{timeFromStartNs:Trajectory.nanoseconds(0),
      durationNs:Trajectory.nanoseconds(2),coefficients:[[0.0,0.0,0.0,1.25,-0.9375,0.1875]]}]);
    var advanced = source.advanceJoint(0,0.03,0.04);
    var whole = advanced.shape([[0.0,0.02]],[[0.5,0.5]]);
    advanced.dispose();
    var authoredShaping = [new ShaperImpulses([0.0,0.02],[0.5,0.5])];
    var settings:MotionFilterSettings = {pressureAdvance:0.03,pressureAdvanceWindow:0.04,
      shaping:authoredShaping,shapingDelay:0.01};
    var state = new TimedMotionBuffer(settings,null,0,0.1);
    settings.pressureAdvance = 0.0;
    authoredShaping[0].delays[1] = 1.0;
    near(state.lagSeconds,0.01,"filter settings are snapshotted",1e-12);
    var chunks = 0, published = Int64.ofInt(0);
    for (i in 0...10) {
      var part = source.sliceTicks(Trajectory.nanoseconds(i*0.2),Trajectory.nanoseconds((i+1)*0.2));
      state.append(part,i == 9); part.dispose();
      while (true) {
        var candidate = state.preview();
        if (candidate == null) break;
        if (chunks == 0) {
          var replacement = state.preview();
          if (replacement == null) throw "Expected a replacement filter preview";
          var rejected = false;
          try state.retireCertified(candidate) catch (_:Dynamic) rejected = true;
          check(rejected && Int64.compare(state.publishedThrough(),published) == 0,"stale filter retirement is atomic");
          candidate.dispose(); candidate = replacement;
        }
        check(Int64.compare(candidate.startNs,published) == 0,"filter chunks share one global integer clock");
        var duration = candidate.trajectory.durationSeconds();
        check(duration <= 0.1+1e-9,"filter publication obeys its application budget");
        if (!candidate.finalChunk)
          check(duration >= 0.1-2e-9,"moving filter suffix waits for a full publication budget");
        var origin = Int64.toFloat(candidate.startNs)*1e-9;
        for (sample in 0...11) {
          var time = duration*sample/10;
          var reference = whole.evaluate(origin+time), actual = candidate.trajectory.evaluate(time);
          near(actual.positions[0],reference.positions[0],"retained filter buffer preserves whole-motion position",1e-9);
          near(actual.velocities[0],reference.velocities[0],"retained filter buffer preserves whole-motion velocity",1e-7);
          near(actual.accelerations[0],reference.accelerations[0],"retained filter buffer preserves whole-motion acceleration",1e-4);
        }
        published = candidate.endNs; ++chunks;
        state.retireCertified(candidate); candidate.dispose();
      }
    }
    check(state.finished() && chunks>10,"final filter context flushes once across many publications");
    near(Int64.toFloat(published)*1e-9,whole.durationSeconds(),"shaper tail is emitted only at the true end",2e-9);
    state.dispose();
    var a = new TimedMotionBuffer({pressureAdvance:0.03,pressureAdvanceWindow:0.04,
      shaping:null,shapingDelay:0.0},null,0,0.1);
    var b = new TimedMotionBuffer({pressureAdvance:0.03,pressureAdvanceWindow:0.04,
      shaping:null,shapingDelay:0.0},null,0,0.1);
    var first = source.sliceTicks(Trajectory.nanoseconds(0),Trajectory.nanoseconds(0.2));
    a.append(first,false); b.append(first,false); first.dispose();
    var left = a.preview(), right = b.preview();
    if (left == null || right == null) throw "Expected independent filter previews";
    var refused = false;
    try a.retireCertified(right) catch (_:Dynamic) refused = true;
    check(refused,"foreign filter preview cannot retire");
    var later = source.sliceTicks(Trajectory.nanoseconds(0.2),Trajectory.nanoseconds(0.4));
    a.append(later,false); later.dispose();
    refused = false;
    try a.retireCertified(left) catch (_:Dynamic) refused = true;
    check(refused,"appending invalidates old filter previews");
    a.dispose(); b.dispose();
    near(left.trajectory.evaluate(0.0).positions[0],whole.evaluate(0.0).positions[0],"owned filter result survives cancellation",1e-10);
    left.dispose(); right.dispose();
    var events = new TimedMotionBuffer({pressureAdvance:0.0,pressureAdvanceWindow:0.04,
      shaping:null,shapingDelay:0.0},null,null,0.1,0.3);
    var early = source.sliceTicks(Trajectory.nanoseconds(0),Trajectory.nanoseconds(0.2));
    events.append(early,false); early.dispose();
    check(events.preview() == null,"event lead prevents premature prefix publication");
    events.dispose();
    var ticks = new TimedMotionBuffer({pressureAdvance:0.0,pressureAdvanceWindow:0.04,
      shaping:[ShaperImpulses.delay(0.4e-9)],shapingDelay:0.4e-9},null,null,null,1.6e-9);
    var tiny = source.sliceTicks(Int64.ofInt(0),Int64.ofInt(10));
    ticks.append(tiny,false); tiny.dispose();
    var quantized = ticks.preview();
    if (quantized == null) throw "Expected a representable event-safe tick prefix";
    check(Int64.compare(quantized.endNs,Int64.ofInt(8)) <= 0,"event lead and lag use separately quantized clock values");
    ticks.dispose(); quantized.dispose(); source.dispose(); whole.dispose();
  }
  public static function run():Void { var test = new PathTimingStreamTests(); test.exercise(); test.motors(); test.compilation(); test.authoredGeometryBound(); test.coupledJointEntry(); test.execution(); test.context(); test.buffer(); }
  public static function main():Void { run(); Sys.println("Path timing stream tests passed"); }
}
