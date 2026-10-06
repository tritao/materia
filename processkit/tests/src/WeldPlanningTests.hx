import motionkit.kinematics.IkTolerance;
import motionkit.robot.ManipulatorKinematics;
import processkit.WeldCorner;
import processkit.WeldCorner.WristLimits;
import processkit.WeldPathPlanner;
import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.Manipulator;
import robotkit.model.Frame;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import processkit.skill.WeldPlan;
import processkit.skill.WeldPlan.WeldParameters;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/**
 * The corner turn is derived from the angle and the wrist's limits; and a weld is planned for an arm (a UR5-style 6R arm
 * with a torch body on its wrist) so that it is reachable and clear: the torch's roll is chosen to keep its neck off a wall
 * beside the seam, and a seam that cannot be welded without hitting something is refused with the part and the pose.
 */
class WeldPlanningTests {
  static var assertions = 0;
  static final WRIST:WristLimits = {angularSpeed: 3.0, angularAcceleration: 2.0};
  static final PARAMETERS:WeldParameters = {wireSpeed: 8.0, voltage: 24.0, travelSpeed: 0.0115, approach: 0.04, startDwell: 0.15, craterDwell: 0.15,
    burnback: 0.1};

  public static function run():Int {
    assertions = 0;
    testCornerTurn();
    testJoinedPathProblem();
    testProblemTravelAlternatives();
    testRotaryWristSelection();
    testRollAvoidsTheWall();
    testEntryBranchReachesTheWholeWeld();
    testClosedRunChoosesAReachableCorner();
    testBranchJumpIsRefused();
    testCompiledMotionRejectsAnEntry();
    testBlockedJointEntrySkipsDownstreamRolls();
    testEntryUsesTheCurrentConfiguration();
    testReverseTravelPreservesThePushAngle();
    testCollidingWeldIsRefused();
    Sys.println('ProcessKit weld planning tests passed ($assertions assertions)');
    return assertions;
  }

  static function testJoinedPathProblem():Void {
    var requested=seam();
    var problem=new processkit.WeldPathProblem(requested,WRIST,[WeldCorner.AROUND],"weld-task");
    near(problem.seamOffset,PARAMETERS.approach,1e-12,"joined weld retains approach distance before deposition");
    near(problem.seamLength,requested.length(),1e-12,"joined weld preserves deposited seam length");
    near(problem.path.length(),requested.length()+2*PARAMETERS.approach,1e-12,
      "joined geometric route covers approach, seam and retreat");
    check(problem.sections.length==4,"joined weld retains approach, deposition, burnback and retreat stops");
    check(problem.phases[0]==processkit.WeldPathProblem.WeldPathPhase.Approach &&
      problem.phases[1]==processkit.WeldPathProblem.WeldPathPhase.Weld &&
      problem.phases[2]==processkit.WeldPathProblem.WeldPathPhase.Burnback &&
      problem.phases[3]==processkit.WeldPathProblem.WeldPathPhase.Retreat,
      "stopped sections retain their engagement phases");
    near(problem.seamEnd,problem.seamOffset+problem.seamLength,0,"deposition ends at its global process boundary");
    near(problem.burnbackEnd-problem.seamEnd,WeldPathPlanner.LIFT,1e-12,"burnback retains the complete lift distance");
    near(problem.sections[2].primitives[0].speedLimit(),WeldPathPlanner.LIFT/PARAMETERS.burnback,1e-12,
      "burnback geometry uses the execution lift speed");
    check(!problem.contact(problem.approach) && !problem.contact(problem.retreat),
      "air endpoints keep the air clearance margin");
    check(problem.contact(problem.seam.poseAt(0.05)),"seam positions permit the contact margin");
    check(problem.contact(new motionkit.kinematics.Pose3(0.4,0.2,0.155)),
      "burnback lift remains inside the geometric contact zone");
    for(approach in [WeldPathPlanner.LIFT,WeldPathPlanner.LIFT/2]){
      var parameters:WeldParameters={wireSpeed:PARAMETERS.wireSpeed,voltage:PARAMETERS.voltage,
        travelSpeed:PARAMETERS.travelSpeed,approach:approach,startDwell:PARAMETERS.startDwell,
        craterDwell:PARAMETERS.craterDwell,burnback:PARAMETERS.burnback};
      var shortProblem=new processkit.WeldPathProblem(new WeldPlan(requested.segments,parameters),WRIST,
        [WeldCorner.AROUND],"weld-task");
      check(shortProblem.sections.length==(approach==WeldPathPlanner.LIFT?3:4),
        "coincident retreat omits zero-length motion while shorter retreats preserve reverse travel");
      near(shortProblem.path.length(),requested.length()+approach+WeldPathPlanner.LIFT+
        Math.abs(approach-WeldPathPlanner.LIFT),1e-12,"short retreat preserves actual execution travel");
    }
    var restart=problem.recovery(0.037);
    near(restart.startDistance,0.037,0,"recovery retains original seam coordinates");
    near(restart.fullSeamLength,problem.seamLength,0,"recovery retains complete authored extent");
    near(restart.seamLength,problem.seamLength-0.037,1e-12,"recovery selects only remaining geometry");
    for(distance in [0.0,0.01,restart.seamLength]){
      var expected=problem.seam.poseAt(distance+restart.startDistance),actual=restart.seam.poseAt(distance);
      near(motionkit.path.PoseMath.distance(expected,actual),0,1e-12,"restart preserves authored TCP geometry");
      near(motionkit.path.PoseMath.angle(expected,actual),0,1e-7,"restart preserves authored orientation");
    }
    var rejected=false;try problem.recovery(problem.seamLength) catch(_:Dynamic)rejected=true;
    check(rejected,"restart at the completed seam is refused");
    var fixture=arm(),solver=new ManipulatorKinematics(fixture.arm),before=fixture.arm.numericSolveCount();
    var backend=motionkit.robot.BranchIk.of(fixture.arm);
    check(backend.family()=="UR6R","authored weld fixture has a model-derived analytic family");
    var goals=backend.branches(problem.approach,[0.0,-1.5708,1.5708,-1.5708,-1.5708,0.0]);
    check(goals.length>0,"joined wire approach has an analytic start");
    var start=goals[0].q;
    var request=problem.request(start,new IkTolerance(1e-6,1e-6),[for(_ in start)0.2],[for(_ in start)3.0]);
    for(boundary in [problem.seamOffset,problem.seamEnd,problem.burnbackEnd]){
      var retained=false;for(distance in request.distances)if(Math.abs(distance-boundary)<1e-12)retained=true;
      check(retained,"global selection grid retains each engagement stop");
    }
    var world=cell(null).clearance.withGroup(fixture.arm);
    var planner=new motionkit.robot.StructuredJointPathPlanner(fixture.arm,
      new motionkit.robot.CandidateProblem.CandidateSamplingOptions(8,2,4),null,world,8,false,
      null,null,null,0,null,problem.contact);
    var curves=planner.planSections(problem.sections,request);
    check(curves.length==4,"one joined weld problem refines all four process timing sections");
    var offset=0.0;
    for(section in 0...curves.length){
      var curve=curves[section];
      for(i in 0...curve.q.length){
        var target=problem.sections[section].poseAt(curve.s[i]);
        near(motionkit.path.PoseMath.distance(solver.forward(curve.q[i]),target),0,1e-6,
          "joined weld refinement preserves authored TCP positions");
        check(world.violation(curve.q[i],problem.contact(solver.forward(curve.q[i])))==null,
          "joined weld curve respects physical air/contact clearance");
      }
      if(section>0)for(j in 0...start.length)near(curves[section-1].q[curves[section-1].q.length-1][j],curve.q[0][j],
        1e-7,"joined approach/seam/retreat share physical boundary joints");
      offset+=problem.sections[section].length();
    }
    near(request.distances[request.distances.length-1],offset,0,"joined request covers the full global route");
    check(fixture.arm.numericSolveCount()==before,"joined weld builder and selection need no numeric pose IK");
    var measured=[0.0,-1.5708,1.5708,-1.5708,-1.5708,0.0],entryChecks=0;
    var acceptedEntry:Null<motionkit.trajectory.Trajectory> = null;
    var freeRequest=problem.request(measured,new IkTolerance(1e-6,1e-6),
      [for(_ in measured)0.2],[for(_ in measured)3.0]);
    var freeCurves:Array<motionkit.planner.JointPathSamples>;
    try {
      freeCurves=problem.select(fixture.arm,freeRequest,null,world,(from,to)->{
        entryChecks++;
        if(acceptedEntry!=null){acceptedEntry.dispose();acceptedEntry=null;}
        acceptedEntry=motionkit.trajectory.Trajectory.generateStateToState(from,[for(_ in from)0.0],
          [for(_ in from)0.0],to,[for(_ in from)3.0],[for(_ in from)2.0],[for(_ in from)20.0]);
        return motionkit.robot.TrajectoryClearance.violation(world,acceptedEntry,false,0.01,
          q->problem.contact(solver.forward(q)));
      });
      check(entryChecks>0 && acceptedEntry!=null,"free weld start checks its actual generated entry motion");
      var entered=acceptedEntry.evaluate(acceptedEntry.durationSeconds()).positions;
      for(j in 0...measured.length)near(entered[j],freeCurves[0].q[0][j],1e-7,
        "accepted free-entry motion reaches the globally selected approach start");
      check(freeCurves.length==4,"free entry choice covers the complete approach, weld, burnback and retreat");
      check(fixture.arm.numericSolveCount()==before,"free-start weld selection needs no numeric pose IK");
    } catch(error:Dynamic){if(acceptedEntry!=null)acceptedEntry.dispose();throw error;}
    if(acceptedEntry!=null)acceptedEntry.dispose();
    var compiler=new motionkit.robot.ProgramCompiler(solver,
      new motionkit.trajectory.ValidationLimits(6,haxe.Int64.ofInt(1),haxe.Int64.ofInt(0)),"weld-task",
      [for(_ in measured)3.0],[for(_ in measured)2.0],[for(_ in measured)20.0],
      motionkit.robot.StartTolerances.uniform(6,0.01,0.01,0.01),null,0.002,0.2,
      0.001,0.02,new IkTolerance(1e-6,1e-6));
    var recoveryProblem=problem.recovery(0.037);
    var recoveryRequest=recoveryProblem.request(measured,compiler.ikTolerance,compiler.perJointMaxJump,compiler.maxVelocity);
    var recoveryCurves=recoveryProblem.select(fixture.arm,recoveryRequest,null,world);
    var recoveryProgram=new processkit.WeldPathProgram(recoveryProblem,recoveryCurves,
      {arc:"arc",wireSpeed:"wire",voltage:"voltage"},0.039);
    var recoveryCompiled=recoveryProgram.compile(compiler,fixture.arm,measured,haxe.Int64.ofInt(701),world);
    try {
      var dwells=0,maintenance=false,restored=false;
      for(block in recoveryCompiled.blocks){
        switch block.barrier {case Dwell(_):dwells++;default:}
        for(i in 0...block.plans.length){
          var section=recoveryProgram.sectionOps.indexOf(block.opIndices[i]);
          if(section>=0 && recoveryProblem.phases[section]==processkit.WeldPathProblem.WeldPathPhase.Weld){
            var status=recoveryProgram.progress(recoveryCompiled,new motionkit.robot.ManipulatorProgress(
              recoveryCompiled.blocks.indexOf(block),block.opIndices[i],0.001));
            near(status.seamDistance,0.038,1e-12,"recovery progress stays in original seam coordinates");
            for(event in block.plans[i].events)if(event.channel=="wire")switch event.value {
              case Analog(rate):
                if(rate==processkit.tool.WeldArcModel.MIN_WIRE_SPEED)maintenance=true;
                else if(maintenance && haxe.Int64.toFloat(event.timeNs)*1e-9<block.plans[i].durationSeconds-1e-8)restored=true;
              default:
            }
          }
        }
      }
      check(dwells==1,"restrike omits pooling while preserving crater dwell");
      check(maintenance && restored,"recovery maintains arc across overlap then restores deposition");
      var done=recoveryProgram.progress(recoveryCompiled,new motionkit.robot.ManipulatorProgress(0,-1,0),true);
      near(done.seamDistance,problem.seamLength,0,"recovery completion reaches original seam extent");
    }catch(error:Dynamic){recoveryCompiled.dispose();throw error;}
    recoveryCompiled.dispose();
    check(fixture.arm.numericSolveCount()==before,"sliced recovery selection and retained compilation use analytic kinematics");
    var selectedProgram=new processkit.WeldPathProgram(problem,freeCurves,{arc:"arc",wireSpeed:"wire",voltage:"voltage"});
    var compiled=selectedProgram.compile(compiler,fixture.arm,measured,haxe.Int64.ofInt(801),world);
    try {
      var sectionsFound=0,waits=0,dwells=0,quantity=0.0,arcEnds=0;
      for(block in compiled.blocks){
        switch block.barrier {
          case WaitInput(channel,_,_):check(channel==processkit.WeldingPlanRunner.ARC_ESTABLISHED,
            "selected execution waits for established arc");
            var blockIndex=compiled.blocks.indexOf(block);
            var cursor=new motionkit.robot.ManipulatorProgress(blockIndex,-1,0,null,block.barrier);
            var status=selectedProgram.progress(compiled,cursor);
            check(status.phase==processkit.WeldPathProgram.WeldExecutionPhase.WaitingForArc && status.seamDistance==0,
              "selected progress distinguishes ignition before deposition");waits++;
          case Dwell(_):
            var status=selectedProgram.progress(compiled,new motionkit.robot.ManipulatorProgress(
              compiled.blocks.indexOf(block),-1,0,null,block.barrier));
            check(status.phase==(dwells==0?processkit.WeldPathProgram.WeldExecutionPhase.Pooling:
              processkit.WeldPathProgram.WeldExecutionPhase.FillingCrater),"selected progress distinguishes start and crater dwells");
            near(status.seamDistance,dwells==0?0:problem.seamLength,0,"dwell progress retains authored deposition extent");
            dwells++;
          case null:
        }
        for(index in 0...block.plans.length){
          var section=selectedProgram.sectionOps.indexOf(block.opIndices[index]);
          if(section<0)continue;
          sectionsFound++;var timed=block.plans[index],curve=freeCurves[section];
          var halfway=problem.sections[section].length()/2;
          var status=selectedProgram.progress(compiled,new motionkit.robot.ManipulatorProgress(
            compiled.blocks.indexOf(block),block.opIndices[index],halfway));
          near(status.seamDistance,section==0?0:section==1?halfway:problem.seamLength,1e-12,
            "selected progress counts deposition while excluding approach, burnback and retreat travel");
          var begin=timed.evaluate(0),end=timed.evaluate(timed.durationSeconds);
          for(j in 0...6){near(begin.positions[j],curve.q[0][j],1e-7,"timed weld reuses selected section entry");
            near(end.positions[j],curve.q[curve.q.length-1][j],1e-7,"timed weld reuses selected section exit");
            near(begin.velocities[j],0,1e-7,"engagement section starts stopped");
            near(end.velocities[j],0,1e-7,"engagement section ends stopped");}
          var rates=[for(event in timed.events)if(event.channel=="wire")event];
          if(problem.phases[section]==processkit.WeldPathProblem.WeldPathPhase.Weld){
            check(rates.length>1,"deposition rates use the actual compiled clock");
            for(i in 0...rates.length-1)switch rates[i].value {
              case Analog(rate):quantity+=rate*haxe.Int64.toFloat(haxe.Int64.sub(rates[i+1].timeNs,rates[i].timeNs))*1e-9;
              case _:throw "Expected timed wire rate";
            }
          }
          if(problem.phases[section]==processkit.WeldPathProblem.WeldPathPhase.Burnback){
            for(event in timed.events)if(event.channel=="arc")switch event.value {
              case Digital(false):near(haxe.Int64.toFloat(event.timeNs)*1e-9,timed.durationSeconds,1e-8,
                "arc turns off at the completed burnback lift");arcEnds++;
              case _:throw "Expected burnback arc off";
            }
          }
        }
      }
      check(sectionsFound==4 && waits==1 && dwells==2 && arcEnds==1,
        "one selected compilation retains all process phases and engagement barriers");
      near(quantity,PARAMETERS.wireSpeed/PARAMETERS.travelSpeed*problem.seamLength,1e-6,
        "selected execution deposits the authored quantity over the final clock");
      check(fixture.arm.numericSolveCount()==before,"selected weld compilation performs no numeric pose IK");
      var stopped=selectedProgram.progress(compiled,new motionkit.robot.ManipulatorProgress(0,-1,0));
      check(stopped.phase==processkit.WeldPathProgram.WeldExecutionPhase.Approaching && stopped.seamDistance==0,
        "an inactive cursor does not falsely report a completed weld");
      var finished=selectedProgram.progress(compiled,new motionkit.robot.ManipulatorProgress(0,-1,0),true);
      check(finished.phase==processkit.WeldPathProgram.WeldExecutionPhase.Complete && finished.seamDistance==problem.seamLength,
        "only confirmed execution completion closes the deposited seam");
    }catch(error:Dynamic){compiled.dispose();throw error;}
    compiled.dispose();
    var harness=new robotkit.runtime.SimulationHarness(0.01);
    var blueprint=robotkit.runtime.RobotRuntimeCompiler.compile(fixture.arm.robot,new robotkit.profile.RobotProfile());
    var channels={arc:"selected.arc",wireSpeed:"selected.wire",voltage:"selected.voltage"};
    blueprint.channels.push(new robotkit.execution.ProcessChannelDeclaration(channels.arc,robotkit.execution.ProcessEventValue.Digital(false)));
    blueprint.channels.push(new robotkit.execution.ProcessChannelDeclaration(channels.wireSpeed,robotkit.execution.ProcessEventValue.Analog(0)));
    blueprint.channels.push(new robotkit.execution.ProcessChannelDeclaration(channels.voltage,robotkit.execution.ProcessEventValue.Analog(0)));
    var runtime=harness.simulation.addRobot(blueprint);
    var robot=new robotkit.simulation.SimulatedRobot("selected-weld",runtime,fixture.arm.robot.name,
      [for(link in fixture.arm.robot.links)link.name],[for(joint in fixture.arm.robot.joints)joint.name]);
    robot.submit(robotkit.core.RobotCommand.JointTargets([for(j in 0...6)robotkit.core.JointTarget.position(j,measured[j])],null));
    var tick=0;for(_ in 0...400)harness.step(haxe.Int64.ofInt(++tick));
    var arc=false;
    var runner=processkit.WeldingPlanRunner.create(robot,fixture.arm,()->{
      var batch=runtime.pollEvents();
      for(event in batch.events)if(event.channel==channels.arc)switch event.value {
        case robotkit.execution.ProcessEventValue.Digital(value):arc=value;
        default:
      }
      return batch;
    },channels,motionkit.robot.PlanningLimits.ofGroup(fixture.arm,new robotkit.model.SteadyLoads(),2.0),1,world);
    var executionProblem=new processkit.WeldPathProblem(requested,WRIST,[WeldCorner.AROUND],processkit.WeldingPlanRunner.FRAME);
    try {
      runner.runSelected(executionProblem,freeCurves);
      for(_ in 0...4000){
        runner.update(0.01,{arc:arc,currentA:arc?200.0:0.0,voltageV:24.0,touch:false,
          fault:processkit.tool.WeldFault.None,powerW:arc?4800.0:0.0});
        harness.step(haxe.Int64.ofInt(++tick));
        if(runner.completed() || runner.failure()!=null)break;
      }
      check(runner.completed() && runner.failure()==null,'selected runner executes its retained compilation: ${runner.failure()}');
      check(runner.motion.planningMetrics().seconds==0 && runner.motion.planningMetrics().numericIkSolves==0,
        "selected runner launch performs no additional compilation");
      check(runner.restarts()==0 && !arc,"selected runner completes burnback without a recovery or active arc");
      check(cast(runner.current,processkit.ProcessRun).state==processkit.ProcessRunState.Completion,"selected runner completes through the process lifecycle");
      runner.runSelected(executionProblem,freeCurves);
      var interrupted=false;
      for(_ in 0...5000){
        var cursor=runner.motion.progress();
        var inject=!interrupted && arc && cursor.pathDistance>0.04;
        if(inject)interrupted=true;
        runner.update(0.01,{arc:arc && !inject,currentA:arc?200.0:0.0,voltageV:24.0,touch:false,
          fault:inject?processkit.tool.WeldFault.ArcLost:processkit.tool.WeldFault.None,powerW:arc?4800.0:0.0});
        harness.step(haxe.Int64.ofInt(++tick));
        if(runner.completed() || runner.failure()!=null)break;
      }
      check(interrupted && runner.restarts()==1,"selected runner handles one injected arc loss");
      near(cast(runner.current,processkit.ProcessRun).interruptedAt-cast(runner.current,processkit.ProcessRun).lastProgramStart,
        processkit.WeldingPlanRunner.BACKOFF,1e-12,"selected recovery activates the original-coordinate backoff boundary");
      check(runner.completed() && runner.failure()==null,'selected recovery completes: ${runner.failure()}');
      check(runner.motion.planningMetrics().seconds==0 && runner.planningIkSolves==0,
        "selected recovery submits retained compilation with no worker or numeric solve");
      check(!arc && cast(runner.current,processkit.ProcessRun).state==processkit.ProcessRunState.Completion,
        "selected recovery completes through process lifecycle with arc off");

    }catch(error:Dynamic){runner.abort();harness.dispose();throw error;}
    harness.dispose();

  }

  static function testProblemTravelAlternatives():Void {
    var lean=Quat.fromAxisAngle(new Vec3(1,0,0),0.75*Math.PI);
    var points=[new Vec3(0.35,0.2,0.15),new Vec3(0.45,0.2,0.15),new Vec3(0.45,0.3,0.15),new Vec3(0.35,0.3,0.15)];
    var segments:Array<processkit.skill.WeldPlan.WeldSegment> = [];
    for(i in 0...3){var rotation=Quat.fromAxisAngle(new Vec3(0,0,1),i*Math.PI/2).multiply(lean);
      segments.push(new processkit.skill.WeldPlan.WeldSegment(new Transform3(points[i],rotation),
        new Transform3(points[i+1],rotation),'side$i',new Vec3(0,0,1)));}
    var authored=new WeldPlan(segments,PARAMETERS),styles=[WeldCorner.AROUND,WeldCorner.ARC,WeldCorner.ROTATION];
    var original=new processkit.WeldPathProblem(authored,WRIST,styles,"weld-task");
    styles[1]=WeldCorner.AROUND; // Caller mutation must not change the saved alternative.
    var reversed=original.reversed();
    check([for(s in reversed.plan.segments)s.name].join(",")=="side2,side1,side0",
      "reversed problem visits each seam once in opposite order");
    near(reversed.plan.length(),authored.length(),1e-12,"reversed problem preserves deposited length");
    check(authored.segments[0].name=="side0","travel alternative leaves authored seam order intact");
    var expected=new processkit.WeldPathProblem(authored.reversed(),WRIST,
      [WeldCorner.AROUND,WeldCorner.ROTATION,WeldCorner.ARC],"weld-task");
    for(i in 0...21){var distance=expected.seam.length()*i/20;
      near(motionkit.path.PoseMath.distance(reversed.seam.poseAt(distance),expected.seam.poseAt(distance)),0,1e-9,
        "reversed problem preserves physical corner positions");
      near(motionkit.path.PoseMath.angle(reversed.seam.poseAt(distance),expected.seam.poseAt(distance)),0,1e-7,
        "reversed problem maps each saved corner style to its physical join");}
    var restored=authored.reversed().reversed();
    for(i in 0...segments.length){
      near(restored.segments[i].start.rotation.angularDistance(segments[i].start.rotation),0,1e-7,
        "reversing twice restores authored orientation");
      check(restored.segments[i].open==segments[i].open,"reversed travel preserves the material open-side frame");}
    var alternative=original.withCornerStyles([WeldCorner.AROUND,WeldCorner.AROUND,WeldCorner.AROUND]);
    var changed=false;
    for(i in 0...101){var distance=original.seam.length()*i/100;
      if(motionkit.path.PoseMath.angle(original.seam.poseAt(distance),alternative.seam.poseAt(distance))>1e-4)changed=true;}
    check(changed,"corner alternatives are distinct geometric problems before IK or timing");
    near(alternative.seam.length(),original.seam.length(),1e-12,"corner alternatives preserve deposited length");
  }

  static function testRotaryWristSelection():Void {
    var model = new RobotModel("XYZ-CA");
    var base = model.addLink(new Link("base")), parent = base;
    for (id in ["x", "y", "z", "c", "a"]) {
      var child = model.addLink(new Link(id + "-link"));
      var rotary = id == "c" || id == "a";
      var joint = model.addJoint(new Joint(id, rotary ? JointType.Revolute : JointType.Prismatic, parent, child));
      joint.axis = id == "x" || id == "a" ? [1.0, 0, 0] : id == "y" ? [0.0, 1, 0] : [0.0, 0, 1];
      joint.limits.lower = -3.0; joint.limits.upper = 3.0; joint.limits.velocity = rotary ? 2.0 : 0.1;
      parent = child;
    }
    var flange = model.addFrame(new Frame("flange", parent));
    var group = new Manipulator(model, base.id, flange.id);
    var ids = [for (id in group.jointIds()) Std.string(id)];
    var selected = [for (index in processkit.WeldingPlanRunner.wristJointIndices(group)) ids[index]];
    check(selected.join(",") == "a,c", "a CA wrist uses both rotary joints and no linear velocity as an angular cap");
    var z = [for (link in model.links) if (link.id == "z-link") link][0];
    var fixedFrame = model.addFrame(new Frame("fixed-torch", z));
    var xyz = new Manipulator(model, base.id, fixedFrame.id);
    check(processkit.WeldingPlanRunner.wristJointIndices(xyz).length == 0,
      "a fixed XYZ torch has no rotary wrist limits");
    var fixture = arm();
    ids = [for (id in fixture.arm.jointIds()) Std.string(id)];
    selected = [for (index in processkit.WeldingPlanRunner.wristJointIndices(fixture.arm)) ids[index]];
    check(selected.join(",") == "j5,j4,j3", "a six-axis arm retains its nearest three rotary wrist joints");
  }

  static function testCornerTurn():Void {
    var travel = 0.0115;
    check(WeldCorner.turnLength(0.0, travel, WRIST) == 0.0, "a straight join has no turn");
    check(WeldCorner.turnLength(1e-9, travel, WRIST) == 0.0, "an orientation change of nothing has no turn");
    var shallow = WeldCorner.turnLength(0.3, travel, WRIST);
    var quarter = WeldCorner.turnLength(Math.PI / 2, travel, WRIST);
    var half = WeldCorner.turnLength(Math.PI, travel, WRIST);
    check(shallow >= WeldCorner.MIN_TURN && shallow < quarter, 'a shallower corner turns over less than a quarter turn ($shallow m, $quarter m)');
    check(quarter < half || half == WeldCorner.MAX_TURN, 'a sharper corner turns over more, up to the limit ($quarter m, $half m)');
    check(half <= WeldCorner.MAX_TURN && shallow >= WeldCorner.MIN_TURN, "turns stay within their bounds");
    // The time of a quarter turn at 0.5 of a wrist of 3 rad/s and 2 rad/s^2: the speed limit (1.5) is not reached, so 2 sqrt(a / 1).
    near(WeldCorner.turnTime(Math.PI / 2, WRIST), 2.0 * Math.sqrt(Math.PI / 2), 1e-9, "a quarter turn takes 2 sqrt(a / alpha')");
    near(quarter, travel * WeldCorner.turnTime(Math.PI / 2, WRIST) / 2.0, 1e-9, "the tip travels at its speed while the torch turns, half on each side");
    // A stronger wrist turns faster, so the stretch is shorter; a faster weld needs more of it.
    var strong = WeldCorner.turnLength(Math.PI / 2, travel, {angularSpeed: 6.0, angularAcceleration: 8.0});
    check(strong < quarter, "a stronger wrist needs less path to turn");
    check(WeldCorner.turnLength(Math.PI / 2, travel * 1.5, WRIST) > quarter, "a faster weld needs more path to turn");
    // A turn at speed w' over a long way reaches the speed limit: a - w'^2 / alpha' > 0.
    var long = WeldCorner.turnTime(2 * Math.PI, WRIST);
    near(long, 2 * Math.PI / 1.5 + 1.5 / 1.0, 1e-9, "a long turn reaches the speed limit");
    // The turn keeps the wire on the shortest arc between its two directions. Two sides of a post: wires 75 degrees apart, the
    // second rolled a quarter turn. Half way, the wire is the bisector of the two, steeper than the rotations' own interpolation.
    var first = Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), 0.0).multiply(Quat.fromAxisAngle(new Vec3(1.0, 0.0, 0.0), Math.PI * 0.75));
    var second = Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), Math.PI / 2).multiply(first);
    var w1 = first.rotate(new Vec3(0.0, 0.0, 1.0)), w2 = second.rotate(new Vec3(0.0, 0.0, 1.0));
    var wire = WeldCorner.orientationAt(first, second, 0.5).rotate(new Vec3(0.0, 0.0, 1.0));
    var bisector = w1.add(w2).normalized();
    near(wire.dot(bisector), 1.0, 1e-9, "half way through a turn the wire is the bisector of the two");
    // Round a corner whose edge is vertical (travel along +Y, then along -X), the wire keeps its elevation: the nozzle keeps its
    // distance from the edge. The great arc between the wires would steepen it half way.
    var yaw = Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), Math.PI / 2);
    var leaning = Quat.fromAxisAngle(new Vec3(1.0, 0.0, 0.0), Math.PI * 0.75);
    var away = yaw.multiply(leaning);
    var around = WeldCorner.orientationAt(leaning, away, 0.5, new Vec3(0.0, 1.0, 0.0), new Vec3(-1.0, 0.0, 0.0)).rotate(new Vec3(0.0, 0.0, 1.0));
    near(around.z, leaning.rotate(new Vec3(0.0, 0.0, 1.0)).z, 1e-9, "round a vertical corner the wire keeps its elevation");
    var shortcut = WeldCorner.orientationAt(leaning, away, 0.5).rotate(new Vec3(0.0, 0.0, 1.0));
    check(shortcut.z < around.z - 0.05, "where the shortest arc between the wires would have steepened it");
    near(WeldCorner.orientationAt(first, second, 0.0).angularDistance(first), 0.0, 1e-9, "a turn starts at its first orientation");
    near(WeldCorner.orientationAt(first, second, 1.0).angularDistance(second), 0.0, 1e-9, "and ends at its second");
    // The roll turns evenly: a quarter turn in all, an eighth at half way, while the wire stays put when the wires are equal.
    var rolled = WeldCorner.orientationAt(first, Quat.fromAxisAngle(w1, Math.PI / 2).multiply(first), 0.5);
    near(rolled.rotate(new Vec3(0.0, 0.0, 1.0)).dot(w1), 1.0, 1e-9, "a turn of the roll alone leaves the wire");
    near(first.angularDistance(rolled), Math.PI / 4, 1e-9, "and turns it evenly");
    var continued = WeldCorner.continuationRoll(first, second);
    var held = second.multiply(Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), continued));
    near(held.rotate(new Vec3(0.0, 0.0, 1.0)).dot(w2), 1.0, 1e-9, "continuation roll preserves the next seam's wire direction");
    near(first.angularDistance(held), Math.atan2(w1.cross(w2).norm(), w1.dot(w2)), 1e-9, "continuation uses only the wire's swing, with no added twist");
    // A short segment gives no more than its share to a corner.
    near(WeldCorner.given(0.02, 0.020), WeldCorner.SHARE * 0.020, 1e-12, "a short segment gives its share");
    near(WeldCorner.given(0.02, 0.2), 0.02, 1e-12, "a long segment gives the whole turn");
    check(WeldCorner.given(0.0, 0.2) == 0.0, "no turn takes nothing from a segment");
  }

  /** A UR5-style arm with joints, its flange the tool frame (+Z the wire out of the torch), and its links' ids. */
  static function arm():{arm:Manipulator, links:Array<String>} {
    var d1 = 0.089159, shoulderOffset = 0.13585, elbowOffset = -0.1197, a2 = 0.425, a3 = 0.39225, d4 = 0.10915, d5 = 0.09465, d6 = 0.0823;
    var model = new RobotModel("planning-arm");
    var names = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"];
    var links = [for (name in names) model.addLink(new Link(name))];
    var offsets = [new Vec3(0.0, 0.0, d1), new Vec3(0.0, shoulderOffset, 0.0), new Vec3(0.0, elbowOffset, a2), new Vec3(0.0, 0.0, a3),
      new Vec3(0.0, d4, 0.0), new Vec3(0.0, 0.0, d5)];
    var axes = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]];
    for (i in 0...6) {
      var joint = model.addJoint(new Joint('j$i', JointType.Revolute, links[i], links[i + 1]));
      joint.parentFramePosition = offsets[i].toArray();
      joint.axis = axes[i];
      joint.limits.lower = -2.0 * Math.PI;
      joint.limits.upper = 2.0 * Math.PI;
      joint.limits.velocity = 3.0;
    }
    var flange = model.addFrame(new Frame("flange", links[6]));
    flange.position = [0.0, d6, 0.0];
    return {arm: new Manipulator(model, links[0].id, flange.id), links: names};
  }

  static function box(x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float):Array<Float> {
    var result:Array<Float> = [];
    for (x in [x0, x1]) for (y in [y0, y1]) for (z in [z0, z1]) {
      result.push(x);
      result.push(y);
      result.push(z);
    }
    return result;
  }

  /**
   * The cell: the torch is a bar behind the wire tip (along -Z of the tool frame) with a neck sticking out to the tool's
   * +X; a table below the seam; and, when `wall` is given, a wall as a fixed body beside the seam's end.
   */
  static function cell(wallX:Null<Float>):{planner:WeldPathPlanner, clearance:ArmClearance, start:Array<Float>} {
    var made = arm();
    var d6 = 0.0823;
    var bodies = [
      {name: "torch", link: "wrist_3_link", vertices: box(-0.012, 0.012, d6 - 0.012, d6 + 0.012, -0.14, -0.03), tool: true},
      {name: "neck", link: "wrist_3_link", vertices: box(0.0, 0.10, d6 - 0.012, d6 + 0.012, -0.14, -0.10), tool: true},
      {name: "table", link: "base_link", vertices: box(-0.8, 0.8, -0.8, 0.8, -0.1, 0.0), tool: false}
    ];
    if (wallX != null) bodies.push({name: "wall", link: "base_link", vertices: box(wallX, wallX + 0.05, 0.05, 0.35, 0.0, 0.5), tool: false});
    var start = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0];
    var clearance = new ArmClearance(made.arm, bodies, start);
    var solver = new ManipulatorKinematics(made.arm, 1e-8);
    solver.preferTargetOrientation = true;
    var planner = new WeldPathPlanner(solver, new IkTolerance(2e-4, 1e-3, 300, 0.03), [for (_ in 0...6) 3.0], WRIST, clearance);
    return {planner: planner, clearance: clearance, start: start};
  }

  /** A straight seam along +X at height 0.15, the wire pointing down. */
  static function seam():WeldPlan {
    var down = Quat.fromAxisAngle(new Vec3(1.0, 0.0, 0.0), Math.PI);
    return WeldPlan.straight(new Transform3(new Vec3(0.35, 0.2, 0.15), down), new Transform3(new Vec3(0.45, 0.2, 0.15), down), PARAMETERS);
  }

  static function testRollAvoidsTheWall():Void {
    var free = cell(null);
    var open = free.planner.plan(seam(), free.start);
    near(open.rolls[0], 0.0, 1e-9, "with nothing beside it the torch keeps the seam frame's own roll");
    check(open.entry.name == "along the wire", 'it comes in along the wire (${open.entry.name})');
    check(open.checked > 20, 'every pose along the way was checked (${open.checked})');
    // A wall 30 mm past the seam's end, on the side the neck points to at roll 0: the neck would hit it, the other side is clear.
    var walled = cell(0.48);
    var turned = walled.planner.plan(seam(), walled.start);
    check(Math.abs(turned.rolls[0]) > 0.5, 'beside a wall the torch is rolled so that its neck points away (${turned.rolls[0]} rad)');
    Sys.println('weld planning: open seam roll ${open.rolls[0]} via "${open.entry.name}", ${open.checked} poses; beside a wall roll ${turned.rolls[0]}, ${turned.checked} poses');
    check(turned.checked > open.checked / 2, 'and the rolls tried before it were checked too (${turned.checked} poses)');
  }

  static function testEntryBranchReachesTheWholeWeld():Void {
    var planner = new WeldPathPlanner(new EntryBranchFixture(), new IkTolerance(2e-4, 1e-3, 300, 0.03), [3.0], WRIST);
    var planned = planner.plan(seam(), [0.0]);
    check(planned.entry.joints[0] == 1.0, "entries that only reach the start are replaced by a later IK branch that reaches the whole seam");
    near(planned.rolls[0], 0.0, 1e-9, "choosing the other entry branch keeps the requested weld orientation");
  }

  static function testCollidingWeldIsRefused():Void {
    // A wall right at the seam's end: no roll takes the neck and the torch clear of it, nor any way in or out.
    var tight = cell(0.452);
    var message:Null<String> = null;
    try tight.planner.plan(seam(), tight.start) catch (error:Dynamic) message = Std.string(error);
    check(message != null, "a weld that cannot be done clear of the wall is refused");
    var text = message == null ? "" : message;
    check(text.indexOf("wall") >= 0 && (text.indexOf("torch") >= 0 || text.indexOf("neck") >= 0), 'the reason names the parts: $text');
    check(text.indexOf("mm") >= 0 && text.indexOf("tip ") >= 0, 'and the pose of the tip: $text');
    Sys.println('weld planning: refused: $text');
  }

  static function testClosedRunChoosesAReachableCorner():Void {
    var rotation = seam().start().rotation;
    var points = [new Vec3(0.35, 0.2, 0.15), new Vec3(0.45, 0.2, 0.15), new Vec3(0.45, 0.3, 0.15), new Vec3(0.35, 0.3, 0.15)];
    var segments = [for (i in 0...4) new processkit.skill.WeldPlan.WeldSegment(new Transform3(points[i], rotation),
      new Transform3(points[(i + 1) % 4], rotation), 'side$i')];
    var requested = new WeldPlan(segments, PARAMETERS);
    var planner = new WeldPathPlanner(new EntryCornerFixture(), new IkTolerance(2e-4, 1e-3, 300, 0.03), [3.0], WRIST);
    var planned = planner.plan(requested, [0.0]);
    check(planned.plan.segments[0].name == "side1", "a closed run may start at the next reachable corner");
    check([for (segment in planned.plan.segments) segment.name].join(",") == "side1,side2,side3,side0", "every seam keeps its direction and is welded exactly once");
    near(planned.plan.length(), requested.length(), 1e-9, "changing the entry corner preserves the deposited length");
    check(requested.segments[0].name == "side0", "planning leaves the CAD mission unchanged");
  }

  static function testBranchJumpIsRefused():Void {
    var planner = new WeldPathPlanner(new BranchJumpFixture(), new IkTolerance(2e-4, 1e-3, 300, 0.03), [3.0], WRIST);
    var message = "";
    try planner.plan(seam(), [0.0]) catch (error:Dynamic) message = Std.string(error);
    check(message.indexOf("IK branch discontinuity") >= 0, "reachable individual poses with a joint branch jump are refused before welding");
  }

  static function testCompiledMotionRejectsAnEntry():Void {
    var probe = {calls: 0};
    var planner = new WeldPathPlanner(new ContinuousBranchFixture(), new IkTolerance(2e-4, 1e-3, 300, 0.03), [3.0], WRIST, null, null,
      function(planned, start) {
        probe.calls++;
        if (planned.entry.joints[0] < 1.0) throw "The compiler rejects this entry branch";
        return new motionkit.robot.CompiledProgram([], []);
      });
    var planned = planner.plan(seam(), [0.0]);
    check(probe.calls == 3, "each complete candidate is compiled before it can be accepted");
    check(planned.entry.joints[0] == 1.0, "a compiler rejection tries a different entry before starting the weld");
  }

  static function testEntryUsesTheCurrentConfiguration():Void {
    var planner = new WeldPathPlanner(new CurrentEntryFixture(), new IkTolerance(2e-4, 1e-3, 300, 0.03), [3.0], WRIST);
    var planned = planner.plan(seam(), [0.3]);
    near(planned.entry.joints[0], 0.3, 1e-9, "the current arm configuration can reach an entry missed by broad IK sampling");
  }

  static function testBlockedJointEntrySkipsDownstreamRolls():Void {
    for (op in [0, 1]) testBlockedEntryOperation(op);
  }

  static function testBlockedEntryOperation(op:Int):Void {
    var made = arm();
    var clearance = new EntryTrajectoryClearance(made.arm);
    var probe = {calls: 0};
    var straight = seam();
    var middle = new Transform3(new Vec3(0.4, 0.2, 0.15), straight.start().rotation);
    var requested = new WeldPlan([
      new processkit.skill.WeldPlan.WeldSegment(straight.start(), middle, "first"),
      new processkit.skill.WeldPlan.WeldSegment(middle, straight.stop(), "second")
    ], PARAMETERS);
    var planner = new WeldPathPlanner(new EntryTrajectorySolver(), new IkTolerance(2e-4, 1e-3, 300, 0.03),
      [3.0], WRIST, clearance, null, function(planned, start) {
        probe.calls++;
        // The sampled entry sweep clears, but the compiler's actual entry
        // passes through the obstacle on the first two entry branches.
        var q = planned.entry.joints[0] < 1.0 ? 3.0 : 1.0;
        var trajectory = motionkit.trajectory.Trajectory.generateStateToState([q], [0.0], [0.0], [q + 0.001], [3.0], [2.0], [100.0]);
        var limits = new motionkit.trajectory.ValidationLimits(1, haxe.Int64.ofInt(1), haxe.Int64.ofInt(1));
        var plan = motionkit.trajectory.ExecutionPlan.create(trajectory, limits, haxe.Int64.ofInt(1),
          [q], [0.0], [0.0], [0.01], [0.01], [0.01]);
        trajectory.dispose();
        return new motionkit.robot.CompiledProgram([new motionkit.robot.ProgramBlock([plan], [op], null)], []);
      });
    var planned = planner.plan(requested, [0.0]);
    check(probe.calls == 3, 'blocked entry operation $op is checked once, regardless of downstream seam rolls');
    check(planned.entry.joints[0] == 1.0, "a blocked compiled entry advances to a clear entry branch");
    check(planned.plan.segments.length == 2, "entry pruning preserves every seam in the accepted run");
  }

  static function testReverseTravelPreservesThePushAngle():Void {
    var down = seam().start().rotation;
    var tilted = Quat.fromAxisAngle(new Vec3(0.0, 1.0, 0.0), -0.17453292519943295).multiply(down);
    var requested = WeldPlan.straight(new Transform3(seam().start().translation, tilted), new Transform3(seam().stop().translation, tilted), PARAMETERS);
    var planner = new WeldPathPlanner(new EntryCornerFixture(), new IkTolerance(2e-4, 1e-3, 300, 0.03), [3.0], WRIST);
    var planned = planner.plan(requested, [0.0]);
    near(planned.plan.start().translation.x, requested.stop().translation.x, 1e-9, "a seam may enter from its other reachable endpoint");
    near(planned.plan.stop().translation.x, requested.start().translation.x, 1e-9, "reversed travel deposits the whole seam");
    var before = tilted.rotate(new Vec3(0.0, 0.0, 1.0));
    var after = planned.plan.start().rotation.rotate(new Vec3(0.0, 0.0, 1.0));
    near(after.x, -before.x, 1e-9, "reversing travel reverses the push component of the wire");
    near(after.z, before.z, 1e-9, "and preserves the wire's work angle in the face cross-section");
  }

  static function near(actual:Float, expected:Float, tolerance:Float, message:String):Void {
    assertions++;
    if (!(Math.abs(actual - expected) <= tolerance)) throw 'Assertion failed: $message (expected $expected, got $actual)';
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'Assertion failed: $message';
  }
}

/** Three reachable entry branches; only the third can reach the last half of the seam. */
private class EntryBranchFixture implements motionkit.kinematics.KinematicsSolver {
  public function new() {}
  public function jointCount():Int return 1;
  public function fork():motionkit.kinematics.KinematicsSolver return this;
  public function forward(q:Array<Float>):motionkit.kinematics.Pose3 throw "The fixture only solves poses";
  public function sampleCandidates(target:motionkit.kinematics.Pose3, maxCount:Int,
      tolerance:motionkit.kinematics.IkTolerance, ?freedom:motionkit.path.OrientationPolicy):Array<Array<Float>> return [[0.0], [0.5], [1.0]];
  public function solvePose(target:motionkit.kinematics.Pose3, seed:Array<Float>,
      tolerance:motionkit.kinematics.IkTolerance, ?freedom:motionkit.path.OrientationPolicy):Null<Array<Float>>
    return seed[0] < 1.0 && target.x >= 0.4 ? null : seed.copy();
  public function solveDifferential(q:Array<Float>, twist:motionkit.kinematics.Twist6,
      ?redundancyRate:Array<Float>, ?freedom:motionkit.path.OrientationPolicy):Null<Array<Float>> throw "The fixture only solves poses";
  public function solvePath(request:motionkit.kinematics.PathRequest):Array<Null<Array<Float>>> throw "The fixture only solves poses";
}

/** The weld poses are reachable, but an approach is available only on the far side of the perimeter. */
private class EntryCornerFixture extends EntryBranchFixture {
  public function new() super();
  override public function sampleCandidates(target:motionkit.kinematics.Pose3, maxCount:Int,
      tolerance:motionkit.kinematics.IkTolerance, ?freedom:motionkit.path.OrientationPolicy):Array<Array<Float>> return target.x >= 0.4 ? [[1.0]] : [];
}

/** Each pose has an IK answer, but reaching the second half requires a discontinuous joint change. */
private class BranchJumpFixture extends EntryBranchFixture {
  public function new() super();
  override public function solvePose(target:motionkit.kinematics.Pose3, seed:Array<Float>,
      tolerance:motionkit.kinematics.IkTolerance, ?freedom:motionkit.path.OrientationPolicy):Null<Array<Float>> return [seed[0] + (target.x >= 0.4 ? 1.0 : 0.0)];
}

private class ContinuousBranchFixture extends EntryBranchFixture {
  public function new() super();
  override public function solvePose(target:motionkit.kinematics.Pose3, seed:Array<Float>,
      tolerance:motionkit.kinematics.IkTolerance, ?freedom:motionkit.path.OrientationPolicy):Null<Array<Float>> return seed.copy();
}

private class CurrentEntryFixture extends ContinuousBranchFixture {
  public function new() super();
  override public function sampleCandidates(target:motionkit.kinematics.Pose3, maxCount:Int,
      tolerance:motionkit.kinematics.IkTolerance, ?freedom:motionkit.path.OrientationPolicy):Array<Array<Float>> return [];
}

private class EntryTrajectorySolver extends ContinuousBranchFixture {
  public function new() super();
  override public function forward(q:Array<Float>):motionkit.kinematics.Pose3
    return new motionkit.kinematics.Pose3(0.35, 0.2, 0.19, 0, 0, 0, 1);
}

private class EntryTrajectoryClearance extends ArmClearance {
  public function new(group:robotkit.manipulation.KinematicGroup)
    super(group, [], [for (_ in group.jointIds()) 0.0]);
  override public function violation(q:Array<Float>, contact:Bool = false, ?wanted:Float):Null<robotkit.manipulation.ArmClearance.ClearanceViolation>
    return q[0] > 2.0 ? {a: "torch", b: "upright", distance: 0.002, required: 0.003} : null;
}
