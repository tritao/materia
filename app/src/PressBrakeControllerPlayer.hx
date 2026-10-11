package app;

import haxeon.wire.JsonWire;
import materia.project.SceneArtifact.SceneArtifactPressBrake;
import materia.project.SceneArtifact.SceneArtifactPressBrakeStep;
import nativekit.sim.SimSession;
import processkit.pressbrake.PressBrakeInterface;
import processkit.pressbrake.PressBrakeInput;
import processkit.pressbrake.PressBrakeOutputs;
import processkit.pressbrake.PressBrakePhase;
import processkit.pressbrake.PressBrakeCommand;
import processkit.pressbrake.PressBrakeChannels;
import processkit.pressbrake.PressBrakeRobotRequest;
import processkit.pressbrake.PressBrakeAlarm;
import robotkit.core.RobotSnapshot;
import robotkit.runtime.RobotRuntimeCylinderDriveBlueprint;

/** Hosts the press PLC on the simulation clock; only this simulation adapter writes ram coils. */
class PressBrakeControllerPlayer implements SessionMember {
  final authored:SceneArtifactPressBrake;
  final robot:AssemblyRobot;
  final session:SimSession;
  final ram:Int;
  final depth:Int;
  final gaugeX:Int;
  final gaugeR:Null<Int>;
  final cylinders:Array<RobotRuntimeCylinderDriveBlueprint>;
  var logic:PressBrakeInterface;
  var request = new PressBrakeRobotRequest();
  var selected:Int = 0;
  var configured:Int = -1;
  var axes:Null<MachineProgramPlayer>;
  var withdrawing:Bool = false;
  var wasClosing:Bool = false;
  var releaseFromMm:Null<Float>;
  var readyAt:Float;
  public var outputs(default, null):Null<PressBrakeOutputs>;
  public var failure(default, null):Null<String>;

  public function new(authored:SceneArtifactPressBrake, robot:AssemblyRobot, session:SimSession) {
    this.authored = JsonWire.decode(JsonWire.encode(authored));
    this.robot = robot; this.session = session;
    readyAt = session.simulationTime() + session.fixedTimestep();
    ram = joint(authored.ramJoint); depth = joint(authored.depthAxis); gaugeX = joint(authored.gaugeX);
    gaugeR = authored.gaugeR == null ? null : joint(authored.gaugeR);
    cylinders = [];
    for (drive in robot.blueprint.cylinderDrives) {
      var followsRam = [for (coupling in robot.blueprint.couplings)
        if (coupling.leader == ram && coupling.follower == drive.joint) coupling].length > 0;
      if (drive.control == robotkit.runtime.CylinderControl.FlowLimited && (drive.joint == ram || followsRam))
        cylinders.push(drive);
    }
    if (cylinders.length == 0 || [for (drive in cylinders) if (drive.joint == ram) drive].length != 1)
      throw "Press controller needs its hydraulic ram binding";
    for (drive in cylinders) {
      if (drive.channelB == null || drive.channelA != cylinders[0].channelA ||
          drive.channelB != cylinders[0].channelB || drive.extendSign != 1)
        throw "Press controller needs one shared closed-centre valve and downward-positive ram travel";
      if (drive.joint != ram && [for (coupling in robot.blueprint.couplings)
          if (coupling.leader == ram && coupling.follower == drive.joint && coupling.ratio == 1 && coupling.offset == 0) coupling].length != 1)
        throw "Stop-controlled press cylinders must follow the measured ram";
    }
    for (id in [authored.topSwitch].concat(authored.fingerSwitches)) {
      var found = [for (sensor in robot.blueprint.sensors) if (sensor.id == id && sensor.kind == "joint_switch") sensor];
      if (found.length != 1) throw 'Press controller lost measured switch "$id"';
    }
    for (step in authored.steps) {
      bound(depth, step.depthTravelMm); bound(gaugeX, step.gaugeXmm);
      bound(gaugeX, step.gaugeXmm + step.withdrawalMm);
      if (gaugeR != null) bound(gaugeR, step.gaugeRmm);
      if (step.depthTravelMm * 0.001 > cylinders[0].stroke)
        throw "Press program exceeds hydraulic travel";
    }
    logic = new PressBrakeInterface(authored.requestTimeoutSeconds);
  }

  function joint(id:String):Int {
    for (i in 0...robot.model.joints.length) if (robot.model.joints[i].id == id) return i;
    throw 'Press controller lost joint "$id"';
  }

  function bound(index:Int, mm:Float):Void {
    var limit = robot.model.joints[index].limits;
    if (mm * 0.001 < limit.lower || mm * 0.001 > limit.upper)
      throw 'Press program exceeds axis "${robot.model.joints[index].id}" travel';
  }

  /** A request is sampled unchanged until the robot or script publishes a fresh image. */
  public function setRobotInput(image:PressBrakeRobotRequest):Void {
    if (image == null) throw "Press controller needs a robot request image";
    request = image;
  }

  public function robotId():String return robot.robot.id();

  function closeTo(actual:Float, targetMm:Float):Bool
    return Math.abs(actual * 1000 - targetMm) <= authored.positionToleranceMm;

  function sensor(snapshot:RobotSnapshot, id:String):Null<Bool> {
    for (i in 0...snapshot.sensors.length) {
      var frame = snapshot.sensors.get(i);
      if (frame.sensorId == id) {
        if (frame.values.length != 1 || !Math.isFinite(frame.values.get(0))) {
          fail(InvalidFeedback, 'Press switch "$id" has invalid feedback'); return null;
        }
        return frame.values.get(0) >= 0.5;
      }
    }
    fail(InvalidFeedback, 'Press switch "$id" has no sampled feedback'); return null;
  }

  function move(step:SceneArtifactPressBrakeStep, withdraw:Bool):Void {
    var ids = [authored.depthAxis, authored.gaugeX];
    var point = [step.depthTravelMm * 0.001,
      (step.gaugeXmm + (withdraw ? step.withdrawalMm : 0)) * 0.001];
    if (authored.gaugeR != null) { ids.push(authored.gaugeR); point.push(step.gaugeRmm * 0.001); }
    axes = new MachineProgramPlayer({axes: ids, waypoints: [point], loop: false,
      positionTolerance: authored.positionToleranceMm * 0.001}, robot, session);
  }

  public function feed():Void {
    if (failure != null) return;
    var snapshot = robot.robot.snapshot();
    if (snapshot.faultCode != 0 || snapshot.safety != RobotKitRuntimeConstants.RK_SAFETY_READY) {
      fail(RuntimeFailure, 'Press runtime fault ${snapshot.faultCode}, safety ${snapshot.safety}');
      return;
    }
    if (request.programStep < 0 || request.programStep >= authored.steps.length) {
      fail(InvalidProgramStep, 'Unknown press program step ${request.programStep}'); return;
    }
    for (index in [ram, depth, gaugeX])
      if (!Math.isFinite(snapshot.positions.get(index))) { fail(InvalidFeedback, "Press axis feedback is not finite"); return; }
    if (gaugeR != null && !Math.isFinite(snapshot.positions.get(gaugeR))) {
      fail(InvalidFeedback, "Press R feedback is not finite"); return;
    }
    for (cylinder in cylinders) if (!Math.isFinite(snapshot.efforts.get(cylinder.joint))) {
      fail(InvalidFeedback, "Press cylinder effort feedback is not finite"); return;
    }
    var prior = outputs;
    // Motion consumes events while it runs. The PLC keeps consuming between axis moves,
    // when sampled analogue outputs still generate native process events every tick.
    if (axes == null || !axes.motion.running) {
      var events = robot.runtime.pollEvents();
      if (events.overflow) { fail(EventOverflow, "Press runtime process event queue overflowed"); return; }
    }
    if (request.programStep != selected && (prior == null || prior.phase == Idle)) {
      selected = request.programStep; configured = -1; axes = null; withdrawing = false;
    }
    var step = authored.steps[selected];
    if (configured != selected && (prior == null || prior.phase == Idle || prior.phase == Positioning)) {
      if (axes == null) move(step, false);
    }
    if (prior != null && prior.retractBackgauge && prior.alarm == None && !withdrawing) {
      withdrawing = true; move(step, true);
    } else if (prior != null && (prior.phase == Idle || prior.phase == Positioning) && withdrawing) {
      withdrawing = false; configured = -1; move(step, false);
    }
    var moving = axes;
    if (moving != null && (prior == null || prior.alarm == None)) {
      moving.feed();
      if (moving.failure != null) { fail(AxisMotionFailure, 'Press positioning failed: ${moving.failure}'); return; }
      if (moving.passes > 0 && !withdrawing) configured = selected;
    }
    // Establish axis control before the first physics tick; switch frames arrive afterwards.
    if (session.simulationTime() < readyAt) return;
    var x = snapshot.positions.get(gaugeX);
    var depthInPosition = closeTo(snapshot.positions.get(depth), step.depthTravelMm);
    var rInPosition = gaugeR == null || closeTo(snapshot.positions.get(gaugeR), step.gaugeRmm);
    var registered = closeTo(x, step.gaugeXmm) && rInPosition;
    var withdrawn = closeTo(x, step.gaugeXmm + step.withdrawalMm) && rInPosition;
    var travelMm = snapshot.positions.get(ram) * 1000;
    var atTop = sensor(snapshot, authored.topSwitch);
    if (atTop == null) return;
    var fingerLeft = sensor(snapshot, authored.fingerSwitches[0]);
    if (fingerLeft == null) return;
    var fingerRight = sensor(snapshot, authored.fingerSwitches[1]);
    if (fingerRight == null) return;
    var atBottom = travelMm >= step.depthTravelMm - authored.positionToleranceMm;
    var atClamp = !atBottom && travelMm >= step.clampTravelMm - authored.positionToleranceMm;
    // Quasi-static unloading: measured lift plus low net closing actuator load.
    // This is the simulation's declared unloading model, not inferred chamber pressure.
    var closingForce = 0.0;
    for (cylinder in cylinders) closingForce += snapshot.efforts.get(cylinder.joint) * cylinder.extendSign;
    var releaseOrigin = releaseFromMm;
    var unloaded = !wasClosing || releaseOrigin != null && travelMm <= releaseOrigin - authored.releaseTravelMm &&
      Math.max(0, closingForce) <= authored.releaseClosingForceN &&
      prior != null && (prior.phase == Decompressing || prior.phase == Opening || prior.phase == Idle || prior.phase == Positioning);
    var image = new PressBrakeInput(request.requestStroke, request.requestClampOnly, request.requestOpen,
      request.programStep, request.clear, request.holding, request.faultReset, request.safetyReady,
      fingerLeft, fingerRight,
      configured < 0 ? selected : configured, registered, depthInPosition, withdrawn,
      atTop, atClamp, atBottom, unloaded, travelMm);
    var result = logic.update(image, session.fixedTimestep());
    outputs = result;
    if (result.alarm != None && (prior == null || prior.alarm == None)) {
      if (axes != null) axes.beforeReset();
      robot.robot.stop(robotkit.core.StopMode.Normal);
    }
    if (result.command == CloseToClamp || result.command == CloseToDepth) wasClosing = true;
    if (result.command == Decompress && (prior == null || prior.phase != Decompressing)) releaseFromMm = travelMm;
    publish(result);
    var extend = result.command == CloseToClamp || result.command == CloseToDepth;
    // Stop fluid displacement at measured home. Continuing return flow can compress
    // the compliant end stop beyond the switch window; acknowledgement still uses the switch.
    var retract = result.command == Decompress || result.command == Open && travelMm > 0;
    robot.runtime.writeDigitalOutput(cylinders[0].channelA, extend);
    var channelB = cylinders[0].channelB;
    if (channelB == null) throw "Press return coil disappeared";
    robot.runtime.writeDigitalOutput(channelB, retract);
  }

  function publish(image:PressBrakeOutputs):Void {
    robot.runtime.writeDigitalOutput(PressBrakeChannels.READY, image.ready);
    robot.runtime.writeDigitalOutput(PressBrakeChannels.BACKGAUGE_IN_POSITION, image.backgaugeInPosition);
    robot.runtime.writeDigitalOutput(PressBrakeChannels.AT_TOP, image.atTop);
    robot.runtime.writeDigitalOutput(PressBrakeChannels.CLAMPING, image.clamping);
    robot.runtime.writeDigitalOutput(PressBrakeChannels.AT_BOTTOM, image.atBottom);
    robot.runtime.writeDigitalOutput(PressBrakeChannels.DECOMPRESSED, image.decompressed);
    robot.runtime.writeDigitalOutput(PressBrakeChannels.ALARM, image.alarm != None);
    robot.runtime.writeOutput(PressBrakeChannels.ALARM_CODE, robotkit.execution.ProcessEventValue.Analog(PressBrakeChannels.alarmCode(image.alarm)));
    robot.runtime.writeOutput(PressBrakeChannels.RAM_POSITION_MM, robotkit.execution.ProcessEventValue.Analog(image.ramPositionMm));
  }

  function fail(alarm:PressBrakeAlarm, message:String):Void {
    failure = message; logic.trip(alarm);
    if (axes != null) axes.beforeReset();
    robot.robot.stop(robotkit.core.StopMode.Normal);
    // Retain the last measured feedback; a hosting failure cannot supply a new sample.
    var measured = outputs;
    var nativeReady = robot.robot.snapshot().safety == RobotKitRuntimeConstants.RK_SAFETY_READY;
    if (measured != null) {
      outputs = new PressBrakeOutputs(Fault, logic.alarm, Hold, false,
        measured.backgaugeInPosition, measured.atTop, false, measured.atBottom,
        measured.decompressed, false, measured.ramPositionMm);
      if (nativeReady) publish(outputs);
    }
    if (measured == null || !nativeReady) {
      robot.runtime.writeDigitalOutput(PressBrakeChannels.READY, false);
      robot.runtime.writeDigitalOutput(PressBrakeChannels.ALARM, true);
      robot.runtime.writeOutput(PressBrakeChannels.ALARM_CODE,
        robotkit.execution.ProcessEventValue.Analog(PressBrakeChannels.alarmCode(nativeReady ? logic.alarm : RuntimeFailure)));
    }
    robot.runtime.writeDigitalOutput(cylinders[0].channelA, false);
    var channelB = cylinders[0].channelB;
    if (channelB == null) throw "Press return coil disappeared";
    robot.runtime.writeDigitalOutput(channelB, false);
  }

  public function beforeReset():Void {
    if (axes != null) axes.beforeReset();
    robot.robot.stop(robotkit.core.StopMode.Normal);
  }
  public function reset():Void {
    logic = new PressBrakeInterface(authored.requestTimeoutSeconds);
    request = new PressBrakeRobotRequest(); selected = 0; configured = -1;
    axes = null; withdrawing = false; wasClosing = false; releaseFromMm = null; outputs = null; failure = null;
    readyAt = session.simulationTime() + session.fixedTimestep();
  }
  public function present():Void {}
}
