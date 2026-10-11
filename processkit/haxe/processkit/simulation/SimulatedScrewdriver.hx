package processkit.simulation;

import haxe.Int64;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import robotkit.runtime.Simulation;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.SimulatedSuctionTool;
import robotkit.runtime.SimulationStepObserver;
import robotkit.tool.ScrewdriverContract;
import processkit.screwdriving.*;
import processkit.screwdriving.TighteningModel.TighteningJoint;
import processkit.screwdriving.ScrewdriverProcessDevice.ScrewdriverChannelNames;

/** Starting head/socket frame (+Z into the screw), body pose and lead-in point, following the live work in metres. */
typedef ScrewdriverWorkState = {
  var head:SimPose;
  var screwPose:SimPose;
  var engagementPoint:Array<Float>;
  var hole:Array<Float>;
  var direction:Array<Float>;
  var damagedLeadIn:Bool;
};
/** A qualified slot binds one identified screw and a live work frame, not a proximity guess. */
typedef ScrewdriverSetup = {
  var program:TighteningProgram;
  var joint:TighteningJoint;
  var screw:Null<SimObject>;
  var work:Void -> ScrewdriverWorkState;
  /** +1 for right-hand tightening, -1 for left-hand. */
  var hand:Float;
};

/** Deterministic screw process on a compliant tool; only its two tool joints are driven. */
class SimulatedScrewdriver implements SimulationStepObserver implements ScrewdriverFeedback {
  public final simulation:Simulation;
  public final runtime:RobotRuntime;
  public final holding:SimulatedSuctionTool;
  public final channels:ScrewdriverChannelNames;
  public final sensorId:String;
  public var model(default, null):Null<TighteningModel> = null;
  final robotIndex:Int;
  final linkIndex:Int;
  final feedJoint:Int;
  final spindleJoint:Int;
  final tipPosition:Array<Float>;
  final axis:Array<Float>;
  final strokeLimitMm:Float;
  final maxSpeed:Float;
  final maxTorque:Float;
  final springRate:Float;
  final setups:Map<Int, ScrewdriverSetup> = new Map();
  var selected:Null<ScrewdriverSetup> = null;
  var lastTime:Null<Int64> = null;
  var sequence = 0;
  var angle = 0.0;
  var reversed = false;
  var reverseTargetAngle = 0.0;
  var origin = 0.0;
  var activeSlot = 0;
  var lastStart = false;
  var observed:ScrewdriverReading;

  public function new(simulation:Simulation, runtime:RobotRuntime, robotIndex:Int, linkIndex:Int,
      feedJoint:Int, spindleJoint:Int, tipPosition:Array<Float>, axis:Array<Float>, channels:ScrewdriverChannelNames,
      sensorId:String, strokeLimitMm:Float, maxSpeed:Float, maxTorque:Float, springRate:Float) {
    if (simulation == null || runtime == null || channels == null || sensorId == null ||
        robotIndex < 0 || linkIndex < 0 || feedJoint < 0 || spindleJoint < 0 || feedJoint == spindleJoint ||
        tipPosition == null || tipPosition.length != 3 || axis == null || axis.length != 3)
      throw "Simulated screwdriver needs its mounted tool, joints, channels and sensor";
    for (value in [strokeLimitMm, maxSpeed, maxTorque, springRate])
      if (!Math.isFinite(value) || value <= 0) throw "Simulated screwdriver needs positive finite ratings";
    var norm = Math.sqrt(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2]);
    if (!Math.isFinite(norm) || norm <= 0) throw "Simulated screwdriver needs a finite tool axis";
    this.simulation = simulation; this.runtime = runtime; this.robotIndex = robotIndex; this.linkIndex = linkIndex;
    this.feedJoint = feedJoint; this.spindleJoint = spindleJoint; this.tipPosition = tipPosition.copy();
    this.axis = [for (value in axis) value / norm]; this.channels = channels; this.sensorId = sensorId;
    this.strokeLimitMm = strokeLimitMm; this.maxSpeed = maxSpeed; this.maxTorque = maxTorque; this.springRate = springRate;
    holding = new SimulatedSuctionTool(simulation, runtime, robotIndex, linkIndex, null, []);
    observed = ScrewdriverSensor.idle();
  }
  public function configure(slot:Int, setup:ScrewdriverSetup):Void {
    if (slot <= 0 || setup == null || setup.program == null || setup.joint == null || setup.work == null ||
        (setup.hand != 1 && setup.hand != -1)) throw "Screwdriver slot needs an identified screw, live frame and qualified program";
    if (lastStart) throw "Cannot configure a screwdriver while its start output is on";
    var settings = setup.program.settings;
    if (settings.runDownSpeed > maxSpeed || settings.finalSpeed > maxSpeed || settings.finalTorqueLimit > maxTorque ||
        settings.maximumStroke > strokeLimitMm || setup.joint.springRate != springRate)
      throw "Screwdriver program exceeds its capacity, stroke or spring rating";
    var checked = new TighteningModel(setup.program, setup.joint);
    if (holding.held != null && holding.held != setup.screw) throw "Qualified slot does not describe the held screw";
    setups.set(slot, setup);
    if (activeSlot == slot) { activeSlot = 0; selected = null; model = null; }
  }
  public function reading():ScrewdriverReading return ScrewdriverSensor.reading(ScrewdriverSensor.values(observed));
  public function reset():Void {
    holding.reset(); model = null; selected = null; lastTime = null; angle = 0; reversed = false; reverseTargetAngle = 0; origin = 0;
    activeSlot = 0; lastStart = false; observed = ScrewdriverSensor.idle();
  }
  public function afterSimulationStep(timestamp:Int64):Void {
    var previous = lastTime; lastTime = timestamp;
    var dt = previous == null ? 0.0 : Math.max(0, Int64.toFloat(Int64.sub(timestamp, previous)) * 1e-9);
    var start = digital(channels.start), vacuum = digital(channels.vacuum), reverse = digital(channels.direction);
    var slotValue = analog(channels.program);
    var slot = ScrewdriverProcessDevice.programSlot(slotValue, true);
    if (start && lastStart && slot != activeSlot) throw "Screwdriver program cannot change during a cycle";
    if (slot != activeSlot || (start && selected == null)) {
      var setup = setups.get(slot);
      if (setup == null && start) throw 'Screwdriver slot $slot has no qualified joint program';
      selected = setup; activeSlot = slot; model = setup == null ? null : new TighteningModel(setup.program, setup.joint); angle = 0; reversed = false; reverseTargetAngle = 0; origin = runtime.snapshot().q.get(spindleJoint);
    }
    if (start && reverse && model == null) throw "Cannot reverse without a configured screw cycle";
    lastStart = start;
    var setup = selected, process = model;
    if (start && !reverse && reversed) throw "Screwdriver needs a newly configured cycle after reversal";
    if (!vacuum && holding.held != null) holding.release();
    if (setup != null && process != null) {
      var state = setup.work();
      if (state.hole == null || state.hole.length != 3 || state.direction == null || state.direction.length != 3 || state.head == null || state.screwPose == null ||
          state.engagementPoint == null || state.engagementPoint.length != 3)
        throw "Screwdriver live work needs a head and hole axis";
      var link = simulation.linkPose(robotIndex, linkIndex);
      var orientation = new robotkit.spatial.Quat(link.rotation[0],link.rotation[1],link.rotation[2],link.rotation[3]);
      var localTip = orientation.rotate(new robotkit.spatial.Vec3(tipPosition[0],tipPosition[1],tipPosition[2]));
      var tip = [link.position[0]+localTip.x,link.position[1]+localTip.y,link.position[2]+localTip.z];
      var rotatedAxis = orientation.rotate(new robotkit.spatial.Vec3(axis[0],axis[1],axis[2]));
      var direction = [rotatedAxis.x,rotatedAxis.y,rotatedAxis.z];
      var holeAxis = state.direction;
      var norm = Math.sqrt(dot(holeAxis,holeAxis));
      if (!Math.isFinite(norm) || norm <= 0) throw "Screwdriver work axis must be finite and nonzero";
      holeAxis = [for (value in holeAxis) value / norm];
      // Undo the actual feed retraction before comparing the commanded free tip with the head.
      var measured = runtime.snapshot();
      var q = measured.q;
      var driveSpeed = 0.0;
      var feed = q.get(feedJoint);
      var freeTip = [for (i in 0...3) tip[i] - direction[i] * feed];
      var advanced = process.advanceAtAngle(angle) / 1000;
      var toHead = [state.head.x + holeAxis[0]*advanced - freeTip[0],
        state.head.y + holeAxis[1]*advanced - freeTip[1], state.head.z + holeAxis[2]*advanced - freeTip[2]];
      var stroke = Math.max(0, -dot(toHead, direction) * 1000);
      var headError = [for (i in 0...3) state.engagementPoint[i] - state.hole[i]];
      var along = dot(headError,holeAxis);
      var radial = Math.sqrt(Math.max(0,dot(headError,headError)-along*along))*1000;
      var toolError = [for (i in 0...3) freeTip[i]-state.hole[i]];
      var toolAlong = dot(toolError,holeAxis);
      radial = Math.max(radial,Math.sqrt(Math.max(0,dot(toolError,toolError)-toolAlong*toolAlong))*1000);
      var headAxis = new robotkit.spatial.Quat(state.head.qx,state.head.qy,state.head.qz,state.head.qw).rotate(new robotkit.spatial.Vec3(0,0,1));
      var screwAlignment = Math.acos(Math.max(-1,Math.min(1,headAxis.x*holeAxis[0]+headAxis.y*holeAxis[1]+headAxis.z*holeAxis[2])));
      var alignment = Math.acos(Math.max(-1,Math.min(1,dot(direction,holeAxis))));
      alignment = Math.max(alignment,screwAlignment);
      var screw = setup.screw;
      if (vacuum && screw != null && holding.held == null && Math.sqrt(dot(toHead,toHead)) <= SimulatedSuctionTool.REACH) {
        var actual = simulation.objectPose(screw);
        var gap = [actual.x-state.screwPose.x,actual.y-state.screwPose.y,actual.z-state.screwPose.z];
        if (Math.sqrt(dot(gap,gap)) <= SimulatedSuctionTool.REACH) holding.attach(screw, actual);
      }
      if (reverse && start) {
        if (!reversed) { reversed = true; reverseTargetAngle = angle; }
        reverseTargetAngle = Math.max(0, reverseTargetAngle - setup.program.settings.runDownSpeed * dt);
        angle = Math.max(0, setup.hand*(q.get(spindleJoint)-origin));
        // Numerical zero for a settled spindle, while retaining measured reversal feedback.
        if (angle < 1e-6) angle = 0;
        observed.torqueNm = 0; observed.angle = angle; observed.speed = -setup.program.settings.runDownSpeed;
        if (angle == 0) observed.speed = 0;
        driveSpeed = observed.speed;
        observed.strokeMm = stroke; observed.vacuum = holding.held != null;
      } else if (!reversed) {
        if (start) angle = Math.max(angle, setup.hand*(q.get(spindleJoint)-origin));
        observed = process.step({screwPresent:setup.screw != null && holding.held == setup.screw, vacuum:vacuum && holding.held != null,
          start:start, angle:angle, strokeMm:stroke, radialErrorMm:radial, axialErrorMm:along*1000, axisAngle:alignment, damagedLeadIn:state.damagedLeadIn});
        if (observed.result != ScrewdriverContract.NONE) angle = observed.angle;
        driveSpeed = observed.speed;
      }
      if (!start) { driveSpeed = 0; if (reversed) reverseTargetAngle = angle; }
      simulation.driveProcessJoint(robotIndex, feedJoint, -Math.min(strokeLimitMm,stroke)/1000, 0);
      var targetAngle = start && reverse ? reverseTargetAngle : angle + driveSpeed*dt;
      simulation.driveProcessJoint(robotIndex, spindleJoint, origin + setup.hand*targetAngle,
        setup.hand*driveSpeed);
      observed.speed = setup.hand*measured.dq.get(spindleJoint);
      if (holding.held != null) {
        var advance = process.advanceAtAngle(angle)/1000;
        var rotation = robotkit.spatial.Quat.fromAxisAngle(new robotkit.spatial.Vec3(holeAxis[0],holeAxis[1],holeAxis[2]), setup.hand*angle);
        var initial = new robotkit.spatial.Quat(state.screwPose.qx,state.screwPose.qy,state.screwPose.qz,state.screwPose.qw);
        var turned = rotation.multiply(initial);
        var offset = rotation.rotate(new robotkit.spatial.Vec3(state.screwPose.x-state.head.x,
          state.screwPose.y-state.head.y,state.screwPose.z-state.head.z));
        holding.placeHeld(new SimPose(state.head.x+offset.x+holeAxis[0]*advance,state.head.y+offset.y+holeAxis[1]*advance,
          state.head.z+offset.z+holeAxis[2]*advance,turned.x,turned.y,turned.z,turned.w));
      }
    } else {
      observed = ScrewdriverSensor.idle();
      var measured = runtime.snapshot();
      observed.strokeMm = Math.max(0,-measured.q.get(feedJoint)*1000);
      observed.speed = measured.dq.get(spindleJoint);
      observed.vacuum = holding.held != null;
      simulation.driveProcessJoint(robotIndex,feedJoint,0,0);
      simulation.driveProcessJoint(robotIndex,spindleJoint,measured.q.get(spindleJoint),0);
    }
    runtime.publishSensorFrame(sensorId, ScrewdriverSensor.values(observed), Int64.ofInt(++sequence),
      runtime.snapshot().sourceTimestampNs, runtime.endpoint.sourceClockId());
  }
  static function dot(a:Array<Float>,b:Array<Float>):Float return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
  function digital(channel:String):Bool return switch runtime.channelValue(channel) {
    case Digital(on): on; case _: throw 'Screwdriver channel "$channel" needs digital values';
  };
  function analog(channel:String):Float return switch runtime.channelValue(channel) {
    case Analog(value): value; case _: throw 'Screwdriver channel "$channel" needs analog values';
  };
}
