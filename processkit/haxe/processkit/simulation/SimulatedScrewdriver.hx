package processkit.simulation;

import haxe.Int64;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import robotkit.runtime.Simulation;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.SimulatedSuctionTool;
import robotkit.runtime.SimulationStepObserver;
import robotkit.tool.ScrewdriverContract;
import robotkit.spatial.Transform3;
import robotkit.spatial.Quat;
import robotkit.spatial.Vec3;
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
  /** On successful release, retain the actual seated body on its identified threaded member. */
  @:optional var retain:SimObject -> SimPose -> Void;
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
  /** Starting body relative to the live socket frame, after physical lead-in contact. */
  var anchoredBody:Null<Transform3> = null;

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
  /** Absolute measured shaft coordinate, used to align the next free-bit approach. */
  public function spindlePosition():Float return runtime.snapshot().q.get(spindleJoint);
  public function reading():ScrewdriverReading return ScrewdriverSensor.reading(ScrewdriverSensor.values(observed));
  public function reset():Void {
    holding.reset(); anchoredBody = null; model = null; selected = null; lastTime = null; angle = 0; reversed = false; reverseTargetAngle = 0; origin = 0;
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
      anchoredBody = null; selected = setup; activeSlot = slot; model = setup == null ? null : new TighteningModel(setup.program, setup.joint); angle = 0; reversed = false; reverseTargetAngle = 0; origin = runtime.snapshot().q.get(spindleJoint);
    }
    if (start && reverse && model == null) throw "Cannot reverse without a configured screw cycle";
    lastStart = start;
    var setup = selected, process = model;
    if (start && !reverse && reversed) throw "Screwdriver needs a newly configured cycle after reversal";
    if (!vacuum) {
      var held = holding.held;
      if (held != null) {
        var pose = simulation.objectPose(held);
        var retain = setup == null ? null : setup.retain;
        var fastened = anchoredBody != null && process != null && process.reading.result == ScrewdriverContract.OK;
        holding.release();
        if (fastened && retain != null) retain(held,pose);
      }
      anchoredBody = null;
    }
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
      var screw = setup.screw;
      var referenceHead = transform(state.head);
      var bodyToHead = transform(state.screwPose).inverse().compose(referenceHead);
      var leadInHead = referenceHead.inverse().transformPoint(Vec3.fromArray(state.engagementPoint));
      var actualBody = screw == null ? transform(state.screwPose) : transform(simulation.objectPose(screw));
      var actualHead = actualBody.compose(bodyToHead);
      var headAxis = actualHead.transformVector(new Vec3(0,0,1));
      var pickError = actualHead.translation.sub(Vec3.fromArray(tip));
      var pickAlignment = Math.acos(Math.max(-1,Math.min(1,headAxis.x*direction[0]+headAxis.y*direction[1]+headAxis.z*direction[2])));
      // The named socket is calibrated against the body's collision origin. Only the
      // identified body can seal; another screw near the bit is never substituted.
      if (vacuum && screw != null && holding.held == null && pickError.norm() <= SimulatedSuctionTool.REACH &&
          pickAlignment <= setup.joint.maximumAxisAngle) holding.attach(screw, pose(actualBody));
      var lead = actualHead.transformPoint(leadInHead);
      var leadError = lead.sub(Vec3.fromArray(state.hole));
      var along = leadError.dot(Vec3.fromArray(holeAxis));
      var radial = Math.sqrt(Math.max(0,leadError.dot(leadError)-along*along))*1000;
      var screwAlignment = Math.acos(Math.max(-1,Math.min(1,headAxis.x*holeAxis[0]+headAxis.y*holeAxis[1]+headAxis.z*holeAxis[2])));
      var alignment = Math.max(pickAlignment,screwAlignment);
      var anchor = anchoredBody;
      if (anchor == null && holding.held != null && !reversed &&
          radial <= setup.joint.leadInMm && Math.abs(along)*1000 <= setup.joint.leadInMm &&
          alignment <= setup.joint.maximumAxisAngle) {
        // Retain the actual contact pose rather than snapping to the nominal CAD axis.
        anchor = referenceHead.inverse().compose(actualBody);
        anchoredBody = anchor;
      }
      var startingBody = anchor == null ? actualBody : referenceHead.compose(anchor);
      var startingHead = startingBody.compose(bodyToHead);
      var head = startingHead.translation;
      var leadStart = startingHead.transformPoint(leadInHead);
      var headError = leadStart.sub(Vec3.fromArray(state.hole));
      along = headError.dot(Vec3.fromArray(holeAxis));
      radial = Math.sqrt(Math.max(0,headError.dot(headError)-along*along))*1000;
      var toolError = [for (i in 0...3) freeTip[i]-state.hole[i]];
      var toolAlong = dot(toolError,holeAxis);
      radial = Math.max(radial,Math.sqrt(Math.max(0,dot(toolError,toolError)-toolAlong*toolAlong))*1000);
      var advanced = process.advanceAtAngle(angle) / 1000;
      var toHead = [head.x + holeAxis[0]*advanced - freeTip[0],
        head.y + holeAxis[1]*advanced - freeTip[1],head.z + holeAxis[2]*advanced - freeTip[2]];
      // During transfer the bit carries the screw freely. Compliance only compresses
      // against a contacted thread, never against the distant authored work frame.
      var stroke = anchor == null ? 0.0 : Math.max(0,-dot(toHead,direction)*1000);
      if (anchor != null && reversed && angle == 0 && !start && dot(toHead,direction) > SimulatedSuctionTool.REACH) {
        anchoredBody = null; anchor = null; stroke = 0;
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
      if (holding.held != null && anchor != null) {
        var advance = process.advanceAtAngle(angle)/1000;
        var rotation = robotkit.spatial.Quat.fromAxisAngle(new robotkit.spatial.Vec3(holeAxis[0],holeAxis[1],holeAxis[2]), setup.hand*angle);
        var initial = startingBody.rotation;
        var turned = rotation.multiply(initial);
        var offset = rotation.rotate(new robotkit.spatial.Vec3(startingBody.translation.x-head.x,
          startingBody.translation.y-head.y,startingBody.translation.z-head.z));
        holding.placeHeld(new SimPose(head.x+offset.x+holeAxis[0]*advance,head.y+offset.y+holeAxis[1]*advance,
          head.z+offset.z+holeAxis[2]*advance,turned.x,turned.y,turned.z,turned.w));
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
    // The process result retains its final angle. Live feedback follows the
    // measured shaft through stopping and reversal, including physical coast.
    var measuredAngle = selected == null ? observed.angle : Math.max(0,selected.hand*(runtime.snapshot().q.get(spindleJoint)-origin));
    observed = {torqueNm:observed.torqueNm,angle:measuredAngle,speed:observed.speed,
      strokeMm:observed.strokeMm,phase:observed.phase,result:observed.result,vacuum:holding.held != null};
    runtime.publishSensorFrame(sensorId, ScrewdriverSensor.values(observed), Int64.ofInt(++sequence),
      runtime.snapshot().sourceTimestampNs, runtime.endpoint.sourceClockId());
  }
  static function dot(a:Array<Float>,b:Array<Float>):Float return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
  static function transform(value:SimPose):Transform3
    return new Transform3(new Vec3(value.x,value.y,value.z),new Quat(value.qx,value.qy,value.qz,value.qw));
  static function pose(value:Transform3):SimPose
    return new SimPose(value.translation.x,value.translation.y,value.translation.z,
      value.rotation.x,value.rotation.y,value.rotation.z,value.rotation.w);
  function digital(channel:String):Bool return switch runtime.channelValue(channel) {
    case Digital(on): on; case _: throw 'Screwdriver channel "$channel" needs digital values';
  };
  function analog(channel:String):Float return switch runtime.channelValue(channel) {
    case Analog(value): value; case _: throw 'Screwdriver channel "$channel" needs analog values';
  };
}
