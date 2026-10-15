package processkit.simulation;

import haxe.Int64;
import NativeKitSim;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import robotkit.runtime.Simulation;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.SimulatedSuctionTool;
import robotkit.runtime.SimulationStepObserver;
import robotkit.runtime.SimulationObjectFixture;
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
  @:optional var fixture:SimulationObjectFixture;
  /** Exact CAD connection; required when a released joint can be continued. */
  @:optional var connection:{fastening:String,member:String,face:String};
  @:optional var retightening:Bool;
  /** Selects the solver-coupled screw body instead of the process-model pose. */
  @:optional var physicalThread:Bool;
  /** CAD thread lead, distinct from the qualified feed-stroke model, in millimetres per revolution. */
  @:optional var threadLeadMm:Float;
  /** Explicit simulation force cap for the solver-coupled physical thread slide, in newtons. */
  @:optional var physicalThreadMaxForceN:Float;
  /** Axial stiffness of the engaged thread, in N/m. */
  @:optional var physicalThreadStiffnessNPerM:Float;
  /** The arm owns axial search, engagement or recovery motion; lock the compliant feed slide. */
  @:optional var armOwnsAxialMotion:Null<Void->Bool>;
};

/**
 * How the bit carries the identified screw. Exactly one body path owns the screw at a time, and the
 * topology changes only between physics ticks.
 */
private enum ScrewCarriage {
  Empty;
  /** Held by vacuum on the bit link; the process model places it while it is driven. */
  Suction(screw:SimObject);
  /** A rigid physical joint to the bit link while the arm owns axial motion, so contact loads the feed and arm. */
  Welded(screw:SimObject, joint:nksim_joint);
  /** On a helical joint in its threaded member, turned through the socket: the solver carries run-down. */
  Threaded(screw:SimObject, joint:nksim_joint);
  /**
   * Seated after a physical run-down: retained on its member at the measured seat, still in the socket. The
   * process model turns the final angle, so the frictionless thread never carries the clamp load.
   */
  Seated(screw:SimObject);
  /**
   * Held by the thread start on its threaded member (the process model's anchor), turned there by the process model
   * through the socket. Its contacts with the work are the member's, not tool load.
   */
  Engaged(screw:SimObject);
}

private typedef RetainedEngagement = {
  var fastening:String;
  var member:String;
  var face:String;
  var screw:SimObject;
  var hand:Float;
  var model:TighteningModel;
  var anchor:Transform3;
  var bodyToHead:Transform3;
  var leadInHead:Vec3;
  var fixture:SimulationObjectFixture;
};

/** Deterministic screw process on a compliant tool; only its two tool joints are driven. */
class SimulatedScrewdriver implements SimulationStepObserver implements ScrewdriverFeedback {
  public final simulation:Simulation;
  public final runtime:RobotRuntime;
  public final holding:SimulatedSuctionTool;
  public final channels:ScrewdriverChannelNames;
  public final sensorId:String;
  public var model(default, null):Null<TighteningModel> = null;
  public var supply:Null<SimulationScrewdriverSupply> = null;
  final robotIndex:Int;
  final linkIndex:Int;
  final feedJoint:Int;
  final spindleJoint:Int;
  /** Everything the spindle turns, about its axis (kg m^2). */
  final spindleInertia:Float;
  /** Everything the feed slide carries (kg). */
  final feedMass:Float;
  final tipPosition:Array<Float>;
  final axis:Array<Float>;
  final strokeLimitMm:Float;
  final maxSpeed:Float;
  final maxTorque:Float;
  final springRate:Float;
  /** Signed alignment of positive spindle rotation with the screwdriver's +Z axis. */
  final spindleAxisAlignment:Float;
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
  final retained:Array<RetainedEngagement> = [];
  var pendingEngagement:Null<RetainedEngagement> = null;
  var carriage:ScrewCarriage = Empty;

  public function new(simulation:Simulation, runtime:RobotRuntime, robotIndex:Int, linkIndex:Int,
      feedJoint:Int, spindleJoint:Int, tipPosition:Array<Float>, axis:Array<Float>, channels:ScrewdriverChannelNames,
      sensorId:String, strokeLimitMm:Float, maxSpeed:Float, maxTorque:Float, springRate:Float,
      spindleAxisAlignment:Float, spindleInertia:Float, feedMass:Float) {
    if (simulation == null || runtime == null || channels == null || sensorId == null ||
        robotIndex < 0 || linkIndex < 0 || feedJoint < 0 || spindleJoint < 0 || feedJoint == spindleJoint ||
        tipPosition == null || tipPosition.length != 3 || axis == null || axis.length != 3)
      throw "Simulated screwdriver needs its mounted tool, joints, channels and sensor";
    for (value in [strokeLimitMm, maxSpeed, maxTorque, springRate, spindleInertia, feedMass])
      if (!Math.isFinite(value) || value <= 0) throw "Simulated screwdriver needs positive finite ratings and spindle inertia";
    var norm = Math.sqrt(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2]);
    if (!Math.isFinite(norm) || norm <= 0) throw "Simulated screwdriver needs a finite tool axis";
    if (!Math.isFinite(spindleAxisAlignment) || Math.abs(spindleAxisAlignment) > 1.0 + 1e-8)
      throw "Simulated screwdriver needs a finite spindle-axis alignment";
    this.simulation = simulation; this.runtime = runtime; this.robotIndex = robotIndex; this.linkIndex = linkIndex;
    this.feedJoint = feedJoint; this.spindleJoint = spindleJoint; this.tipPosition = tipPosition.copy();
    this.spindleInertia = spindleInertia;
    this.feedMass = feedMass;
    this.axis = [for (value in axis) value / norm]; this.channels = channels; this.sensorId = sensorId;
    this.strokeLimitMm = strokeLimitMm; this.maxSpeed = maxSpeed; this.maxTorque = maxTorque; this.springRate = springRate;
    this.spindleAxisAlignment = Math.max(-1, Math.min(1, spindleAxisAlignment));
    holding = new SimulatedSuctionTool(simulation, runtime, robotIndex, linkIndex, null, []);
    observed = ScrewdriverSensor.idle();
  }
  public function configure(slot:Int, setup:ScrewdriverSetup):Null<processkit.screwdriving.ScrewPlan.ScrewFinalContact> {
    if (slot <= 0 || setup == null || setup.program == null || setup.joint == null || setup.work == null ||
        (setup.hand != 1 && setup.hand != -1)) throw "Screwdriver slot needs an identified screw, live frame and qualified program";
    if (lastStart) throw "Cannot configure a screwdriver while its start output is on";
    if (threadedScrew() != null && threadedScrew() != setup.screw)
      throw "Cannot replace the qualified screw while its physical thread slide is attached";
    var settings = setup.program.settings;
    if (settings.runDownSpeed > maxSpeed || settings.finalSpeed > maxSpeed || settings.finalTorqueLimit > maxTorque ||
        settings.maximumStroke > strokeLimitMm || setup.joint.springRate != springRate)
      throw "Screwdriver program exceeds its capacity, stroke or spring rating";
    var checked = new TighteningModel(setup.program, setup.joint);
    if (carried() != null && carried() != setup.screw) throw "Qualified slot does not describe the held screw";
    var connection = setup.connection;
    if (setup.physicalThread == true) {
      requiredPositive(setup.threadLeadMm,"Physical screw engagement needs a positive CAD thread lead");
      requiredPositive(setup.physicalThreadMaxForceN,"Physical screw engagement needs an explicit positive axial force cap");
      requiredPositive(setup.physicalThreadStiffnessNPerM,"Physical screw engagement needs an explicit positive thread stiffness");
      if (setup.screw == null || setup.fixture == null || connection == null)
        throw "Physical screw engagement needs its identified screw and retained fixture";
    }
    if (setup.physicalThread == true && Math.abs(Math.abs(spindleAxisAlignment) - 1.0) > 1e-6)
      throw "Physical screw engagement requires the spindle axis to be collinear with the tool thread axis";
    if (setup.fixture != null || setup.retightening == true) {
      if (connection == null || connection.fastening == null || connection.fastening.length == 0 ||
          connection.member == null || connection.member.length == 0 || connection.face == null || connection.face.length == 0)
        throw "Retained screw needs its exact fastening, threaded member and named face";
    }
    var contact:Null<processkit.screwdriving.ScrewPlan.ScrewFinalContact> = null;
    if (setup.retightening == true) {
      if (carried() != null) throw "Final setup needs a successfully released screw";
      var engagement = retainedFor(setup);
      var continued = engagement.model.retighten(setup.program,setup.joint);
      var socket = transform(simulation.objectPose(engagement.screw)).compose(engagement.bodyToHead);
      contact = {socket:socket,advanceMm:continued.advanceMm};
    }
    setups.set(slot, setup);
    if (activeSlot == slot) { activeSlot = 0; selected = null; model = null; }
    return contact;
  }
  function retainedFor(setup:ScrewdriverSetup):RetainedEngagement {
    var connection = setup.connection;
    if (connection == null || setup.screw == null) throw "Final setup needs its identified released screw and connection";
    for (engagement in retained) if (engagement.screw == setup.screw && engagement.fastening == connection.fastening &&
        engagement.member == connection.member && engagement.face == connection.face) {
      if (engagement.hand != setup.hand) throw "Final setup cannot change the engaged thread hand";
      if (!engagement.fixture.carries(engagement.screw)) {
        retained.remove(engagement);
        throw 'Fastening "${connection.fastening}" is no longer retained by its named threaded member "${connection.member}"';
      }
      var fixture = setup.fixture;
      if (fixture == null || fixture != engagement.fixture || !fixture.carries(engagement.screw))
        throw 'Fastening "${connection.fastening}" final setup does not identify its original release fixture for member "${connection.member}"';
      return engagement;
    }
    throw 'Fastening "${connection.fastening}" has no retained successful engagement for its exact screw, member and face';
  }
  /** Absolute measured shaft coordinate, used to align the next free-bit approach. */
  public function spindlePosition():Float return supply == null ? runtime.snapshot().q.get(spindleJoint) : supply.joints().spindlePosition;
  /** Receiving interlock uses measured angular/feed speeds with explicit commissioned limits. */
  public function receivingReady(angularSpeed:Float, feedSpeed:Float):Bool {
    if (!Math.isFinite(angularSpeed) || angularSpeed <= 0 || !Math.isFinite(feedSpeed) || feedSpeed <= 0)
      throw "Receiving speed bounds must be commissioned positive values";
    if (carried() != null) return false;
    var measured = runtime.snapshot();
    for (i in 0...measured.dq.length) if (Math.abs(measured.dq.get(i)) > (i == feedJoint ? feedSpeed : angularSpeed)) return false;
    return !digital(channels.start) && !digital(channels.direction);
  }
  public function reading():ScrewdriverReading return supply == null ? ScrewdriverSensor.reading(ScrewdriverSensor.values(observed)) : supply.reading();
  public function reset():Void {
    beforeReset();
    holding.reset(); carriage = Empty; anchoredBody = null; model = null; selected = null; lastTime = null; angle = 0; reversed = false; reverseTargetAngle = 0; origin = 0;
    activeSlot = 0; lastStart = false; observed = ScrewdriverSensor.idle();
    retained.resize(0); pendingEngagement = null;
    if (supply != null) supply.reset();
  }
  /** The identified screw the bit carries by any path, or null. */
  public function carried():Null<SimObject> return switch carriage {
    case Empty: null;
    case Suction(screw): screw;
    case Welded(screw, _): screw;
    case Threaded(screw, _): screw;
    case Seated(screw): screw;
    case Engaged(screw): screw;
  };
  function jointOf():Null<nksim_joint> return switch carriage {
    case Welded(_, joint): joint;
    case Threaded(_, joint): joint;
    case _: null;
  };
  function threadedScrew():Null<SimObject> return switch carriage { case Threaded(screw, _): screw; case _: null; };
  function suctionScrew():Null<SimObject> return switch carriage { case Suction(screw): screw; case _: null; };
  function seatedScrew():Null<SimObject> return switch carriage { case Seated(screw): screw; case _: null; };
  function engagedScrew():Null<SimObject> return switch carriage { case Engaged(screw): screw; case _: null; };
  /** Places the anchored screw on its threaded member at the process model's turned pose. */
  function engage(screw:SimObject, fixture:SimulationObjectFixture, placed:SimPose):Void {
    if (suctionScrew() != null) holding.release();
    carriage = Engaged(screw);
    fixture.retain(screw, placed);
  }
  /** Removes runtime screw joints before the session restores authored object poses. */
  public function beforeReset():Void {
    var joint = jointOf();
    if (joint == null) return;
    simulation.stopForTopologyChange();
    simulation.destroyObjectJoint(joint);
    carriage = Empty;
  }
  /** Whether the qualified screw is in the socket on a physical thread: running down or seated on its member. */
  public function physicalThreadAttached():Bool return threadedScrew() != null || seatedScrew() != null;
  /** Whether the screw is rigidly jointed to the bit for an arm-owned search. */
  public function welded():Bool return switch carriage { case Welded(_, _): true; case _: false; };
  /**
   * Runs before the next physics step. While the arm owns axial motion the held screw is a rigid physical
   * child of the bit, so its contacts load the feed spring and the arm; once the process model takes over it
   * is held by vacuum again at its measured pose. A physical thread engages after the mission's approach.
   */
  public function prepareCarriage():Void {
    var setup = selected;
    var axialOwner:Null<Void->Bool> = setup == null ? null : cast setup.armOwnsAxialMotion;
    var armOwnsAxialMotion = axialOwner != null && axialOwner();
    var held = suctionScrew();
    // Once the thread start holds the screw (anchored), the bit no longer carries it.
    if (held != null && armOwnsAxialMotion && setup != null && held == setup.screw && !reversed && anchoredBody == null)
      weld(held);
    else if (welded() && !armOwnsAxialMotion) unjoin(true);
    // An arm-owned search keeps the screw on the bit until it hands the seat over.
    if (setup == null || setup.physicalThread != true || threadedScrew() != null || seatedScrew() != null ||
        reversed || armOwnsAxialMotion || setup.screw == null || carried() != setup.screw) return;
    var state = setup.work();
    if (state == null || state.hole == null || state.hole.length != 3 || state.direction == null ||
        state.direction.length != 3 || state.head == null || state.screwPose == null ||
        state.engagementPoint == null || state.engagementPoint.length != 3)
      throw "Physical thread engagement lost its qualified CAD work frame";
    var referenceHead = transform(state.head);
    var actualBody = transform(simulation.objectPose(setup.screw));
    var bodyToHead = transform(state.screwPose).inverse().compose(referenceHead);
    var actualHead = actualBody.compose(bodyToHead);
    var leadInHead = referenceHead.inverse().transformPoint(Vec3.fromArray(state.engagementPoint));
    var lead = actualHead.transformPoint(leadInHead);
    var holeAxis = Vec3.fromArray(state.direction).normalized();
    var leadError = lead.sub(Vec3.fromArray(state.hole));
    var along = leadError.dot(holeAxis);
    var radial = Math.sqrt(Math.max(0,leadError.dot(leadError)-along*along))*1000;
    var link = simulation.linkPose(robotIndex,linkIndex);
    var linkFrame = new Transform3(Vec3.fromArray(link.position),
      new Quat(link.rotation[0],link.rotation[1],link.rotation[2],link.rotation[3]));
    var toolAxis = linkFrame.transformVector(Vec3.fromArray(axis)).normalized();
    var headAxis = actualHead.transformVector(new Vec3(0,0,1)).normalized();
    var pickAlignment = Math.acos(Math.max(-1,Math.min(1,headAxis.dot(toolAxis))));
    var screwAlignment = Math.acos(Math.max(-1,Math.min(1,headAxis.dot(holeAxis))));
    var alignment = Math.max(pickAlignment,screwAlignment);
    // The thread engages where the process model's would: the screw's lead-in point within the qualified lead-in of
    // the hole's entry, on axis. Earlier, the screw is still carried by the bit.
    if (radial <= setup.joint.leadInMm && Math.abs(along)*1000 <= setup.joint.leadInMm &&
        alignment <= setup.joint.maximumAxisAngle)
      attachPhysicalThread(setup,referenceHead,actualBody,lead,holeAxis);
  }
  /** Holds the identified screw by vacuum at `worldPose`. */
  function grip(screw:SimObject, worldPose:SimPose):Void {
    if (carried() != null) throw "Screwdriver suction needs an empty bit";
    holding.attach(screw, worldPose);
    carriage = Suction(screw);
  }
  /** Replaces the vacuum hold with a rigid joint at the measured pose, between ticks. */
  function weld(screw:SimObject):Void {
    simulation.stopForTopologyChange();
    var pose = simulation.objectPose(screw);
    holding.release();
    carriage = Empty;
    try {
      carriage = Welded(screw, simulation.createObjectFixedFollower(robotIndex, linkIndex, screw));
    } catch (error:Dynamic) {
      grip(screw, pose);
      throw error;
    }
  }
  /** Removes a welded or threaded joint at the live pose, optionally regripping by vacuum. */
  function unjoin(regrip:Bool):SimPose {
    var screw = carried(), joint = jointOf();
    if (screw == null || joint == null) throw "Screwdriver has no jointed screw to release";
    simulation.stopForTopologyChange();
    var currentPose = simulation.objectPose(screw);
    simulation.destroyObjectJoint(joint);
    carriage = Empty;
    if (regrip) grip(screw, currentPose);
    return currentPose;
  }
  /** Lets the screw go by whichever path holds it, returning its live pose. */
  function letGo():Null<SimPose> {
    var engaged = engagedScrew();
    if (engaged != null) {
      // Unfastened, the thread start lets it go free where it stands; a fastened release retains it again.
      var pose = simulation.objectPose(engaged);
      simulation.releaseObject(engaged);
      carriage = Empty;
      return pose;
    }
    var seated = seatedScrew();
    if (seated != null) {
      // Already retained by its member: the bit lifts out of the socket.
      carriage = Empty;
      return simulation.objectPose(seated);
    }
    var screw = suctionScrew();
    if (screw == null) return jointOf() == null ? null : unjoin(false);
    var pose = simulation.objectPose(screw);
    holding.release();
    carriage = Empty;
    return pose;
  }
  /**
   * Threads the screw into its member: a helical joint along the hole axis through the screw's own lead-in point,
   * turned by the spindle through the socket. The thread's axial reaction stays between the screw and the member.
   */
  function attachPhysicalThread(setup:ScrewdriverSetup, referenceHead:Transform3, actualBody:Transform3,
      leadPoint:Vec3, holeAxis:Vec3):Void {
    var screw = setup.screw, fixture = setup.fixture;
    if (threadedScrew() != null || screw == null || fixture == null || carried() != screw || setup.physicalThread != true)
      throw "Physical thread attachment needs the qualified screw held at the screwdriver over its fixture";
    var leadMm = requiredPositive(setup.threadLeadMm,"Physical thread lost its CAD lead");
    var stiffness = requiredPositive(setup.physicalThreadStiffnessNPerM,"Physical thread lost its stiffness");
    var settings = setup.program.settings;
    // Every turn the program may take, either way, with a turn to spare.
    var span = settings.maximumRunDownAngle + settings.maximumAngle + 2 * Math.PI;
    var lead = setup.hand * leadMm / 1000.0;
    if (welded()) unjoin(true);
    var member = fixture.threadedBody();
    simulation.stopForTopologyChange();
    holding.release();
    carriage = Empty;
    try {
      carriage = Threaded(screw, simulation.createObjectScrewJoint(robotIndex, spindleJoint, member, screw,
        leadPoint.toArray(), holeAxis.toArray(), lead, -span, span, spindleAxisAlignment < 0 ? -1 : 1,
        stiffness, 1.0, threadArmature(lead / (2 * Math.PI))));
      anchoredBody = referenceHead.inverse().compose(actualBody);
    } catch (error:Dynamic) {
      grip(screw, pose(actualBody));
      throw error;
    }
  }
  /** The run-down torque the spindle may apply through a physical thread: the program's limit, and no more than
   * the commissioned axial force cap through the lead. */
  function threadTorqueLimit(setup:ScrewdriverSetup):Float {
    var leadM = requiredPositive(setup.threadLeadMm,"Physical thread lost its CAD lead") / 1000.0;
    var cap = requiredPositive(setup.physicalThreadMaxForceN,"Physical thread lost its axial force cap");
    return Math.min(setup.program.settings.runDownTorqueLimit, cap * leadM / (2 * Math.PI));
  }
  /** At the model's seat the screw leaves the thread for its member, retained at its measured seated pose. */
  function seat(setup:ScrewdriverSetup):Void {
    var screw = threadedScrew(), fixture = setup.fixture;
    if (screw == null || fixture == null) throw "Seating a physical thread lost its screw or fixture";
    var seated = unjoin(false);
    fixture.retain(screw, seated);
    if (!fixture.carries(screw)) throw "Seating a physical thread did not retain the screw on its member";
    carriage = Seated(screw);
  }
  /**
   * While the arm owns axial motion the feed slide is what it is mechanically: a spring of the commissioned rate,
   * resting extended and critically damped, which takes up contact the arm's force loop has not yet answered.
   */
  function springFeed():Void {
    var stiffness = springRate * 1000.0;
    simulation.springProcessJoint(robotIndex, feedJoint, 0, stiffness, 2.0 * Math.sqrt(stiffness * feedMass));
  }

  /**
   * Inertia on the thread's advance. A rigid thread reflects the spindle's turning inertia onto the screw axis as
   * `inertia / ratio^2` (`ratio` in metres per radian); a tenth of it lets the step resolve a stiff thread on a light
   * screw while adding only a tenth to what the spindle drive accelerates.
   */
  function threadArmature(ratio:Float):Float return 0.1 * spindleInertia / (ratio * ratio);

  static function requiredPositive(value:Null<Float>, diagnostic:String):Float {
    if (value == null) throw diagnostic;
    var checked:Float = cast value;
    if (!Math.isFinite(checked) || checked <= 0) throw diagnostic;
    return checked;
  }
  public function afterSimulationStep(timestamp:Int64):Void {
    var previous = lastTime; lastTime = timestamp;
    var dt = previous == null ? 0.0 : Math.max(0, Int64.toFloat(Int64.sub(timestamp, previous)) * 1e-9);
    var controls = supply == null ? null : supply.controls();
    var joints = supply == null ? null : supply.joints();
    var start = controls == null ? digital(channels.start) : controls.start;
    var vacuum = controls == null ? digital(channels.vacuum) : controls.vacuum;
    var reverse = controls == null ? digital(channels.direction) : controls.reverse;
    var slotValue = controls == null ? analog(channels.program) : controls.program;
    var slot = ScrewdriverProcessDevice.programSlot(slotValue, true);
    if (start && lastStart && slot != activeSlot) throw "Screwdriver program cannot change during a cycle";
    // A successful cycle can release vacuum and select the next fastening in
    // the same sampled controller image. Resolve the old thread before the
    // slot guard below, while its qualified process and fixture are still live.
    var threadedBody = threadedScrew();
    if (threadedBody == null) threadedBody = seatedScrew();
    if (!vacuum && threadedBody != null) {
      var oldSetup = selected, oldProcess = model, threaded:SimObject = threadedBody;
      if (oldSetup == null || oldProcess == null)
        throw "Physical thread release lost its qualified screw setup";
      var releasedPose = letGo();
      var fastened = oldProcess.reading.result == ScrewdriverContract.OK;
      if (fastened && oldSetup.fixture != null) {
        var fixture = oldSetup.fixture, anchor = anchoredBody, state = oldSetup.work();
        var connection = oldSetup.connection;
        if (releasedPose == null || anchor == null || state == null || connection == null)
          throw "Successful physical thread release lost its qualified engagement";
        fixture.retain(threaded,releasedPose);
        if (!fixture.carries(threaded)) throw "Successful physical thread release did not retain its screw on the named fixture";
        var head = transform(state.head);
        retained.push({fastening:connection.fastening,member:connection.member,face:connection.face,screw:threaded,
          hand:oldSetup.hand,model:oldProcess,anchor:anchor,bodyToHead:transform(state.screwPose).inverse().compose(head),
          leadInHead:head.inverse().transformPoint(Vec3.fromArray(state.engagementPoint)),fixture:fixture});
      }
      anchoredBody = null;
    }
    if (slot != activeSlot || (start && selected == null)) {
      var setup = setups.get(slot);
      if (setup == null && start) throw 'Screwdriver slot $slot has no qualified joint program';
      if ((threadedScrew() != null || seatedScrew() != null) && (setup == null || carried() != setup.screw))
        throw "Cannot select another fastening while a screw is threaded in the socket";
      anchoredBody = null; selected = setup; activeSlot = slot; model = setup == null ? null : new TighteningModel(setup.program, setup.joint); angle = 0; reversed = false; reverseTargetAngle = 0; origin = joints == null ? runtime.snapshot().q.get(spindleJoint) : joints.spindlePosition;
      pendingEngagement = null;
      if (setup != null) {
        if (setup.retightening == true) {
          var engagement = retainedFor(setup);
          var continued = engagement.model.retighten(setup.program,setup.joint);
          model = continued;
          angle = continued.reading.angle; origin -= setup.hand*angle;
          pendingEngagement = engagement;
        }
        // Once selected, an attempt owns this body's engagement. A failed
        // continuation cannot later resurrect the preceding successful state.
        var old = [for (item in retained) if (item.screw == setup.screw) item];
        for (item in old) retained.remove(item);
      }
    }
    if (start && reverse && model == null) throw "Cannot reverse without a configured screw cycle";
    lastStart = start;
    var setup = selected, process = model;
    var state = setup == null ? null : setup.work();
    var axialOwner:Null<Void->Bool> = setup == null ? null : cast setup.armOwnsAxialMotion;
    var armOwnsAxialMotion = axialOwner != null && axialOwner();
    if (state != null && (state.hole == null || state.hole.length != 3 || state.direction == null || state.direction.length != 3 ||
        state.head == null || state.screwPose == null || state.engagementPoint == null || state.engagementPoint.length != 3))
      throw "Screwdriver live work needs a head and hole axis";
    if (start && !reverse && reversed) throw "Screwdriver needs a newly configured cycle after reversal";
    if (setup != null && process != null && threadedScrew() != null && reversed && !start && !reverse && vacuum) {
      var shaft = joints == null ? runtime.snapshot().q.get(spindleJoint) : joints.spindlePosition;
      angle = Math.max(0, setup.hand * (shaft - origin));
      if (angle < 1e-6) angle = 0;
      if (angle == 0) unjoin(true);
    }
    if (!vacuum) {
      var released = carried();
      var fixture = setup == null ? null : setup.fixture;
      var fastened = process != null && process.reading.result == ScrewdriverContract.OK && anchoredBody != null;
      var releasedPose = letGo();
      if (fastened && fixture != null) {
        if (released == null || releasedPose == null) throw "Successful release lost the identified screw body or its live pose";
        fixture.retain(released,releasedPose);
        if (!fixture.carries(released)) throw "Successful release did not retain the screw on its named fixture";
        if (setup == null) throw "Successful release lost its qualified setup";
        var connection = setup.connection, anchor = anchoredBody;
        if (connection == null || anchor == null || process == null || state == null) throw "Successful release lost its qualified engagement";
        var head = transform(state.head);
        retained.push({fastening:connection.fastening,member:connection.member,face:connection.face,screw:released,
          hand:setup.hand,model:process,anchor:anchor,bodyToHead:transform(state.screwPose).inverse().compose(head),
          leadInHead:head.inverse().transformPoint(Vec3.fromArray(state.engagementPoint)),fixture:fixture});
      }
      anchoredBody = null;
    }
    if (setup != null && process != null) {
      if (state == null) throw "Screwdriver lost its sampled live work";
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
      var feed = joints == null ? q.get(feedJoint) : joints.feedPosition;
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
      if (vacuum && screw != null && carried() == null &&
          pickError.norm() <= SimulatedSuctionTool.REACH &&
          pickAlignment <= setup.joint.maximumAxisAngle) grip(screw, pose(actualBody));
      var lead = actualHead.transformPoint(leadInHead);
      var leadError = lead.sub(Vec3.fromArray(state.hole));
      var along = leadError.dot(Vec3.fromArray(holeAxis));
      var radial = Math.sqrt(Math.max(0,leadError.dot(leadError)-along*along))*1000;
      var screwAlignment = Math.acos(Math.max(-1,Math.min(1,headAxis.x*holeAxis[0]+headAxis.y*holeAxis[1]+headAxis.z*holeAxis[2])));
      var alignment = Math.max(pickAlignment,screwAlignment);
      var linkFrame = new Transform3(Vec3.fromArray(link.position), new Quat(link.rotation[0],link.rotation[1],link.rotation[2],link.rotation[3]));
      var anchor = anchoredBody;
      var pending = pendingEngagement;
      if (anchor == null && pending != null && suctionScrew() != null) {
        if (suctionScrew() != pending.screw || bodyToHead.translation.sub(pending.bodyToHead.translation).norm() > 1e-9 ||
            Math.abs(quaternionDot(bodyToHead.rotation,pending.bodyToHead.rotation)) < 1-1e-12 ||
            leadInHead.sub(pending.leadInHead).norm() > 1e-9)
          throw 'Fastening "${pending.fastening}" on "${pending.member}" face "${pending.face}": final recontact changed its retained socket or thread-entry geometry';
        var initial = referenceHead.compose(pending.anchor);
        var expected = transform(turnedPose(initial,initial.compose(bodyToHead).translation,holeAxis,
          setup.hand*angle,process.advanceAtAngle(angle)/1000));
        if (actualBody.translation.sub(expected.translation).norm() > 1e-6 ||
            Math.abs(quaternionDot(actualBody.rotation,expected.rotation)) < 1-1e-8)
          throw 'Fastening "${pending.fastening}" on "${pending.member}" face "${pending.face}": final recontact no longer describes the actual retained seated screw';
        anchor = pending.anchor; anchoredBody = anchor; pendingEngagement = null;
      }
      // Thread teeth are not collision geometry, so the thread start is this anchor: a screw reaching the entry
      // within the qualified lead-in stops there until it is turned, also under an arm-owned search.
      if (anchor == null && setup.physicalThread != true && setup.retightening != true &&
          (suctionScrew() != null || welded()) && !reversed &&
          radial <= setup.joint.leadInMm && Math.abs(along)*1000 <= setup.joint.leadInMm &&
          alignment <= setup.joint.maximumAxisAngle) {
        if (welded()) unjoin(true);
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
      var stroke = armOwnsAxialMotion || anchor == null ? 0.0 : Math.max(0,-dot(toHead,direction)*1000);
      if (anchor != null && reversed && angle == 0 && !start && dot(toHead,direction) > SimulatedSuctionTool.REACH) {
        anchoredBody = null; anchor = null; stroke = 0;
        // Backed out to the thread start and lifted: the bit's vacuum carries it again.
        var engaged = engagedScrew();
        if (engaged != null) { var live = simulation.objectPose(engaged); carriage = Empty; grip(engaged, live); }
      }
      if (reverse && start) {
        if (seatedScrew() != null) throw "Backing out a physically seated screw is not modelled; release it first";
        if (!reversed) { reversed = true; reverseTargetAngle = angle; }
        reverseTargetAngle = Math.max(0, reverseTargetAngle - setup.program.settings.runDownSpeed * dt);
        angle = Math.max(0, setup.hand*((joints == null ? q.get(spindleJoint) : joints.spindlePosition)-origin));
        // Numerical zero for a settled spindle, while retaining measured reversal feedback.
        if (angle < 1e-6) angle = 0;
        observed.torqueNm = 0; observed.angle = angle; observed.speed = -setup.program.settings.runDownSpeed;
        if (angle == 0) observed.speed = 0;
        driveSpeed = observed.speed;
        observed.strokeMm = stroke; observed.vacuum = carried() != null;
      } else if (!reversed) {
        if (start) angle = Math.max(angle, setup.hand*((joints == null ? q.get(spindleJoint) : joints.spindlePosition)-origin));
        var screwRetained = setup.screw != null && carried() == setup.screw;
        observed = process.step({screwPresent:setup.screw != null && screwRetained, vacuum:vacuum && screwRetained,
          start:start, angle:angle, strokeMm:stroke, radialErrorMm:radial, axialErrorMm:along*1000, axisAngle:alignment, damagedLeadIn:state.damagedLeadIn});
        if (observed.result != ScrewdriverContract.NONE) angle = observed.angle;
        driveSpeed = observed.speed;
        var seatedPhase = observed.phase == ScrewdriverContract.SNUG || observed.phase == ScrewdriverContract.FINAL ||
          observed.result == ScrewdriverContract.OK;
        if (threadedScrew() != null && seatedPhase) seat(setup);
        // A head bearing on its seat sits flat: the clamp turns it onto the hole axis about the head, keeping
        // its lateral offset. The aligned pose becomes the anchor a later pass continues from.
        if (seatedPhase && anchoredBody != null && setup.physicalThread != true) {
          var aligned = seatedOnAxis(startingBody, bodyToHead, Vec3.fromArray(holeAxis));
          if (aligned != null) {
            startingBody = aligned;
            anchoredBody = referenceHead.inverse().compose(aligned);
          }
        }
      }
      if (!start) { driveSpeed = 0; if (reversed) reverseTargetAngle = angle; }
      if (armOwnsAxialMotion) springFeed();
      else simulation.driveProcessJoint(robotIndex, feedJoint, -Math.min(strokeLimitMm,stroke)/1000, 0);
      var targetAngle = start && reverse ? reverseTargetAngle : angle + driveSpeed*dt;
      simulation.driveProcessJoint(robotIndex, spindleJoint, origin + setup.hand*targetAngle,
        setup.hand*driveSpeed, threadedScrew() != null ? threadTorqueLimit(setup) : 0);
      observed.speed = setup.hand*(joints == null ? measured.dq.get(spindleJoint) : joints.spindleSpeed);
      var anchoredScrew = suctionScrew() != null ? suctionScrew() : engagedScrew();
      if (anchoredScrew != null && anchor != null) {
        var advance = process.advanceAtAngle(angle)/1000;
        var placed = turnedPose(startingBody,head,holeAxis,setup.hand*angle,advance);
        var fixture = setup.fixture;
        if (fixture != null) engage(anchoredScrew, fixture, placed);
        else holding.placeHeld(placed);
      }
    } else {
      observed = ScrewdriverSensor.idle();
      var measured = runtime.snapshot();
      observed.strokeMm = Math.max(0,-(joints == null ? measured.q.get(feedJoint) : joints.feedPosition)*1000);
      observed.speed = joints == null ? measured.dq.get(spindleJoint) : joints.spindleSpeed;
      observed.vacuum = carried() != null;
      simulation.driveProcessJoint(robotIndex,feedJoint,0,0);
      simulation.driveProcessJoint(robotIndex,spindleJoint,joints == null ? measured.q.get(spindleJoint) : joints.spindlePosition,0);
    }
    // The process result retains its final angle. Live feedback follows the
    // measured shaft through stopping and reversal, including physical coast.
    var measuredAngle = selected == null ? observed.angle : Math.max(0,selected.hand*((joints == null ? runtime.snapshot().q.get(spindleJoint) : joints.spindlePosition)-origin));
    observed = {torqueNm:observed.torqueNm,angle:measuredAngle,speed:observed.speed,
      strokeMm:observed.strokeMm,phase:observed.phase,result:observed.result,
      vacuum:carried() != null};
    if (supply != null) supply.publish(observed);
    else runtime.publishSensorFrame(sensorId, ScrewdriverSensor.values(observed), Int64.ofInt(++sequence),
      runtime.snapshot().sourceTimestampNs, runtime.endpoint.sourceClockId());
  }
  static function dot(a:Array<Float>,b:Array<Float>):Float return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
  /** `body` turned about its head point so the head's +Z lies on `holeAxis`; null when it already does. */
  static function seatedOnAxis(body:Transform3, bodyToHead:Transform3, holeAxis:Vec3):Null<Transform3> {
    var head = body.compose(bodyToHead);
    var axis = head.transformVector(new Vec3(0,0,1)).normalized(), target = holeAxis.normalized();
    var cross = axis.cross(target), sine = cross.norm(), cosine = axis.dot(target);
    if (sine < 1e-12) return null;
    var turn = Quat.fromAxisAngle(cross.scale(1/sine), Math.atan2(sine, cosine));
    var offset = turn.rotate(body.translation.sub(head.translation));
    return new Transform3(head.translation.add(offset), turn.multiply(body.rotation));
  }
  static function quaternionDot(a:Quat,b:Quat):Float return a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w;
  static function turnedPose(startingBody:Transform3,head:Vec3,holeAxis:Array<Float>,angle:Float,advance:Float):SimPose {
    var rotation = Quat.fromAxisAngle(new Vec3(holeAxis[0],holeAxis[1],holeAxis[2]),angle);
    var turned = rotation.multiply(startingBody.rotation);
    var offset = rotation.rotate(new Vec3(startingBody.translation.x-head.x,
      startingBody.translation.y-head.y,startingBody.translation.z-head.z));
    return new SimPose(head.x+offset.x+holeAxis[0]*advance,head.y+offset.y+holeAxis[1]*advance,
      head.z+offset.z+holeAxis[2]*advance,turned.x,turned.y,turned.z,turned.w);
  }
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
