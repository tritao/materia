package app;

import haxe.Int64;
import robotkit.runtime.*;
import robotkit.model.*;
import robotkit.spatial.*;
import robotkit.tool.ScrewdriverContract;
import robotkit.execution.ProcessEventValue;
import processkit.screwdriving.*;
import processkit.screwdriving.TighteningModel.TighteningJoint;
import processkit.screwdriving.ScrewdriverProcessDevice.ScrewdriverChannelNames;
import processkit.simulation.SimulatedScrewdriver;
import nativekit.sim.SimPose;

/** Process-owned tool motion, live frames, holding, feedback and stop/resume on both backends. */
class ScrewdriverSimulationTests {
  static var count = 0;
  static function check(value:Bool,message:String):Void { count++; if (!value) throw message; }
  static function refuses(run:Void -> Void):Void {
    var refused = false; try run() catch (_:Dynamic) refused = true;
    check(refused,"Invalid driver operation must be refused");
  }
  static function program():TighteningProgram return new TighteningProgram({runDownSpeed:20,finalSpeed:2,
    targetTorque:9.9,minimumAngle:0.4,maximumAngle:0.6,runDownTorqueLimit:1,finalTorqueLimit:12,
    maximumRunDownAngle:30,minimumStroke:0.5,maximumStroke:15});
  static function joint():TighteningJoint return {leadMm:1,runDownMm:2,leadInMm:0.5,maximumAxisAngle:0.1,
    springRate:2,prevailingTorque:0.2,stiffnessNmPerRad:19.4,stripTorque:20};
  public static function run():Void {
    count = 0;
    for (backend in [SimulationSpace.DETERMINISTIC,SimulationSpace.MUJOCO]) {
      runBackend(backend);
      runTransport(backend);
    }
    runSequencing();
    var device = new DriverDeviceFixture();
    var channels:ScrewdriverChannelNames = {start:"start",direction:"reverse",program:"program",vacuum:"vacuum"};
    var process = new ScrewdriverProcessDevice(device,device,channels,1);
    process.prepare(); check(process.ready() && device.slot == 1 && !device.running && !device.reversing,"Prepared device selects its slot and stops the spindle");
    function event(channel:String,value:robotkit.execution.ProcessEventValue):robotkit.execution.FiredProcessEvent
      return new robotkit.execution.FiredProcessEvent(Int64.ofInt(1),channel,value,Int64.ofInt(0),Int64.ofInt(0),0);
    process.apply([event("program",Analog(2)),event("start",Digital(true)),event("reverse",Digital(true)),event("vacuum",Digital(true))]);
    check(device.slot == 2 && device.running && device.reversing && device.held,"Typed controller events drive each declared output");
    refuses(() -> process.apply([event("program",Analog(1.5))]));
    refuses(() -> process.apply([event("program",Analog(1e12))]));
    refuses(() -> process.apply([event("start",Analog(1))]));
    refuses(() -> process.apply([event("unknown",Digital(true))]));
    device.held = true; process.safe();
    check(!process.ready() && device.held && !device.running,"Device safe retains vacuum on a commanded stop");
    device.feedback.result = ScrewdriverContract.CROSS_THREAD;
    check(process.fault() == "screwdriver: cross-thread","Hardware and simulation share result diagnostics");
    Sys.println('Screwdriver simulation: $count checks passed on MuJoCo and the test backend');
  }
  static function runSequencing():Void {
    var device = new DriverDeviceFixture();
    var motion = new DriverMotionFixture();
    var runner = new ScrewdrivingRunner(motion,device,() -> true);
    var pickup = new Transform3(new Vec3(0.1,0,0.2),Quat.identity());
    var entry = new Transform3(new Vec3(0.2,0,0.2),Quat.identity());
    var plan = new ScrewPlan("probe-fastening",1,program(),pickup,entry,1,2);
    runner.run(plan);
    for (_ in 0...20) if (!device.running) {
      device.feedback.vacuum = device.held;
      runner.update(0.01,device.reading());
    }
    check(device.running && device.held && motion.targets.length == 5,
      "The runner picks, verifies vacuum, transfers and engages before starting the spindle");
    device.feedback.strokeMm = plan.strokeMm-1;
    runner.update(0.01,device.reading());
    var followed = motion.targets[motion.targets.length-1];
    check(Math.abs(followed.translation.z-(entry.translation.z+(plan.strokeMm+1)/1000)) < 1e-9,
      "Arm following corrects measured compression without deriving advance from spindle angle");
    device.feedback.result = ScrewdriverContract.OK;
    device.feedback.torqueNm = 9.9; device.feedback.angle = 4*Math.PI+0.5; device.feedback.speed = 2;
    runner.update(0.01,device.reading());
    runner.update(0.01,device.reading());
    check(!device.running && device.held && !runner.completed(),
      "A result stops the spindle but retains vacuum until measured rotation stops");
    device.feedback.speed = 0; runner.update(0.01,device.reading());
    check(!device.held && !runner.completed(),"Stopped tightening releases vacuum before retracting");
    device.feedback.vacuum = false;
    runner.update(0.01,device.reading()); runner.update(0.01,device.reading());
    var result = runner.tighteningResult;
    check(runner.completed() && result != null && result.fastening == "probe-fastening" && result.torqueNm == 9.9 &&
      result.angle == 4*Math.PI+0.5 && result.outcome == ScrewdriverContract.OK,
      "The result preserves measured torque and cycle angle under the CAD fastening's identity");
    var absent = new ScrewdrivingRunner(new DriverMotionFixture(),device,() -> false);
    absent.run(plan);
    for (_ in 0...10) if (absent.running()) absent.update(0.01,ScrewdriverSensor.idle());
    var missing = absent.tighteningResult;
    check(absent.failure() != null && missing != null && missing.outcome == ScrewdriverContract.NO_SCREW && !device.running,
      "An empty presenter reports no screw before the spindle starts");
    refuses(() -> { var invalid = new ScrewdrivingArmMotion(null); });
    refuses(() -> { var invalid = new processkit.skill.DriveScrew(null,null,null,""); });
  }
  /** A distant lead-in cannot pull a screw out of the presenter or compress its feed. */
  static function runTransport(backend:Int):Void {
    var harness = new SimulationHarness(0.01,1,backend);
    try {
      var model = new RobotModel("screw transport probe");
      var base = model.addLink(new Link("base")), arm = model.addLink(new Link("arm"));
      var slide = model.addLink(new Link("slide")), carrier = model.addLink(new Link("carrier"));
      // Distinct probe bodies keep self-collision enabled without overlapping
      // the default 100 mm fallback boxes at every joint origin.
      var bodies = [base,arm,slide,carrier];
      for (i in 0...bodies.length)
        bodies[i].collisionShapes.push(new CollisionShape(Box(0.001,0.001,0.001),[0,(i-2)*0.01,0]));
      var travel = model.addJoint(new Joint("travel",JointType.Prismatic,base,arm));
      travel.axis = [1,0,0]; travel.limits = new JointLimits(0,0.04,0.1,100);
      var feed = model.addJoint(new Joint("feed",JointType.Prismatic,arm,slide));
      feed.axis = [0,0,1]; feed.limits = new JointLimits(-0.02,0,0.1,40);
      var spindle = model.addJoint(new Joint("spindle",JointType.Continuous,slide,carrier));
      spindle.axis = [0,0,1]; spindle.limits = new JointLimits(-1000,1000,60,20);
      var sensor = model.addSensor(new Sensor("driver","tool_screwdriver",0));
      var tipFrame = model.addFrame(new Frame("tip",carrier));
      tipFrame.position = [0,0,0.103];
      sensor.frame = tipFrame;
      var blueprint = RobotRuntimeCompiler.compile(model,new robotkit.profile.RobotProfile());
      blueprint.addTool(new robotkit.tool.ScrewdriverChannels("start","reverse","program","vacuum"));
      blueprint.processModelJoints.push(1); blueprint.processModelJoints.push(2);
      var runtime = harness.simulation.addRobot(blueprint);
      // Support the presented body physically; an unsupported body falls before vacuum seals.
      harness.spawnBox([0,0,0.0959],[0.005,0.005,0.005]);
      var planningArm = ScrewdriverArmKinematics.create(model,base.id,tipFrame.id,feed.id,spindle.id);
      check(planningArm.jointIndices().length == 1 && planningArm.jointIndices()[0] == 0 &&
        Math.abs(planningArm.tcpPose([0.01]).translation.x-0.01) < 1e-9 &&
        Math.abs(planningArm.tcpPose([0.01]).translation.z-0.103) < 1e-9,
        "Arm IK exposes the uncompressed bit while excluding the process-owned feed and spindle");
      var screw = harness.spawnBox([0,0,0.101],[0.0001,0.0001,0.0001],true,0.01);
      var channels:ScrewdriverChannelNames = {start:"start",direction:"reverse",program:"program",vacuum:"vacuum"};
      var observer = new SimulatedScrewdriver(harness.simulation,runtime,0,3,1,2,[0,0,0.103],[0,0,1],channels,"driver",20,60,20,2);
      var head = new SimPose(0.02,0,0.1,0,0,0,1);
      var body = new SimPose(0.02,0,0.101,0,0,0,1);
      observer.configure(1,{program:program(),joint:joint(),screw:screw,hand:1,
        work:() -> ({head:head,screwPose:body,engagementPoint:[0.02,0.0,0.1],hole:[0.02,0.0,0.1],direction:[0.0,0.0,1.0],damagedLeadIn:false})});
      var id = harness.simulation.addStepObserver(observer);
      var outputs = new ChannelScrewdriverOutputs(runtime,channels);
      outputs.selectProgram(1);
      // Let gravity and the physical nest settle before the pickup command.
      for (_ in 0...100) harness.step();
      outputs.vacuum(true); harness.step();
      var pickupBody = harness.simulation.objectPose(screw);
      check(observer.holding.held == screw,"Pickup uses the actual identified socket, away from the work frame");
      check(Math.abs(harness.simulation.objectPose(screw).x) < 1e-5 && observer.reading().strokeMm == 0,
        "Pickup leaves the screw in the nest and the compliant feed uncompressed");
      refuses(() -> harness.simulation.driveProcessJoint(0,1,1e-15,0));
      runtime.submitPosition(0,0.01,1);
      for (_ in 0...1000) if (Math.abs(runtime.snapshot().q.get(0)-0.01) > 1e-5) harness.step();
      // Native holding consumes the carrier pose on the following physics tick.
      harness.step();
      var carried = harness.simulation.objectPose(screw);
      check(Math.abs(carried.x-0.01) < 0.0001 && Math.abs(carried.z-pickupBody.z) < 0.00001 && observer.reading().strokeMm == 0,
        'The screw follows the physical bit halfway to the hole without being pinned to the work (backend $backend, body ${haxe.Json.stringify(carried)}, reading ${haxe.Json.stringify(observer.reading())}, q ${haxe.Json.stringify(runtime.snapshot().q.toArray())}, safety ${runtime.snapshot().safety}, fault ${runtime.snapshot().faultCode}, program ${Std.string(runtime.channelValue("program"))}, vacuum ${Std.string(runtime.channelValue("vacuum"))})');
      runtime.submitPosition(0,0.02,2);
      for (_ in 0...1000) if (Math.abs(runtime.snapshot().q.get(0)-0.02) > 1e-5) harness.step();
      harness.step();
      var actualCompression = (0.104-pickupBody.z)*1000;
      check(Math.abs(observer.reading().strokeMm-actualCompression) < 0.01,
        "The slide compresses only after the carried screw reaches the lead-in");
      outputs.start(true);
      for (_ in 0...5000) if (observer.reading().result == ScrewdriverContract.NONE) harness.step();
      check(observer.reading().result == ScrewdriverContract.OK,
        "A screw carried from the presenter tightens successfully on the real tool joints");
      var seated = harness.simulation.objectPose(screw);
      check(Math.abs(seated.z-0.103) < 0.0001 && Math.abs(seated.x-0.02) <= 0.00051,
        "The transported screw reaches its seat while retaining its actual lead-in contact offset");
      outputs.start(false); harness.step(); outputs.vacuum(false); harness.step();
      runtime.submitPosition(0,0,3);
      for (_ in 0...1000) if (runtime.snapshot().q.get(0) > 1e-5) harness.step();
      check(observer.holding.held == null && Math.abs(harness.simulation.objectPose(screw).x-seated.x) < 0.0001,
        "Vacuum release leaves the tightened screw at the work as the bit retracts");
      var adapter = new RuntimeRobotAdapter("screw-sensor-probe",runtime,"Screw sensor probe",
        [for (link in model.links) Std.string(link.id)],[for (joint in model.joints) Std.string(joint.id)]);
      var state = adapter.snapshot();
      var localization = new robotkit.localization.SimulationTruthLocalization(harness.simulation,0);
      var located = localization.update(state);
      var checked = new ScrewdrivingRunner(new DriverMotionFixture(),new DriverDeviceFixture(),() -> true);
      var samplePlan = new ScrewPlan("sensor-freshness",1,program(),Transform3.identity(),Transform3.identity(),1,2);
      var skill = new processkit.skill.DriveScrew(checked,localization,() -> samplePlan,"driver");
      skill.start();
      var firstStatus = skill.update(state,0.01);
      var staleStatus = skill.update(state,2.01);
      var staleResult = skill.result();
      check(staleResult != null && staleResult.message.indexOf("fresh frame") >= 0 && !checked.running(),
        "Repeated cached sensor frames time out and stop the screwdriver skill");
      harness.simulation.removeStepObserver(id);
    } catch (error:Dynamic) { harness.dispose(); throw error; }
    harness.dispose();
  }
  static function runBackend(backend:Int):Void {
    var harness = new SimulationHarness(0.01,1,backend);
    try {
      var model = new RobotModel("driver observer probe");
      var base = model.addLink(new Link("base")), arm = model.addLink(new Link("arm"));
      var slide = model.addLink(new Link("slide")), carrier = model.addLink(new Link("carrier"));
      var armJoint = model.addJoint(new Joint("arm",JointType.Revolute,base,arm));
      armJoint.axis = [0,0,1]; armJoint.limits = new JointLimits(-1,1,1,10);
      var feed = model.addJoint(new Joint("feed",JointType.Prismatic,base,slide));
      feed.axis = [0,0,1]; feed.limits = new JointLimits(-0.02,0,0.1,40);
      var spindle = model.addJoint(new Joint("spindle",JointType.Continuous,slide,carrier));
      spindle.axis = [0,0,1]; spindle.limits = new JointLimits(-1000,1000,60,20);
      var sensor = model.addSensor(new Sensor("driver","tool_screwdriver",0));
      var tipFrame = model.addFrame(new Frame("tip",carrier));
      tipFrame.position = [0,0,0.103];
      sensor.frame = tipFrame;
      var blueprint = RobotRuntimeCompiler.compile(model,new robotkit.profile.RobotProfile());
      blueprint.addTool(new robotkit.tool.ScrewdriverChannels("start","reverse","program","vacuum"));
      blueprint.processModelJoints.push(1); blueprint.processModelJoints.push(2);
      var runtime = harness.simulation.addRobot(blueprint);
      // The default link box extends 50 mm from its origin; the probe socket lies beyond it.
      // Support the presented body physically; an unsupported body falls before vacuum seals.
      harness.spawnBox([0,0,0.0959],[0.005,0.005,0.005]);
      var screw = harness.spawnBox([0,0,0.101],[0.0001,0.0001,0.0001],true,0.01);
      var channels:ScrewdriverChannelNames = {start:"start",direction:"reverse",program:"program",vacuum:"vacuum"};
      var observer = new SimulatedScrewdriver(harness.simulation,runtime,0,3,1,2,[0,0,0.103],[0,0,1],channels,"driver",20,60,20,2);
      var head = new SimPose(0,0,0.1,0,0,0,1);
      var bodyPose = new SimPose(0,0,0.101,0,0,0,1);
      observer.configure(1,{program:program(),joint:joint(),screw:screw,hand:1,
        work:() -> ({head:head,screwPose:bodyPose,engagementPoint:[0.0,0.0,0.1],hole:[0.0,0.0,0.1],direction:[0.0,0.0,1.0],damagedLeadIn:false})});
      var observerId = harness.simulation.addStepObserver(observer);
      var outputs = new ChannelScrewdriverOutputs(runtime,channels);
      outputs.selectProgram(1);
      // MuJoCo resolves support contact after integration; seal after the nest settles.
      for (_ in 0...100) harness.step();
      outputs.vacuum(true); harness.step();
      check(observer.holding.held == screw && observer.reading().phase == ScrewdriverContract.IDLE,
        'Vacuum attaches the identified screw before start (backend $backend, reading ${haxe.Json.stringify(observer.reading())}, body ${haxe.Json.stringify(harness.simulation.objectPose(screw))}, link ${haxe.Json.stringify(harness.simulation.linkPose(0,3))})');
      var contacted = harness.simulation.objectPose(screw);
      check(Math.abs(observer.reading().strokeMm-(0.103-contacted.z+0.001)*1000) < 0.001,
        'Stroke comes from free TCP depth against the actual contacted head (backend $backend, stroke ${observer.reading().strokeMm}, body ${haxe.Json.stringify(contacted)}, q ${haxe.Json.stringify(runtime.snapshot().q.toArray())}, link ${haxe.Json.stringify(harness.simulation.linkPose(0,3))})');
      refuses(() -> harness.simulation.driveProcessJoint(0,0,0.1,0));
      runtime.submitPosition(0,0.25,1);
      outputs.start(true);
      for (_ in 0...20) harness.step();
      check(observer.reading().angle > 0 && observer.reading().result == 0,"The spindle advances the real screw angle during run-down");
      check(observer.reading().speed == runtime.snapshot().dq.get(2),"Speed feedback comes from the actual spindle, independently of its program setpoint");
      var paused = observer.reading().angle;
      outputs.start(false);
      harness.step();
      for (_ in 0...5000) if (Math.abs(observer.reading().speed) > 0.01) harness.step();
      check(observer.reading().angle == paused && observer.holding.held == screw && Math.abs(observer.reading().speed) <= 0.01,"Stop retains the screw and pauses process advance while the physical spindle settles");
      outputs.start(true);
      for (_ in 0...5000) if (observer.reading().result == 0) harness.step();
      var result = observer.reading();
      check(result.result == ScrewdriverContract.OK && Math.abs(result.torqueNm-9.9) < 1e-9,"Measured spindle rotation reaches the qualified torque and angle window");
      check(Math.abs(runtime.snapshot().q.get(1)+result.strokeMm/1000) < 0.0001,
        "The physical feed follows the computed compression within 0.1 mm");
      check(Math.abs(runtime.snapshot().q.get(0)-0.25) < 1e-5,"The driver preserves the unrelated arm's independent position command");
      var posed = harness.simulation.objectPose(screw);
      check(Math.abs(posed.z-0.103) < 0.00002,"The held body preserves its offset from the head while advancing to the seat");
      var frames = runtime.snapshot().sensors.toArray();
      var found = false;
      for (frame in frames) if (frame.sensorId == "driver") found = ScrewdriverContract.valid(frame.values.toArray());
      check(found,"The runtime publishes the canonical sensor frame");
      refuses(() -> runtime.publishSensorFrame("driver",[0.0],Int64.ofInt(100000),runtime.snapshot().sourceTimestampNs,runtime.endpoint.sourceClockId()));
      outputs.start(false); harness.step();
      outputs.reverse(true); outputs.start(true);
      for (_ in 0...5000) if (observer.reading().angle > 0) harness.step();
      check(observer.reading().angle == 0 && observer.holding.held == screw,"Reverse backs out the driven angle without releasing the screw");
      outputs.reverse(false); refuses(() -> harness.step());
      outputs.start(false); outputs.reverse(false); outputs.vacuum(false); harness.step();
      check(observer.holding.held == null,"Vacuum off releases the screw");
      observer.reset();
      check(observer.reading().phase == ScrewdriverContract.IDLE && observer.model == null,"Reset clears the process and holding state");
      observer.configure(2,{program:program(),joint:joint(),screw:null,hand:1,
        work:() -> ({head:head,screwPose:bodyPose,engagementPoint:[0.0,0.0,0.1],hole:[0.0,0.0,0.1],direction:[0.0,0.0,1.0],damagedLeadIn:false})});
      outputs.selectProgram(2); outputs.vacuum(true); outputs.start(true); harness.step();
      check(observer.reading().result == ScrewdriverContract.NO_SCREW,"An absent identified screw reports no screw on the simulation backend");
      outputs.start(false); harness.step();
      harness.removeObject(screw);
      var misalignedScrew = harness.spawnBox([0,0,0.101],[0.0001,0.0001,0.0001],true,0.01);
      observer.configure(3,{program:program(),joint:joint(),screw:misalignedScrew,hand:1,
        work:() -> ({head:head,screwPose:bodyPose,engagementPoint:[0.0,0.0,0.1],hole:[0.01,0.0,0.1],direction:[0.0,0.0,1.0],damagedLeadIn:false})});
      outputs.selectProgram(3); harness.step(); outputs.start(true);
      for (_ in 0...5000) if (observer.reading().result == 0) harness.step();
      check(observer.reading().result == ScrewdriverContract.NO_START && observer.model != null,
        "A laterally misaligned live hole reports no start without guessing another axis");
      harness.simulation.removeStepObserver(observerId);
    } catch (error:Dynamic) { harness.dispose(); throw error; }
    harness.dispose();
  }
}

private class DriverDeviceFixture implements ScrewdriverOutputs implements ScrewdriverFeedback {
  public var slot = 0;
  public var running = false;
  public var reversing = false;
  public var held = false;
  public var feedback:ScrewdriverReading = ScrewdriverSensor.idle();
  public function new() {}
  public function selectProgram(slot:Int):Void this.slot = slot;
  public function start(on:Bool):Void running = on;
  public function reverse(on:Bool):Void reversing = on;
  public function vacuum(on:Bool):Void held = on;
  public function reading():ScrewdriverReading return feedback;
}

/** Immediate position moves isolate process sequencing from the arm planner. */
private class DriverMotionFixture implements ScrewdrivingMotion {
  public final targets:Array<Transform3> = [];
  var done = false;
  public function new() {}
  public function move(target:Transform3,contact:Bool):Void { targets.push(target); done = false; }
  public function update(durationSeconds:Float):Void done = true;
  public function completed():Bool return done;
  public function failure():Null<String> return null;
  public function abort():Void done = false;
}
