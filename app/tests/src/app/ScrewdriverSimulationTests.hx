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
    for (backend in [SimulationSpace.DETERMINISTIC,SimulationSpace.MUJOCO]) runBackend(backend);
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
      sensor.frame = model.addFrame(new Frame("tip",carrier));
      var blueprint = RobotRuntimeCompiler.compile(model,new robotkit.profile.RobotProfile());
      blueprint.addTool(new robotkit.tool.ScrewdriverChannels("start","reverse","program","vacuum"));
      blueprint.processModelJoints.push(1); blueprint.processModelJoints.push(2);
      var runtime = harness.simulation.addRobot(blueprint);
      // The default link box extends 50 mm from its origin; the probe socket lies beyond it.
      var screw = harness.spawnBox([0,0,0.101],[0.0001,0.0001,0.0001],true,0.01);
      var channels:ScrewdriverChannelNames = {start:"start",direction:"reverse",program:"program",vacuum:"vacuum"};
      var observer = new SimulatedScrewdriver(harness.simulation,runtime,0,3,1,2,[0,0,0.103],[0,0,1],channels,"driver",20,60,20,2);
      var head = new SimPose(0,0,0.1,0,0,0,1);
      var bodyPose = new SimPose(0,0,0.101,0,0,0,1);
      observer.configure(1,{program:program(),joint:joint(),screw:screw,hand:1,
        work:() -> ({head:head,screwPose:bodyPose,engagementPoint:[0.0,0.0,0.1],hole:[0.0,0.0,0.1],direction:[0.0,0.0,1.0],damagedLeadIn:false})});
      var observerId = harness.simulation.addStepObserver(observer);
      var outputs = new ChannelScrewdriverOutputs(runtime,channels);
      outputs.selectProgram(1); outputs.vacuum(true);
      harness.step();
      check(observer.holding.held == screw && observer.reading().phase == ScrewdriverContract.IDLE,
        'Vacuum attaches the identified screw before start (backend $backend, reading ${haxe.Json.stringify(observer.reading())}, body ${haxe.Json.stringify(harness.simulation.objectPose(screw))}, link ${haxe.Json.stringify(harness.simulation.linkPose(0,3))})');
      check(Math.abs(observer.reading().strokeMm - 3) < 0.01,"Stroke comes from free TCP depth against the head");
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
