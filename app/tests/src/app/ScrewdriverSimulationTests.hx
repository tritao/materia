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
import processkit.simulation.SimulatedScrewdriver.ScrewdriverSetup;
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
    runTiltedSeat();
    for (backend in [SimulationSpace.DETERMINISTIC,SimulationSpace.MUJOCO]) {
      runBackend(backend);
      runTransport(backend,false,false,false);
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
    count = 0;
    for (backend in [SimulationSpace.DETERMINISTIC,SimulationSpace.MUJOCO]) {
      runTransport(backend,true,false,false);
      runTransport(backend,true,true,false);
      runTransport(backend,true,false,false,true);
    }
    Sys.println('Screwdriver retained engagement: $count checks passed on both backends');
    count = 0;
    for (backend in [SimulationSpace.DETERMINISTIC,SimulationSpace.MUJOCO]) runTransport(backend,true,false,true);
    Sys.println('Screwdriver head-seat continuation: $count checks passed on both backends');
  }
  // Exercise the process travel computed at a tilted thread entry, then the actual
  // seated-pose correction. A 20 mm lever arm at 0.0004 rad leaves 1.6 nm if only
  // the orientation is corrected. Run away from world axes with an offset head.
  static function runTiltedSeat():Void {
    var frame = new Transform3(new Vec3(0.31,-0.12,0.47),Quat.fromAxisAngle(new Vec3(1,2,3).normalized(),0.7));
    var axis = frame.transformVector(new Vec3(0,0,1));
    var bodyToHead = new Transform3(new Vec3(0,0,-0.001),Quat.identity());
    var lead = new Vec3(0,0,0.02);
    var entry = frame.transformPoint(lead);
    for (tilt in [0.0,0.0004,-0.0004]) {
      var head = frame.compose(new Transform3(new Vec3(0.0002,-0.0001,0),Quat.fromAxisAngle(new Vec3(0,1,0),tilt)));
      var body = head.compose(bodyToHead.inverse());
      var along = head.transformPoint(lead).sub(entry).dot(axis);
      var model = new TighteningModel(program(),joint());
      model.step({screwPresent:true,vacuum:true,start:true,angle:0,strokeMm:2,
        radialErrorMm:0.2,axialErrorMm:along*1000,axisAngle:Math.abs(tilt),damagedLeadIn:false});
      var reading = model.step({screwPresent:true,vacuum:true,start:true,angle:20,strokeMm:2,
        radialErrorMm:0.2,axialErrorMm:along*1000,axisAngle:Math.abs(tilt),damagedLeadIn:false});
      check(reading.result == ScrewdriverContract.OK,"Tilted thread entry completes its qualified tightening process");
      var advance = model.advanceAtAngle(reading.angle)/1000;
      var plane = frame.translation.add(axis.scale(joint().runDownMm/1000));
      var seated = @:privateAccess SimulatedScrewdriver.seatedOnPlane(body,bodyToHead,axis,plane,advance);
      var placed = @:privateAccess SimulatedScrewdriver.turnedPose(seated,seated.compose(bodyToHead).translation,
        [axis.x,axis.y,axis.z],reading.angle,advance);
      var actualHead = new Transform3(new Vec3(placed.x,placed.y,placed.z),
        new Quat(placed.qx,placed.qy,placed.qz,placed.qw)).compose(bodyToHead);
      check(Math.abs(actualHead.translation.sub(plane).dot(axis)) < 1e-12,
        "The seated bearing point lies on its plane after tilted thread-entry travel");
      var expected = head.translation.add(axis.scale(plane.sub(head.translation).dot(axis)));
      check(actualHead.translation.sub(expected).norm() < 1e-12,
        "Seating retains the head's lateral contact offset");
      check(actualHead.transformVector(new Vec3(0,0,1)).sub(axis).norm() < 1e-12,
        "The seated head is aligned with the seat normal");
    }
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
  static function runTransport(backend:Int, retighten:Bool = false, detached:Bool = false, headSeat:Bool = false,
      dynamicFixture:Bool = false):Void {
    var expectedTorque = headSeat ? joint().prevailingTorque : 9.9;
    var harness = new SimulationHarness(0.01,1,backend);
    try {
      var model = new RobotModel("screw transport probe");
      var base = model.addLink(new Link("base")), arm = model.addLink(new Link("arm"));
      var slide = model.addLink(new Link("slide")), carrier = model.addLink(new Link("carrier"));
      var fixtureLink = model.addLink(new Link("fixture"));
      // Distinct probe bodies keep self-collision enabled without overlapping
      // the default 100 mm fallback boxes at every joint origin.
      var bodies = [base,arm,slide,carrier,fixtureLink];
      for (i in 0...bodies.length)
        bodies[i].collisionShapes.push(new CollisionShape(Box(0.001,0.001,0.001),[0,(i-2)*0.01,0]));
      var travel = model.addJoint(new Joint("travel",JointType.Prismatic,base,arm));
      travel.axis = [1,0,0]; travel.limits = new JointLimits(0,0.04,0.1,100);
      var feed = model.addJoint(new Joint("feed",JointType.Prismatic,arm,slide));
      feed.axis = [0,0,1]; feed.limits = new JointLimits(-0.02,0,0.1,40);
      var spindle = model.addJoint(new Joint("spindle",JointType.Continuous,slide,carrier));
      spindle.axis = [0,0,1]; spindle.limits = new JointLimits(-1000,1000,60,20);
      var fixtureTravel = model.addJoint(new Joint("fixtureTravel",JointType.Prismatic,base,fixtureLink));
      fixtureTravel.axis = [0,1,0]; fixtureTravel.limits = new JointLimits(0,0.04,0.1,100);
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
      // A separate supported member under the head: gravity acts until a qualified final preload joins it.
      harness.spawnBox([0.03,0.03,0.0959],[0.005,0.005,0.005]);
      var clamped = harness.spawnBox([0.03,0.03,0.101],[0.0001,0.0001,0.0001],true,0.01);
      var freeFixture = harness.space.session.createObject(nativekit.sim.MotionType.Kinematic,
        nativekit.sim.SimShape.box(0.001,0.001,0.001),new SimPose(0.02,0.01,0.1),0);
      var channels:ScrewdriverChannelNames = {start:"start",direction:"reverse",program:"program",vacuum:"vacuum"};
      var observer = new SimulatedScrewdriver(harness.simulation,runtime,0,3,1,2,[0,0,0.103],[0,0,1],channels,"driver",20,60,20,2,1,
        model.turningInertia(spindle),model.turningInertia(feed));
      var head = new SimPose(0.02,0,0.1,0,0,0,1);
      var body = new SimPose(0.02,0,0.101,0,0,0,1);
      var fixture = new SimulationAssemblyPart("work-fixture",harness.simulation,[0.0,0.0,0.0],[],
        {id:"work-fixture",robotIndex:0,linkIndex:4,offset:materia.assembly.AssemblyFrames.identity(),center:[0.0,0.0,0.0]},null);
      var fixtureObject:Null<nativekit.sim.SimObject> = null;
      if (dynamicFixture) {
        var member = harness.spawnBox([0,0.07,0.01],[0.001,0.001,0.001],true,0.1);
        fixture.retain(member,harness.simulation.objectPose(member));
        fixtureObject = member;
        fixture = new SimulationAssemblyPart("work-fixture",harness.simulation,[0.0,0.0,0.0],[],null,member);
      }
      observer.configure(1,{program:new TighteningProgram(program().settings,headSeat),joint:joint(),screw:screw,hand:1,
        fixture:fixture,clamped:clamped,
        connection:{fastening:"transport-probe",member:"work-fixture",face:"bore"},
        work:() -> ({head:head,screwPose:body,engagementPoint:[0.02,0.0,0.1],hole:[0.02,0.0,0.1],direction:[0.0,0.0,1.0],damagedLeadIn:false})});
      var id = harness.simulation.addStepObserver(observer);
      var outputs = new ChannelScrewdriverOutputs(runtime,channels);
      outputs.selectProgram(1);
      // Let gravity and the physical nest settle before the pickup command.
      for (_ in 0...100) harness.step();
      outputs.vacuum(true); harness.step();
      var pickupBody = harness.simulation.objectPose(screw);
      check(observer.carried() == screw,"Pickup uses the actual identified socket, away from the work frame");
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
      check(observer.reading().result == ScrewdriverContract.OK &&
        observer.reading().torqueNm == expectedTorque,
        'A screw carried from the presenter completes its qualified pass on the real tool joints (head-seat $headSeat, measured torque ${observer.reading().torqueNm})');
      var seated = harness.simulation.objectPose(screw);
      var retainedTorque = observer.reading().torqueNm;
      check(Math.abs(seated.z-0.103) < 0.0001 && Math.abs(seated.x-0.02) <= 0.00051,
        "The transported screw reaches its seat while retaining its actual lead-in contact offset");
      var continuation:Null<ScrewdriverSetup> = null;
      if (retighten) {
        var settings = program().settings;
        // Explicit probe qualification; this does not add a production torque table entry.
        settings.targetTorque = 14; settings.finalTorqueLimit = 16; settings.minimumAngle = 0.6; settings.maximumAngle = 0.8;
        var qualified:ScrewdriverSetup = {program:new TighteningProgram(settings),joint:joint(),screw:screw,hand:1,
          fixture:fixture,clamped:clamped,connection:{fastening:"transport-probe",member:"work-fixture",face:"bore"},retightening:true,
          work:() -> ({head:head,screwPose:body,engagementPoint:[0.02,0.0,0.1],hole:[0.02,0.0,0.1],direction:[0.0,0.0,1.0],damagedLeadIn:false})};
        continuation = qualified;
        refuses(() -> {var contact = observer.configure(2,qualified);});
      }
      outputs.start(false); harness.step();
      if (retighten) {
        for (_ in 0...100) if (Math.abs(observer.reading().speed)>0.01) harness.step();
        harness.step();
      }
      outputs.vacuum(false); harness.step();
      runtime.submitPosition(0,0,3);
      for (_ in 0...1000) if (runtime.snapshot().q.get(0) > 1e-5) harness.step();
      var released = harness.simulation.objectPose(screw);
      check(observer.carried() == null && Math.abs(released.x-seated.x) < 0.0001 && Math.abs(released.z-seated.z) < 0.0001,
        "Vacuum release retains the actual seated screw through retraction and gravity");
      check(fixture.carries(screw),
        "The named fixture owns the fastened screw after the bit releases it");
      check(fixture.carries(clamped) == !headSeat,
        "Final preload retains the named clamped member; a snug-only pass leaves it free");
      if (retighten) {
        if (continuation == null) throw "Retightening probe lost its qualification";
        var qualified:ScrewdriverSetup = cast continuation;
        var identity = qualified.connection;
        if (identity == null) throw "Retightening probe lost its named connection";
        identity.face = "another-bore";
        refuses(() -> {var contact = observer.configure(2,qualified);});
        identity.face = "bore";
        var other = harness.spawnBox([0.01,0,0.101],[0.0001,0.0001,0.0001],true,0.01);
        var wrong:ScrewdriverSetup = {program:qualified.program,joint:qualified.joint,screw:other,hand:1,
          fixture:fixture,connection:identity,retightening:true,work:qualified.work};
        refuses(() -> {var contact = observer.configure(2,wrong);});
        var wrongFixture = new SimulationAssemblyPart("different-fixture",harness.simulation,[0.0,0.0,0.0],[],null,freeFixture);
        var misplaced:ScrewdriverSetup = {program:qualified.program,joint:qualified.joint,screw:screw,hand:1,
          fixture:wrongFixture,connection:identity,retightening:true,work:qualified.work};
        refuses(() -> {var contact = observer.configure(2,misplaced);});
        // CAD members can share a native rigid parent. That parent alone does
        // not identify the member on which this particular joint was released.
        var alias = fixtureObject != null
          ? new SimulationAssemblyPart("another-member",harness.simulation,[0.0,0.0,0.0],[],null,fixtureObject)
          : new SimulationAssemblyPart("another-member",harness.simulation,[0.0,0.0,0.0],[],
          {id:"another-member",robotIndex:0,linkIndex:4,offset:materia.assembly.AssemblyFrames.identity(),center:[0.0,0.0,0.0]},null);
        var aliased:ScrewdriverSetup = {program:qualified.program,joint:qualified.joint,screw:screw,hand:1,
          fixture:alias,connection:identity,retightening:true,work:qualified.work};
        if (!alias.carries(screw)) throw "Alias probe must share the actual native parent";
        refuses(() -> {var contact = observer.configure(2,aliased);});
        var contact = observer.configure(2,qualified);
        check(contact != null && Math.abs(contact.socket.translation.z-(released.z-0.001))<1e-8 && contact.advanceMm > 1.9,
          "Final contact comes from the actual released body and retained calibrated advancement");
        var prior = observer.model;
        if (prior == null) throw "Retightening probe lost its engaged model";
        var retainedAngle = prior.reading.angle;
        outputs.selectProgram(2); harness.step();
        check(observer.reading().result == ScrewdriverContract.NONE && Math.abs(observer.reading().angle-retainedAngle)<1e-9 &&
          observer.reading().torqueNm == retainedTorque && observer.reading().strokeMm == 0 && fixture.carries(screw),
          "Selecting final retains preload and angle without moving or acquiring the seated body");
        runtime.submitPosition(0,0.02,5);
        for (_ in 0...1000) if (Math.abs(runtime.snapshot().q.get(0)-0.02)>1e-5) harness.step();
        outputs.vacuum(true); harness.step(); harness.step();
        // Still seated in its thread, the screw stays on its member while the bit takes it in the socket.
        check(observer.carried() == screw && fixture.carries(screw) && observer.reading().strokeMm > 0.5,
          'Final recontact acquires the same body and compresses the physical feed before starting ' +
          '(carried ${observer.carried() == screw}, member ${fixture.carries(screw)}, stroke ${observer.reading().strokeMm})');
        outputs.start(true);
        for (_ in 0...5000) if (observer.reading().result == ScrewdriverContract.NONE) harness.step();
        check(observer.reading().result == ScrewdriverContract.OK && observer.reading().torqueNm == 14 &&
          Math.abs(observer.reading().angle-retainedAngle-(14-retainedTorque)/19.4)<0.02 &&
          Math.abs(harness.simulation.objectPose(screw).z-released.z)<0.00001,
          'Final adds only remaining elastic rotation, preserves the original snug window and leaves the body seated (backend $backend, head-seat $headSeat, source torque $retainedTorque, source angle $retainedAngle, result ${haxe.Json.stringify(observer.reading())}, seat ${seated.z}, actual ${harness.simulation.objectPose(screw).z})');
        outputs.start(false);
        for (_ in 0...100) if (Math.abs(observer.reading().speed)>0.01) harness.step();
        outputs.vacuum(false); harness.step();
        runtime.submitPosition(0,0,6);
        for (_ in 0...1000) if (runtime.snapshot().q.get(0)>1e-5) harness.step();
        check(fixture.carries(screw) && observer.carried() == null,
          "Successful final release returns the exact screw to its named fixture");
        var settings = program().settings;
        settings.targetTorque = 18; settings.finalTorqueLimit = 20; settings.minimumAngle = 0.8; settings.maximumAngle = 1;
        var higher:ScrewdriverSetup = {program:new TighteningProgram(settings),joint:joint(),screw:screw,hand:1,
          fixture:fixture,connection:identity,retightening:true,work:qualified.work};
        var eligible = observer.configure(3,higher);
        check(eligible != null,"Ownership and reset tests start from a valid higher-target continuation");
        if (detached) {
          var actual = harness.simulation.objectPose(screw);
          fixture.releaseRigid(screw);
          check(!fixture.carries(screw),"Native detachment removes the joint even when its body pose is unchanged");
          refuses(() -> {var contact = observer.configure(3,higher);});
          fixture.retain(screw,actual);
          refuses(() -> {var contact = observer.configure(3,higher);});
        } else {
          observer.reset();
          refuses(() -> {var contact = observer.configure(3,higher);});
          // Keep the remaining fixture-motion/freshness checks on an idle slot.
          outputs.selectProgram(1);
        }
      }
      var clampedBefore = harness.simulation.objectPose(clamped);
      runtime.submitPosition(3,0.01,retighten ? 7 : 4);
      for (_ in 0...1000) if (Math.abs(runtime.snapshot().q.get(3)-0.01) > 1e-5) harness.step();
      harness.step();
      var moved = harness.simulation.objectPose(screw);
      check(Math.abs(moved.y-released.y-0.01) < 0.0001 && Math.abs(moved.z-released.z) < 0.0001,
        "The seated screw follows its moving fixture independently of the withdrawn bit");
      check(fixture.carries(clamped) && Math.abs(harness.simulation.objectPose(clamped).y-clampedBefore.y-0.01) < 0.0001,
        "The clamped member travels with the threaded fixture after accepted final tightening");
      var member = new SimulationAssemblyPart("free-fixture",harness.simulation,[0.0,0.0,0.0],[],null,freeFixture);
      fixture.releaseRigid(screw);
      member.retain(screw,moved);
      freeFixture.drive(new SimPose(0.02,0.02,0.1));
      harness.step(); harness.step();
      check(member.carries(screw) && Math.abs(harness.simulation.objectPose(screw).y-moved.y-0.01) < 0.0001,
        "A named environment fixture also carries its fastened screw without a robot-link guess");
      refuses(() -> harness.simulation.holdObjectOnObject(screw,screw,new SimPose(0,0,0)));
      var plate = harness.spawnBox([0.05,0.05,0.1],[0.0001,0.0001,0.0001],true,0.01);
      var assembly = new SimulationAssemblyPart("dynamic-plate",harness.simulation,[0.0,0.0,0.0],[],null,plate);
      member.releaseRigid(screw);
      fixture.releaseRigid(clamped);
      var screwBeforeCarry = harness.simulation.objectPose(screw), housingBeforeCarry = harness.simulation.objectPose(clamped);
      assembly.retainRigid(screw);
      var plateBeforeJoining = harness.simulation.objectPose(plate);
      assembly.retainRigid(clamped);
      var plateAfterJoining = harness.simulation.objectPose(plate), housingAfterJoining = harness.simulation.objectPose(clamped);
      check(Math.abs(plateAfterJoining.x-plateBeforeJoining.x) < 1e-6 &&
        Math.abs(plateAfterJoining.y-plateBeforeJoining.y) < 1e-6 && Math.abs(plateAfterJoining.z-plateBeforeJoining.z) < 1e-6 &&
        Math.abs(housingAfterJoining.x-housingBeforeCarry.x) < 1e-6 &&
        Math.abs(housingAfterJoining.y-housingBeforeCarry.y) < 1e-6 && Math.abs(housingAfterJoining.z-housingBeforeCarry.z) < 1e-6,
        "Physical housing retention preserves both measured body positions before pickup");
      fixture.retain(plate,harness.simulation.objectPose(plate));
      runtime.submitPosition(3,0.02,retighten ? 8 : 5);
      for (_ in 0...1000) if (Math.abs(runtime.snapshot().q.get(3)-0.02) > 1e-5) harness.step();
      harness.step();
      check(fixture.carries(plate) && assembly.carries(screw) && assembly.carries(clamped) &&
        Math.abs(harness.simulation.objectPose(screw).y-screwBeforeCarry.y-0.01) < 0.0001 &&
        Math.abs(harness.simulation.objectPose(clamped).y-housingBeforeCarry.y-0.01) < 0.0001,
        "Picking the dynamic plate carries its retained screw and physically fastened housing through the native carrier chain");
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
      // The compliant feed rests on its extended end stop, 1.1 mm past the soft limit as on the CAD screwdriver.
      feed.limits.overtravel = 0.0011;
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
      var observer = new SimulatedScrewdriver(harness.simulation,runtime,0,3,1,2,[0,0,0.103],[0,0,1],channels,"driver",20,60,20,2,1,
        model.turningInertia(spindle),model.turningInertia(feed));
      var head = new SimPose(0,0,0.1,0,0,0,1);
      var bodyPose = new SimPose(0,0,0.101,0,0,0,1);
      var armOwns = false;
      observer.configure(1,{program:program(),joint:joint(),screw:screw,hand:1,armOwnsAxialMotion:() -> armOwns,
        work:() -> ({head:head,screwPose:bodyPose,engagementPoint:[0.0,0.0,0.1],hole:[0.0,0.0,0.1],direction:[0.0,0.0,1.0],damagedLeadIn:false})});
      var observerId = harness.simulation.addStepObserver(observer);
      var outputs = new ChannelScrewdriverOutputs(runtime,channels);
      outputs.selectProgram(1);
      // MuJoCo resolves support contact after integration; seal after the nest settles.
      for (_ in 0...100) harness.step();
      outputs.vacuum(true); harness.step();
      check(observer.carried() == screw && observer.reading().phase == ScrewdriverContract.IDLE,
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
      var process = observer.model;
      if (process == null) throw "Started driver has no process model";
      var paused = process.reading.angle;
      outputs.start(false);
      harness.step();
      for (_ in 0...5000) if (Math.abs(observer.reading().speed) > 0.01) harness.step();
      var measuredAngle = Math.max(0,runtime.snapshot().q.get(2)-(@:privateAccess observer.origin));
      check(process.reading.angle == paused && observer.reading().angle == measuredAngle &&
        observer.carried() == screw && Math.abs(observer.reading().speed) <= 0.01,
        "Stop pauses process advance and retains the screw while feedback follows the settling physical shaft");
      outputs.start(true);
      for (_ in 0...5000) if (observer.reading().result == 0) harness.step();
      var result = observer.reading();
      check(result.result == ScrewdriverContract.OK && Math.abs(result.torqueNm-9.9) < 1e-9,
        'Measured spindle rotation reaches the qualified torque and angle window (backend $backend, ${haxe.Json.stringify(result)})');
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
      check(observer.reading().angle == 0 && observer.carried() == screw,"Reverse backs out the driven angle without releasing the screw");
      outputs.reverse(false); refuses(() -> harness.step());
      outputs.start(false); outputs.reverse(false); outputs.vacuum(false); harness.step();
      check(observer.carried() == null,"Vacuum off releases the screw");
      observer.reset();
      check(observer.reading().phase == ScrewdriverContract.IDLE && observer.model == null,"Reset clears the process and holding state");
      observer.configure(2,{program:program(),joint:joint(),screw:null,hand:1,
        work:() -> ({head:head,screwPose:bodyPose,engagementPoint:[0.0,0.0,0.1],hole:[0.0,0.0,0.1],direction:[0.0,0.0,1.0],damagedLeadIn:false})});
      outputs.selectProgram(2); outputs.vacuum(true); outputs.start(true); harness.step();
      check(observer.reading().result == ScrewdriverContract.NO_SCREW,"An absent identified screw reports no screw on the simulation backend");
      outputs.start(false); harness.step();
      // While the arm owns axial motion the screw is a rigid physical child of the bit, so its contacts load the
      // feed and the arm; handing back to the process model regrips it by vacuum where it stands. Away from any
      // thread start (a hole 30 mm below), nothing anchors it.
      observer.configure(4,{program:program(),joint:joint(),screw:screw,hand:1,armOwnsAxialMotion:() -> armOwns,
        work:() -> ({head:head,screwPose:bodyPose,engagementPoint:[0.0,0.0,0.1],hole:[0.0,0.0,0.07],direction:[0.0,0.0,1.0],damagedLeadIn:false})});
      outputs.selectProgram(4); harness.step();
      check(observer.carried() == screw,'The reset screw is picked again for the weld check (backend $backend)');
      armOwns = true; observer.prepareCarriage();
      check(observer.welded() && observer.carried() == screw,'An arm-owned search welds the held screw to the bit (backend $backend)');
      check(switch runtime.channelValue("vacuum") { case Digital(on): on; case _: false; },
        'Welding between ticks leaves the controller\'s vacuum output on (backend $backend)');
      for (index in 0...10) {
        harness.step();
        check(observer.welded() && observer.carried() == screw,
          'The weld holds through arm-owned steps (backend $backend, step $index, carriage ' +
          '${Std.string(@:privateAccess observer.carriage)}, vacuum ${runtime.channelValue("vacuum")}, ' +
          'reading ${haxe.Json.stringify(observer.reading())}, safety ${runtime.snapshot().safety}, ' +
          'fault ${runtime.snapshot().faultCode}, screw ${haxe.Json.stringify(harness.simulation.objectPose(screw))})');
      }
      var welded = harness.simulation.objectPose(screw);
      armOwns = false; observer.prepareCarriage();
      var regripped = harness.simulation.objectPose(screw);
      check(!observer.welded() && observer.carried() == screw &&
        Math.abs(regripped.x-welded.x)+Math.abs(regripped.y-welded.y)+Math.abs(regripped.z-welded.z) < 1e-9,
        'Handing back to the process model regrips the screw by vacuum where it stands (backend $backend, welded ' +
        '${observer.welded()}, held ${observer.carried() == screw}, welded pose ${haxe.Json.stringify(welded)}, ' +
        'regripped ${haxe.Json.stringify(regripped)})');
      for (_ in 0...10) harness.step();
      outputs.vacuum(false); harness.step();
      check(observer.carried() == null,'Vacuum off releases the regripped screw (backend $backend)');
      outputs.vacuum(true);
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
  var idle = true;
  var stopping = false;
  public function new() {}
  public function move(target:Transform3,contact:Bool):Void { targets.push(target); done = false; idle = false; }
  public function update(durationSeconds:Float):Void {idle = true; done = !stopping; stopping = false;}
  public function completed():Bool return done;
  public function failure():Null<String> return null;
  public function synchronize():Void {}
  public function stop():Void {stopping = true; idle = false; done = false;}
  public function stopped():Bool return idle;
  public function abort():Void {done = false; idle = false;}
}
