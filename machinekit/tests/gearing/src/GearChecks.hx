import haxe.Int64;
import motionkit.robot.PlanCheck;
import motionkit.robot.PlanCheck.PlanCheckOptions;
import motionkit.robot.TrajectoryStream.ArrayStreamSegments;
import motionkit.event.TimedEvent;
import motionkit.event.EventValue;
import motionkit.trajectory.PlanDiagnostic;
import robotkit.execution.GearEvents;
import robotkit.execution.GearTerm;
import robotkit.model.*;
import robotkit.model.ActuatorDrive.ServoDrive;
import toolpathkit.motion.*;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.Provenance;

class GearChecks {
  static var assertions = 0;
  static function check(ok:Bool, label:String):Void { assertions++; if (!ok) throw label; }
  static function near(a:Float, b:Float, tolerance:Float, label:String):Void check(Math.abs(a-b) <= tolerance, '$label: $a vs $b');
  static function model():RobotModel {
    var m = new RobotModel("geared table");
    var base = m.addLink(new Link("bed")), hob = m.addLink(new Link("hob")), table = m.addLink(new Link("table"));
    table.mass = 1; table.inertiaTensor = [0.01,0.0,0.0,0.0,0.01,0.0,0.0,0.0,0.01];
    var b = m.addJoint(new Joint("spindle/B", JointType.Continuous, base, hob));
    b.limits = new JointLimits(-1e9,1e9,1000,100,1000);
    var c = m.addJoint(new Joint("table/C", JointType.Continuous, base, table));
    c.limits = new JointLimits(-1e9,1e9,4,30,100);
    var drive = new Actuator("table motor", 30, 4, Transmission.SimpleTransmission("table/C",1,0));
    drive.drive = new ServoDrive(10,30,3,4,0.003,8388608); drive.positionLoopRate = 4000;
    drive.encoder = "table encoder"; m.addActuator(drive);
    var spindle = new Actuator("hob motor",10,1000,Transmission.SimpleTransmission("spindle/B",1,0));
    spindle.processVelocity = new ProcessVelocityDrive("hob.speed","hob.direction",2*Math.PI/60);
    spindle.positionLoopRate = 4000; spindle.encoder = "hob encoder"; m.addActuator(spindle);
    m.addEncoder(Encoder.perRevolution("table encoder","table/C",EncoderKind.Absolute,8388608));
    m.addEncoder(Encoder.perRevolution("hob encoder","spindle/B",EncoderKind.Absolute,8388608));
    return m;
  }
  static function events(ratio:Float, rpm:Float = 3000):Array<TimedEvent> {
    var result = [new TimedEvent(Int64.ofInt(0),"hob.speed",Analog(rpm)), new TimedEvent(Int64.ofInt(0),"hob.direction",Analog(1))];
    for (value in GearEvents.engage(1,[new GearTerm(0,ratio,1000000,2*Math.PI/8388608)],4000)) switch value {
      case Process(command, argument): result.push(new TimedEvent(Int64.ofInt(0),GearEvents.channel(1),Process(command,argument)));
      case _: throw "bad event";
    }
    return result;
  }
  public static function main():Void {
    var m = model(), options = new PlanCheckOptions(); options.steady = new SteadyLoads(0,0);
    options.processAccelerations.set("spindle/B",0);
    var checker = new PlanCheck(m,["table/C"],options);
    var still = new ArrayStreamSegments([{timeFromStartNs:Int64.ofInt(0),durationNs:Int64.ofInt(1000000000),coefficients:[[0.0,0.0]]}]);
    var good = checker.checkSegments(still,0,0,events(1/200));
    check(good.diagnostics.length == 0,"15 rpm geared table passes");
    options.resistingEfforts.set("table/C",100);
    var loadedChecker=new PlanCheck(m,["table/C"],options);
    options.resistingEfforts.set("table/C",0);
    var loaded=loadedChecker.checkSegments(still,0,0,events(1/200));
    check(loaded.diagnostics.filter(d -> d.kind == PlanDiagnosticKind.ServoPeakTorque).length>0,
      "explicit cutting reaction overloads geared C; checker owns copied effort map");
    check(new PlanCheck(m,["table/C"],options).checkSegments(still,0,0,events(1/200)).diagnostics.length==0,
      "zero reaction retains existing geared behavior");
    options.resistingEfforts.set("missing",1);var badEffort=false;
    try{new PlanCheck(m,["table/C"],options);}catch(e:Dynamic){badEffort=true;}
    check(badEffort,"unknown effort joint rejected");options.resistingEfforts.remove("missing");
    var fast = checker.checkSegments(still,1,0,events(1/20));
    var speed = fast.diagnostics.filter(d -> d.kind == PlanDiagnosticKind.GearSpeed);
    check(speed.length == 1 && speed[0].axis == "table/C","overspeed names the table joint");
    near(speed[0].value,3000*2*Math.PI/60/20,1e-9,"master rpm contributes to C speed");
    var same = checker.checkSegments(still,2,0,events(1/100));
    check(same.diagnostics.length == 0,"new tooth count needs no model rebuild");
    var additive = new ArrayStreamSegments([{timeFromStartNs:Int64.ofInt(0),durationNs:Int64.ofInt(1000000000),coefficients:[[0.0,1.0]]}]);
    var sum = checker.checkSegments(additive,3,0,events(1/100));
    check(sum.diagnostics.filter(d -> d.kind == PlanDiagnosticKind.GearSpeed).length == 1,"combined additive and geared speed is checked");
    options.processAccelerations.set("spindle/B",1000000);
    var torque = new PlanCheck(m,["table/C"],options).checkSegments(still,4,0,events(1/200));
    check(torque.diagnostics.filter(d -> d.kind == PlanDiagnosticKind.ServoPeakTorque).length > 0,"geared master acceleration contributes to table torque");
    var released = checker.checkSegments(still,5,0,[new TimedEvent(Int64.ofInt(0),GearEvents.channel(1),Process("gear.release",0))]);
    check(released.diagnostics.length == 0,"release clears geared speed");
    // A one-nanosecond overspeed interval between the ordinary sampling grid still gets checked.
    var shortEvents = events(1/200);
    for (event in events(1/20)) shortEvents.push(new TimedEvent(Int64.ofInt(123450001),event.channel,event.value));
    shortEvents.push(new TimedEvent(Int64.ofInt(123450002),GearEvents.channel(1),Process("gear.release",0)));
    var brief = checker.checkSegments(still,6,0,shortEvents);
    check(brief.diagnostics.filter(d -> d.kind == PlanDiagnosticKind.GearSpeed).length == 1,"gear event boundaries are checked between grid samples");
    var ordinary = model(); ordinary.actuators[1].processVelocity = null;
    var masterPlan = new ArrayStreamSegments([{timeFromStartNs:Int64.ofInt(0),durationNs:Int64.ofInt(1000000000),coefficients:[[0.0,100*Math.PI],[0.0,0.0]]}]);
    var ordinaryEvents = events(1/200);
    check(new PlanCheck(ordinary,["spindle/B","table/C"],options).checkSegments(masterPlan,7,0,ordinaryEvents).diagnostics.length == 0,
      "trajectory master derives geared velocity from its polynomial");
    var missing = false;
    try { new PlanCheck(m,["table/C"]).checkSegments(still,8,0,events(1/200)); } catch (e:Dynamic) {
      missing = Std.string(e).indexOf("spindle/B") >= 0 && Std.string(e).indexOf("acceleration") >= 0;
    }
    check(missing,"missing process acceleration cannot silently omit geared torque");
    var binding = new MachineBinding("work","X","Y","Z",0.1,new ToolpathChannels(""));
    var delays:Map<String,Int> = ["spindle/B"=>1000000,"table/C"=>1000000];
    var declarations = GearBinding.install(binding,m,4000,delays,["table/C"]);
    check(declarations.length == 1 && declarations[0].id == GearEvents.channel(1),"follower has safe-release process declaration");
    var timing = binding.gearEncoders.get("spindle/B"); if (timing == null) throw "Missing encoder timing";
    near(timing.loopHz,4000,1e-9,"driver and owner provide loop rate");
    var span = Provenance.cam(0);
    var lowered = new ToolpathLowering(binding).lower([
      ToolpathOp.ElectronicGear("table/C",[new toolpathkit.path.GearTerm("spindle/B",1/200)],span),
      ToolpathOp.ReleaseGear("table/C",span)]);
    var program = lowered.program; if (program == null) throw "missing gear program";
    check(program.ops.length == 6,"lowering emits atomic begin/loop/term/quantum/engage then release");
    switch program.ops[2] {
      case SetOutput(channel, Process(command, argument)):
        check(channel == "motion.gear/1" && command == "gear.term/0/1000000/1","encoder latency reaches native protocol");
        near(argument,1/200,1e-12,"program ratio survives lowering");
      case _: throw "missing term";
    }
    check(m.couplings.length == 0,"runtime gearing leaves mechanical model unchanged");
    Sys.println('GearChecks: $assertions assertions passed');
  }
}
