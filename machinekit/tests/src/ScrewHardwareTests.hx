import haxe.Int64;
import haxe.io.Bytes;
import processkit.screwdriving.*;
import processkit.screwdriving.openprotocol.*;
import processkit.screwdriving.ScrewdriverIo.ScrewdriverIoSample;
import processkit.screwdriving.ScrewdriverAcquisitionSource.ScrewdriverAcquisition;
import robotkit.execution.FiredProcessEvent;
import robotkit.execution.ProcessEventValue;
import robotkit.execution.ProcessChannelDeclaration;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeJointBlueprint;
import robotkit.runtime.RobotRuntimeSensorBlueprint;
import robotkit.core.SensorFrame;
import RobotKitRuntime;

private class HardwareProtocolScript implements OpenProtocolTransport {
	public final mids:Array<Int> = [];
	public final replies:Array<Bytes> = [];
	public var automatic = true;
	public var closed = false;
	public function new() {}
	public function send(bytes:Bytes):Void {
		var frame = OpenProtocolFrame.decode(bytes); mids.push(frame.mid);
		if (!automatic) return;
		if (frame.mid == 1) replies.push(new OpenProtocolFrame(2,"010001020103" + spaces(25)).bytes);
		else if (frame.mid == 9999) replies.push(frame.bytes);
		else if (frame.mid != 62) replies.push(new OpenProtocolFrame(5,OpenProtocolFrame.digits(frame.mid,4)).bytes);
	}
	static function spaces(n:Int):String {var value = ""; for (_ in 0...n) value += " "; return value;}
	public function receive():Bytes {
		if (closed) throw "Scripted controller closed";
		if (replies.length == 0) return Bytes.alloc(0);
		var value = replies.shift(); return value == null ? Bytes.alloc(0) : value;
	}
	public function close():Void closed = true;
}

private class HardwareMeasuredIo implements ScrewdriverIo {
	public final pins = [false,false,false,false,false];
	public var latest:Null<ScrewdriverIoSample> = null;
	public function new() {}
	public function write(output:Int, value:Bool):Void pins[output] = value;
	public function sample():Null<ScrewdriverIoSample> return latest;
	public function acquire(sequence:Int, now:Float, code:Int):Void {
		latest = {sequence:Int64.ofInt(sequence),acquiredAt:now,clockId:"hardware-test-clock",
			selectedProgramCode:code,inputs:[true,false,false],values:[0.2,12.3,pins[2] || pins[3] ? 2 : 0,5,2,0,1]};
	}
}

private class OwnerScrewDevice implements PolledScrewdriverDevice implements ScrewdriverOutputs implements ScrewdriverFeedback {
	var selected = false;
	var active = false;
	var vacuumOn = false;
	var angle = 0.0;
	var acquiredAt = 0.0;
	var sequence = 0;
	public function new() {}
	public function outputs():ScrewdriverOutputs return this;
	public function feedback():ScrewdriverFeedback return this;
	public function ready():Bool return selected;
	public function faultDetail():Null<String> return null;
	public function poll(now:Float):Void {if (active) angle += 0.01; acquiredAt = now; sequence++;}
	public function acquisition():Null<ScrewdriverAcquisition> return {sequence:Int64.ofInt(sequence),acquiredAt:acquiredAt,
		clockId:"hardware-owner-test-clock",values:ScrewdriverSensor.values(reading())};
	public function reading():ScrewdriverReading return {torqueNm:0.2,angle:angle,speed:active ? 2 : 0,
		strokeMm:5,phase:2,result:0,vacuum:vacuumOn};
	public function selectProgram(slot:Int):Void selected = true;
	public function start(on:Bool):Void active = on;
	public function reverse(on:Bool):Void {}
	public function vacuum(on:Bool):Void vacuumOn = on;
	public function safe():Void {active = false; selected = false;}
	public function safeAcknowledged():Bool return !active;
	public function close():Void safe();
}

private class FixedAcquisitionDevice extends OwnerScrewDevice {
	public function new() super();
	override public function poll(now:Float):Void {}
	override public function acquisition():Null<ScrewdriverAcquisition> return {sequence:Int64.ofInt(7),acquiredAt:0.125,
		clockId:"hardware-capture-test",values:ScrewdriverSensor.values(reading())};
}

/** Hardware adapters preserve the same channel and measured-feedback boundary. */
class ScrewHardwareTests {
	static var count = 0;
	public static function main():Void {run(); native();}
	static function check(value:Bool, message:String):Void {count++; if (!value) throw message;}
	static function rejects(f:Void->Void):Void {
		var refused = false; try f() catch (_:Dynamic) refused = true;
		check(refused,"Unsafe hardware output must be refused");
	}
	static function profile(reverseNeedsStart:Bool = true):ScrewdriverIoProfile return new ScrewdriverIoProfile(
		[0,1],[{slot:1,code:1},{slot:2,code:2}],2,3,4,0,1,2,reverseNeedsStart,"hardware-test-clock",0.05);
	static function event(channel:String, value:ProcessEventValue):FiredProcessEvent
		return new FiredProcessEvent(Int64.ofInt(0),channel,value,Int64.ofInt(0),Int64.ofInt(0),0);
	static function await(test:Void->Bool, message:String):Void {
		var deadline = Sys.time() + 2;
		while (!test() && Sys.time() < deadline) Sys.sleep(0.002);
		check(test(),message);
	}
	public static function run():Void {
		count = 0;
		for (needsStart in [false,true]) {
			var io = new HardwareMeasuredIo(), driver = new DiscreteScrewdriver(io,profile(needsStart));
			driver.selectProgram(1); io.acquire(1,0,2); driver.poll(0);
			check(!driver.ready(),"Ready bit cannot acknowledge a different program");
			io.acquire(2,0.01,1); driver.poll(0.01); check(driver.ready(),"Actual selected program acknowledges readiness");
			driver.start(false);
			check(!driver.safeAcknowledged(),"Stop needs an acquisition later than its command");
			driver.reverse(true); driver.start(true);
			check(io.pins[3] && io.pins[2] == needsStart,"Unchanged recovery sequence respects commissioned reverse/start wiring");
			driver.start(false); check(!io.pins[2] && !io.pins[3],"Stop drops every energized reverse output");
			driver.reverse(false);
		}
		var changedIo = new HardwareMeasuredIo(), changed = new DiscreteScrewdriver(changedIo,profile());
		changed.selectProgram(1); changedIo.acquire(1,0,1); changed.poll(0);
		changedIo.acquire(1,0,2); changed.poll(0);
		check(changed.faultDetail() != null,"Repeated acquisition cannot change the selected program echo");
		var server = new HardwareProtocolScript(), io = new HardwareMeasuredIo();
		var client = new OpenProtocolClient(server), hardware = new OpenProtocolScrewdriver(client,io,profile());
		var device = new ScrewdriverProcessDevice(hardware,hardware,
			{start:"start",direction:"reverse",program:"program",vacuum:"vacuum"},1,hardware);
		device.prepare(); rejects(() -> hardware.start(true));
		for (i in 0...10) {io.acquire(i+1,i*0.01,1); hardware.poll(i*0.01);}
		check(device.ready(),"Common process device waits for network acknowledgement and measured readiness");
		check(server.mids.join(",") == "1,42,60,18,43","Program acknowledgement precedes enable");
		device.apply([event("vacuum",Digital(true)),event("start",Digital(true))]);
		check(io.pins[2] && io.pins[4],"Named process channels reach physical I/O");
		device.apply([event("start",Digital(false)),event("reverse",Digital(true)),event("start",Digital(true))]);
		check(io.pins[2] && io.pins[3],"Common channel records run the recovery direction");
		device.safe(); check(!io.pins[2] && !io.pins[3] && io.pins[4],"Common safe boundary stops drive and retains vacuum");
		rejects(() -> hardware.start(true));
		for (i in 10...13) {io.acquire(i+1,i*0.01,1); hardware.poll(i*0.01);}
		check(hardware.safeAcknowledged(),"Network disable must be acknowledged");
		device.prepare();
		for (i in 13...20) {io.acquire(i+1,i*0.01,1); hardware.poll(i*0.01);}
		check(device.ready(),"Explicit reprepare rearms without replaying drive");
		check(!io.pins[2] && !io.pins[3],"Rearming leaves spindle stopped");
		check(hardware.reading().angle == 12.3 && hardware.reading().strokeMm == 5,"Measured cumulative angle and feed stay independent of protocol reports");
		hardware.start(true); hardware.poll(0.25);
		check(hardware.faultDetail() != null && !io.pins[2] && !io.pins[3] && io.pins[4],"Stale measured I/O stops enabled network drive");
		hardware.close(); check(!hardware.safeAcknowledged(),"Closing a socket cannot acknowledge physical stop");
		var quiet = new HardwareProtocolScript(); quiet.automatic = false;
		var quietIo = new HardwareMeasuredIo();
		var lost = new OpenProtocolScrewdriver(new OpenProtocolClient(quiet),quietIo,profile());
		lost.selectProgram(1);
		for (i in 0...52) {quietIo.acquire(i+1,i*0.01,1); lost.poll(i*0.01);}
		check(lost.faultDetail() != null && quiet.closed && !quietIo.pins[2] && !quietIo.pins[3],
			"Missing network acknowledgement faults despite fresh measured I/O");
		lost.close();
		var owner = new ScrewdriverDeviceOwner(new OwnerScrewDevice(),Sys.time,0.002);
		var ownedProcess = new ScrewdriverProcessDevice(owner,owner,
			{start:"start",direction:"reverse",program:"program",vacuum:"vacuum"},1,owner);
		var failure:Null<String> = null;
		try {
			rejects(() -> owner.start(true)); ownedProcess.prepare();
			await(ownedProcess.ready,"Owner did not acquire readiness");
			owner.vacuum(true); owner.start(true);
			await(() -> owner.reading().angle > 0,"Owner did not start drive");
			var before = owner.reading().angle; Sys.sleep(0.1);
			check(owner.reading().angle > before && owner.faultDetail() == null,"Planner stall does not starve controller polling");
			var copy = owner.reading(); copy.angle = -1;
			check(owner.reading().angle >= 0,"Owner feedback is an isolated snapshot");
			owner.start(true); ownedProcess.safe(); rejects(() -> owner.start(true));
			await(() -> owner.reading().speed == 0,"Stop did not reach the owner device");
			check(owner.reading().vacuum,"Owner stop preserves vacuum");
			ownedProcess.prepare(); await(ownedProcess.ready,"Owner explicit reprepare failed");
			check(owner.reading().speed == 0,"Owner reprepare does not replay start");
		} catch (error:Dynamic) failure = Std.string(error);
		owner.close(); if (failure != null) throw failure;
		check(!owner.ready(),"Closed owner is never ready");
		trace('Screw hardware binding tests passed ($count checks)');
	}
	static function frame(runtime:RobotRuntime):SensorFrame {
		var snapshot = runtime.snapshot();
		for (i in 0...snapshot.sensors.length) {
			var found = snapshot.sensors.get(i); if (found.sensorId == "driver") return found;
		}
		throw "Native runtime has no commissioned driver frame";
	}
	/** Standalone proof uses the host runtime libraries, outside the pure MachineKit gate. */
	public static function native():Void {
		var before = count;
		var timeout = "";
		try {var invalid = OpenProtocolClient.connect("",0,-1);} catch (error:Dynamic) timeout = Std.string(error);
		check(timeout.indexOf("response timeout") >= 0,"Invalid response timeout is rejected before the TCP constructor");
		var blueprint = new RobotRuntimeBlueprint(1,1,2);
		blueprint.addJoint(new RobotRuntimeJointBlueprint(0,RobotKitRuntimeConstants.RK_RUNTIME_JOINT_REVOLUTE,0,1,-1,1,1));
		blueprint.sensors.push(new RobotRuntimeSensorBlueprint("driver","tool_screwdriver","driver_frame","base_link",0,[0,0,0],[0,0,0,1]));
		for (name in ["start","reverse","vacuum"]) blueprint.addChannel(new ProcessChannelDeclaration(name,Digital(false),name == "vacuum"));
		blueprint.addChannel(new ProcessChannelDeclaration("program",Analog(0)));
		var runtime = RobotRuntime.create(blueprint); runtime.start();
		var names = {start:"start",direction:"reverse",program:"program",vacuum:"vacuum"};
		var binding = new ScrewdriverRuntimeBinding(runtime,new FixedAcquisitionDevice(),() -> 0.125,"driver",names,1,0.002);
		var failure:Null<String> = null;
		try {
			binding.prepare(); await(binding.ready,"Native hardware binding did not initialize");
			await(function() {try return Int64.compare(frame(runtime).sequence,Int64.ofInt(7)) == 0 catch (_:Dynamic) return false;},
				"Measured acquisition did not reach the native mission snapshot");
			var acquired = frame(runtime);
			check(Int64.compare(acquired.sourceTimestampNs,Int64.ofInt(125000000)) == 0 && acquired.sourceClockId == "hardware-capture-test",
				"Native sensor publication retains actual acquisition time and source clock");
			check(acquired.values.get(0) == 0.2 && acquired.values.get(3) == 5,"Native mission consumes the measured seven-value frame");
			Sys.sleep(0.02); var repeated = frame(runtime);
			check(Int64.compare(repeated.receivedTimestampNs,acquired.receivedTimestampNs) == 0,
				"Repeated owner polls cannot refresh an old native acquisition");
			binding.apply([event("another-tool",Digital(true))]); check(binding.fault() == null,"Channel binding leaves other tools with their owners");
			rejects(() -> {var invalid = new ScrewdriverRuntimeBinding(runtime,new FixedAcquisitionDevice(),() -> 0.125,"missing",names,1);});
			rejects(() -> {var invalid = new ScrewdriverRuntimeBinding(runtime,new FixedAcquisitionDevice(),() -> 0.125,"driver",
				{start:"start",direction:"reverse",program:"vacuum",vacuum:"program"},1);});
		} catch (error:Dynamic) failure = Std.string(error);
		binding.close(); runtime.dispose(); if (failure != null) throw failure;
		trace('Screw native sensor binding tests passed (${count-before} checks)');
	}
}
