import processkit.screwdriving.*;
import robotkit.spatial.*;
import robotkit.tool.ScrewdriverContract;

private class FinalOutputs implements ScrewdriverOutputs {
	public var running = false;
	public var holding = false;
	public function new() {}
	public function selectProgram(slot:Int):Void {}
	public function start(on:Bool):Void running = on;
	public function reverse(on:Bool):Void {}
	public function vacuum(on:Bool):Void holding = on;
}
private class FinalMotion implements ScrewdrivingMotion {
	public final targets:Array<Transform3> = [];
	public var stopRequests = 0;
	var done = false;
	var idle = true;
	public function new() {}
	public function move(target:Transform3,contact:Bool):Void {targets.push(target); done = false; idle = false;}
	public function update(dt:Float):Void {done = true; idle = true;}
	public function completed():Bool return done;
	public function failure():Null<String> return null;
	public function synchronize():Void {}
	public function stop():Void {stopRequests++; idle = false; done = false;}
	public function stopped():Bool return idle;
	public function abort():Void done = false;
}

/** Final recontact starts at the installed socket, then follows and recovers the retained engagement. */
class ScrewFinalPassTests {
	static var count = 0;
	static function check(value:Bool,message:String):Void {count++; if (!value) throw message;}
	static function plan():ScrewPlan {
		var head = new Transform3(new Vec3(0.2,0.1,0.302),Quat.fromAxisAngle(new Vec3(0,0,1),0.1));
		return new ScrewPlan("final-probe",1,ScrewdriverProcessTests.program(),
			new Transform3(new Vec3(1,1,1),Quat.identity()),new Transform3(new Vec3(0,0,0),Quat.identity()),1,2,0.04,
			{socket:head,advanceMm:1.8});
	}
	static function runner(motion:FinalMotion,outputs:FinalOutputs):ScrewdrivingRunner
		return new ScrewdrivingRunner(motion,outputs,() -> {throw "Final pass queried a presenter";});
	public static function run():Void {
		count = 0;
		var request = plan(), motion = new FinalMotion(), outputs = new FinalOutputs(), drive = runner(motion,outputs);
		check(Math.abs(request.entry.translation.z-0.3002)<1e-9 && request.entry.rotation.z != 0,
			"Final reference retains the installed socket's roll and actual calibrated travel");
		var moved = request.transformed(new Transform3(new Vec3(1,2,3),Quat.identity()));
		var contact = moved.finalContact;
		check(contact != null && Math.abs(contact.socket.translation.z-3.302)<1e-9 && contact.advanceMm == 1.8,
			"Localizing the plan transforms its installed contact without changing advancement units");
		drive.run(request);
		check(Math.abs(motion.targets[0].translation.z-0.262)<1e-9 && !outputs.running && !outputs.holding,
			"Final approaches the actual installed socket with spindle and vacuum off");
		var reading = ScrewdriverSensor.idle(); reading.angle = 12.7; reading.strokeMm = request.strokeMm;
		drive.update(0.01,reading); drive.update(0.01,reading);
		check(outputs.holding && !outputs.running && motion.targets.length == 2,
			"Final seals at the actual socket before asking for compression");
		reading.vacuum = true; drive.update(0.01,reading);
		check(!outputs.running && Math.abs(motion.targets[2].translation.z-(0.302+request.strokeMm/1000))<1e-9,
			"Final compresses the feed only after acquiring the installed body");
		drive.update(0.01,reading);
		check(outputs.running && motion.targets.length == 3,"Final starts after recontact without pickup or presenter transfer");
		reading.strokeMm -= 1; drive.update(0.01,reading);
		check(Math.abs(motion.targets[3].translation.z-(0.302+(request.strokeMm+1)/1000))<1e-9,
			"Following begins at retained advancement and corrects measured compression");
		reading.result = ScrewdriverContract.OK; reading.torqueNm = 9.9; reading.angle = 13.1; reading.speed = 2;
		drive.update(0.01,reading); drive.update(0.01,reading);
		check(!outputs.running && outputs.holding && !drive.completed(),"Final retains vacuum until measured spindle stop");
		reading.speed = 0; drive.update(0.01,reading); reading.vacuum = false;
		drive.update(0.01,reading); drive.update(0.01,reading);
		var result = drive.tighteningResult;
		check(drive.completed() && result != null && result.angle == 13.1 && result.torqueNm == 9.9 &&
			Math.abs(motion.targets[motion.targets.length-1].translation.z-0.2602)<1e-9,
			"Final preserves cumulative feedback and retracts along the original engaged thread");
		var lostMotion = new FinalMotion(), lostOutputs = new FinalOutputs(), lost = runner(lostMotion,lostOutputs);
		lost.run(request); reading = ScrewdriverSensor.idle(); reading.angle = 12.7;
		lost.update(0.01,reading); lost.update(0.01,reading); reading.vacuum = true; lost.update(0.01,reading);
		reading.vacuum = false; lost.update(0.01,reading);
		check(lostMotion.stopRequests == 1 && !lostOutputs.running && lost.failure() == null,
			"Vacuum loss during final engagement requests a controlled arm stop");
		lost.update(0.01,reading); result = lost.tighteningResult;
		check(lost.failure() != null && result != null && result.outcome == ScrewdriverContract.SCREW_LOST && result.angle == 12.7,
			"Stopped final contact preserves the retained angle and measured loss");
		Sys.println('Screw final-pass runner: $count checks passed');
	}
}
