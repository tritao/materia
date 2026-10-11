import processkit.screwdriving.*;
import processkit.screwdriving.TighteningModel.TighteningInput;
import processkit.screwdriving.TighteningModel.TighteningJoint;
import robotkit.tool.ScrewdriverContract;
import machinekit.standard.SocketHeadCapScrew;

/** Explicit test qualification, never a production program inferred from the torque catalog. */
class ScrewdriverProcessTests {
	static var count = 0;
	static function check(value:Bool, message:String):Void { count++; if (!value) throw message; }
	static function rejects(run:Void -> Void):Void {
		var refused = false; try run() catch (_:Dynamic) refused = true;
		check(refused, "Invalid screwdriver input must be refused");
	}
	public static function program():TighteningProgram return new TighteningProgram({runDownSpeed:20,finalSpeed:2,
		targetTorque:SocketHeadCapScrew.metric("M6",20,Steel88).tightening().torqueNm,
		minimumAngle:0.4,maximumAngle:0.6,runDownTorqueLimit:1,finalTorqueLimit:12,maximumRunDownAngle:30,
		minimumStroke:1,maximumStroke:15});
	public static function joint():TighteningJoint return {leadMm:1,runDownMm:2,leadInMm:0.5,maximumAxisAngle:0.1,
		springRate:2,prevailingTorque:0.2,stiffnessNmPerRad:19.4,stripTorque:20};
	static function input(angle:Float):TighteningInput return {screwPresent:true,vacuum:true,start:true,angle:angle,
		strokeMm:5,radialErrorMm:0,axialErrorMm:0,axisAngle:0,damagedLeadIn:false};
	static function runToEnd(model:TighteningModel, sample:TighteningInput):ScrewdriverReading {
		var result = ScrewdriverSensor.idle();
		for (i in 0...4000) if (result.result == 0) {sample.angle = i * 0.01; result = model.step(sample);}
		return result;
	}
	public static function run():Void {
		count = 0;
		var good = new TighteningModel(program(),joint());
		var ok = runToEnd(good,input(0));
		check(ok.result == ScrewdriverContract.OK && Math.abs(ok.torqueNm - 9.9) < 1e-9 && good.advanceMm == 2,
			"M6 steel reaches its published torque in the explicitly qualified half-radian window");
		check(Math.abs(ok.angle - 4*Math.PI - 0.5) < 1e-9 && ok.speed == 0, "Final angle is measured from snug and the spindle stops at target");
		var sample = input(0); sample.screwPresent = false;
		check(new TighteningModel(program(),joint()).step(sample).result == ScrewdriverContract.NO_SCREW, "No screw");
		sample = input(0); sample.radialErrorMm = 1;
		var misaligned = new TighteningModel(program(),joint());
		check(runToEnd(misaligned,sample).result == ScrewdriverContract.NO_START && misaligned.advanceMm == 0, "Misaligned screw slips without advance");
		sample = input(0); sample.axialErrorMm = 10;
		check(runToEnd(new TighteningModel(program(),joint()),sample).result == ScrewdriverContract.NO_START,
			"A tip outside the thread's axial lead-in cannot start");
		sample = input(0); sample.axisAngle = 0.2;
		check(runToEnd(new TighteningModel(program(),joint()),sample).result == ScrewdriverContract.NO_START, "Tilted start");
		sample = input(0); sample.strokeMm = 0.5;
		check(runToEnd(new TighteningModel(program(),joint()),sample).result == ScrewdriverContract.NO_START, "Insufficient spring force");
		sample = input(0); sample.strokeMm = 16;
		check(runToEnd(new TighteningModel(program(),joint()),sample).result == ScrewdriverContract.NO_START, "Excess spring force");
		sample = input(0); sample.damagedLeadIn = true;
		var crossed = new TighteningModel(program(),joint());
		check(runToEnd(crossed,sample).result == ScrewdriverContract.CROSS_THREAD && crossed.advanceMm == 0, "Damaged lead-in raises torque before run-down");
		var strippedJoint = joint(); strippedJoint.stripTorque = 5;
		var stripped = runToEnd(new TighteningModel(program(),strippedJoint),input(0));
		check(stripped.result == ScrewdriverContract.STRIPPED && stripped.torqueNm == 0, "Stripping drops torque");
		var obstructed = joint(); obstructed.prevailingTorque = 2;
		check(runToEnd(new TighteningModel(program(),obstructed),input(0)).result == ScrewdriverContract.EARLY_HIGH_TORQUE, "Early high torque");
		var soft = joint(); soft.stiffnessNmPerRad = 5;
		check(runToEnd(new TighteningModel(program(),soft),input(0)).result == ScrewdriverContract.ANGLE_OUT_OF_WINDOW, "Soft joint exceeds angle window");
		var hard = joint(); hard.stiffnessNmPerRad = 50;
		check(runToEnd(new TighteningModel(program(),hard),input(0)).result == ScrewdriverContract.ANGLE_OUT_OF_WINDOW, "Hard joint reaches torque too early");
		var long = joint(); long.runDownMm = 10;
		check(runToEnd(new TighteningModel(program(),long),input(0)).result == ScrewdriverContract.RUN_DOWN_TOO_LONG, "Run-down watchdog");
		var lost = new TighteningModel(program(),joint()); var first = lost.step(input(0));
		sample = input(1); sample.vacuum = false;
		var lostReading = lost.step(sample);
		check(lostReading.result == ScrewdriverContract.SCREW_LOST && lostReading.angle == 1 && lostReading.speed == 0,
			"Lost vacuum stops at the current measured spindle angle");
		for (offset in [-0.2,0.2]) {
			var actualStart = new TighteningModel(program(),joint());
			var measured = input(0); measured.axialErrorMm = offset;
			var seated = runToEnd(actualStart,measured);
			check(seated.result == ScrewdriverContract.OK && Math.abs(actualStart.advanceMm+offset-2) < 1e-9,
				"Measured lead-in contact offset changes travel while retaining the CAD seat");
		}

		var a = new TighteningModel(program(),joint()), b = new TighteningModel(program(),joint());
		var same = true;
		for (i in 0...1500) if (haxe.Json.stringify(a.step(input(i*0.01))) != haxe.Json.stringify(b.step(input(i*0.01)))) same = false;
		check(same, "Identical samples give identical traces");
		var boundary = new TighteningModel(program(),joint()); var initial = boundary.step(input(0));
		var snug = boundary.step(input(4*Math.PI));
		check(snug.phase == ScrewdriverContract.SNUG, "The head reaches its seat at the exact lead-derived angle");
		var coarse = new TighteningModel(program(),joint()); var coarseInitial = coarse.step(input(0));
		check(coarse.step(input(20)).result == ScrewdriverContract.OK, "A coarse sample stops at target before a later strip threshold");
		var coarseLong = new TighteningModel(program(),long); var longInitial = coarseLong.step(input(0));
		var longResult = coarseLong.step(input(100));
		check(longResult.result == ScrewdriverContract.RUN_DOWN_TOO_LONG && longResult.angle == 30 && coarseLong.advanceMm < 10,
			"A coarse sample cannot seat the screw beyond the run-down watchdog");
		var slipping = input(100); slipping.radialErrorMm = 1;
		check(new TighteningModel(program(),joint()).step(slipping).angle == 30,"Slipping stops at the programmed angle watchdog");
		var damaged = input(100); damaged.damagedLeadIn = true;
		var crossResult = new TighteningModel(program(),joint()).step(damaged);
		check(crossResult.result == ScrewdriverContract.CROSS_THREAD && crossResult.torqueNm == 1 && crossResult.angle < 30,
			"A coarse damaged-lead sample stops at its first phase torque limit");
		var weakLead = joint(); weakLead.stiffnessNmPerRad = 0.0001;
		check(runToEnd(new TighteningModel(program(),weakLead),damaged).result == ScrewdriverContract.NO_START,
			"A blocked lead-in with little torque rise still has a bounded start watchdog");
		check(ScrewdriverSensor.valid(ScrewdriverSensor.values(ok)), "Valid sensor frame");
		var invalid = ScrewdriverSensor.values(ok); invalid[4] = 0.5;
		check(!ScrewdriverSensor.valid(invalid), "Fractional phase refused");
		invalid = ScrewdriverSensor.values(ok); invalid[0] = Math.NaN;
		check(!ScrewdriverSensor.valid(invalid), "Nonfinite sensor frame refused");
		rejects(() -> { var p = program(); p.settings.maximumStroke = 0; var invalidProgram = new TighteningProgram(p.settings); });
		var cell = new ScrewdrivingCell();
		rejects(() -> { var p = machinekit.screwdriving.TighteningPrograms.fromFastening(cell.work.fastenings().require()[0],
			{runDownSpeed:20,finalSpeed:2,runDownTorqueLimit:1,finalTorqueLimit:20,maximumRunDownAngle:30,minimumStroke:1,maximumStroke:15}); });
		var fastening = cell.work.fastenings().require()[0];
		var qualified = new machinekit.screwdriving.Fastening(fastening.screw,fastening.clamped,fastening.threaded,
			fastening.frame,fastening.tip,fastening.engagementLength,fastening.minimumEngagement,fastening.thread,fastening.tightening,{minimum:0.4,maximum:0.6});
		var derived = machinekit.screwdriving.TighteningPrograms.fromFastening(qualified,
			{runDownSpeed:20,finalSpeed:2,runDownTorqueLimit:1,finalTorqueLimit:20,maximumRunDownAngle:30,minimumStroke:1,maximumStroke:15});
		check(derived.settings.targetTorque == fastening.tightening.torqueNm && derived.settings.minimumAngle == 0.4,
			"Program torque and qualified angle limits come from the current CAD fastening");
		check(good.advanceAtAngle(4*Math.PI+0.25) == 2 && Math.abs(good.advanceAtAngle(2*Math.PI)-1)<1e-9,
			"Reverse unloads the elastic angle before backing the screw out by its lead");
		Sys.println('Screwdriver process: $count checks passed');
	}
}
