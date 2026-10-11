import processkit.screwdriving.*;
import processkit.screwdriving.TighteningModel.TighteningInput;
import robotkit.tool.ScrewdriverContract;

/** Final passes retain the actual engagement, rather than starting another empty thread. */
class ScrewRetighteningTests {
	static var count = 0;
	static function check(value:Bool,message:String):Void {count++; if (!value) throw message;}
	static function rejects(run:Void->Void):Void {
		var failed = false; try run() catch (_:Dynamic) failed = true;
		check(failed,"Invalid retained engagement must be refused");
	}
	static function input(angle:Float):TighteningInput return {screwPresent:true,vacuum:true,start:true,angle:angle,
		strokeMm:5,radialErrorMm:0,axialErrorMm:0,axisAngle:0,damagedLeadIn:false};
	static function snug():TighteningModel {
		var settings = ScrewdriverProcessTests.program().settings;
		// Explicit test qualification: 0.1 rad of elastic rotation, not catalog data.
		settings.targetTorque = 2.14; settings.minimumAngle = 0; settings.maximumAngle = 0.2;
		var model = new TighteningModel(new TighteningProgram(settings),ScrewdriverProcessTests.joint());
		var slipping = input(0.2); slipping.radialErrorMm = 1;
		var ignored = model.step(slipping);
		for (i in 0...2000) if (model.reading.result == ScrewdriverContract.NONE) {
			var sample = input(0.3+i*0.01); sample.axialErrorMm = 0.2;
			var observed = model.step(sample);
		}
		return model;
	}
	public static function run():Void {
		count = 0;
		var seated = snug(), original = seated.reading.angle;
		check(seated.reading.result == ScrewdriverContract.OK && Math.abs(seated.advanceMm-1.8)<1e-9,
			"Snug retains the measured contact travel, including slipping before engagement");
		var finalPass = seated.retighten(ScrewdriverProcessTests.program(),ScrewdriverProcessTests.joint());
		check(finalPass.reading.result == ScrewdriverContract.NONE && finalPass.reading.torqueNm == 2.14 &&
			finalPass.reading.angle == original && finalPass.advanceMm == seated.advanceMm,
			"Final starts with existing preload and cumulative angle, without inventing an OK");
		check(Math.abs(finalPass.advanceAtAngle(0.2+1.8*Math.PI)-0.9)<1e-9 && finalPass.advanceAtAngle(0)==0,
			"Full reversal preserves the original calibrated engagement and thread lead");
		seated.reset();
		check(finalPass.reading.angle == original && Math.abs(finalPass.advanceMm-1.8)<1e-9,
			"Resumed engagement owns independent state");
		var idle = input(original); idle.start = false; idle.vacuum = false;
		var resting = finalPass.step(idle);
		check(resting.result == ScrewdriverContract.NONE && resting.torqueNm == 2.14 && resting.speed == 0,
			"Released preload remains intact until measured recontact and start");
		for (i in 0...100) if (finalPass.reading.result == ScrewdriverContract.NONE) {
			var observed = finalPass.step(input(original+i*0.01));
		}
		check(finalPass.reading.result == ScrewdriverContract.OK && Math.abs(finalPass.reading.torqueNm-9.9)<1e-9 &&
			Math.abs(finalPass.reading.angle-original-0.4)<1e-9 && Math.abs(finalPass.advanceMm-1.8)<1e-9,
			"Final window stays relative to the original snug point and adds only missing elastic rotation");
		var misplaced = snug().retighten(ScrewdriverProcessTests.program(),ScrewdriverProcessTests.joint());
		var sample = input(misplaced.reading.angle); sample.radialErrorMm = 1;
		check(misplaced.step(sample).result == ScrewdriverContract.NO_START,"Final requires actual aligned recontact");
		var uncompressed = snug().retighten(ScrewdriverProcessTests.program(),ScrewdriverProcessTests.joint());
		sample = input(uncompressed.reading.angle); sample.strokeMm = 0;
		check(uncompressed.step(sample).result == ScrewdriverContract.NO_START,"Final rechecks measured feed force");
		var lost = snug().retighten(ScrewdriverProcessTests.program(),ScrewdriverProcessTests.joint());
		var retainedAngle = lost.reading.angle; sample = input(retainedAngle); sample.vacuum = false;
		check(lost.step(sample).result == ScrewdriverContract.SCREW_LOST && lost.reading.angle == retainedAngle,
			"Lost final-pass vacuum preserves the full angle needed for recovery");
		var window = ScrewdriverProcessTests.program().settings; window.minimumAngle = 0.55;
		var outside = snug().retighten(new TighteningProgram(window),ScrewdriverProcessTests.joint());
		var from = outside.reading.angle;
		for (i in 0...100) if (outside.reading.result == ScrewdriverContract.NONE) {
			var observed = outside.step(input(from+i*0.01));
		}
		check(outside.reading.result == ScrewdriverContract.ANGLE_OUT_OF_WINDOW,
			"Final checks its qualified window from original snug, not from recontact");
		var empty = new TighteningModel(ScrewdriverProcessTests.program(),ScrewdriverProcessTests.joint());
		rejects(() -> {var invalid = empty.retighten(ScrewdriverProcessTests.program(),ScrewdriverProcessTests.joint());});
		var retained = snug(), changed = ScrewdriverProcessTests.joint(); changed.stiffnessNmPerRad += 1;
		rejects(() -> {var invalid = retained.retighten(ScrewdriverProcessTests.program(),changed);});
		var lower = ScrewdriverProcessTests.program().settings; lower.targetTorque = 2; lower.minimumAngle = 0;
		rejects(() -> {var invalid = retained.retighten(new TighteningProgram(lower),ScrewdriverProcessTests.joint());});
		var shortened = ScrewdriverProcessTests.program().settings; shortened.maximumRunDownAngle = 1;
		rejects(() -> {var invalid = retained.retighten(new TighteningProgram(shortened),ScrewdriverProcessTests.joint());});
		Sys.println('Screw retightening: $count checks passed');
		runHeadSeat();
	}
	static function runHeadSeat():Void {
		count = 0;
		var qualification = ScrewdriverProcessTests.program();
		var first = new TighteningModel(new TighteningProgram(qualification.settings,true),ScrewdriverProcessTests.joint());
		// Cross the seat and final thresholds in one sample: snug must still stop
		// at the calibrated seat without adding a fabricated elastic preload.
		var start = input(0); start.axialErrorMm = 0.2;
		var engaged = first.step(start);
		var stopped = first.step(input(100));
		check(stopped.result == ScrewdriverContract.OK && Math.abs(first.advanceMm-1.8)<1e-9 &&
			Math.abs(stopped.angle-1.8*2*Math.PI)<1e-9 && stopped.torqueNm == first.joint.prevailingTorque,
			"Snug stops at the measured head seat even when one sample crosses final thresholds");
		check(first.program.settings.targetTorque == qualification.settings.targetTorque &&
			first.program.settings.minimumAngle == qualification.settings.minimumAngle,
			"Snug retains the final qualification rather than inventing a lower torque target");
		var finalPass = first.retighten(qualification,ScrewdriverProcessTests.joint());
		var finished = finalPass.step(input(100));
		var elastic = (qualification.settings.targetTorque-first.joint.prevailingTorque)/first.joint.stiffnessNmPerRad;
		check(finished.result == ScrewdriverContract.OK && finished.torqueNm == qualification.settings.targetTorque &&
			Math.abs(finished.angle-stopped.angle-elastic)<1e-9 && finalPass.advanceMm == first.advanceMm,
			"Final resumes the original head seat and applies the qualified elastic rotation once");
		rejects(() -> {var invalid = first.retighten(new TighteningProgram(qualification.settings,true),ScrewdriverProcessTests.joint());});
		var damaged = new TighteningModel(new TighteningProgram(qualification.settings,true),ScrewdriverProcessTests.joint());
		var bad = input(100); bad.damagedLeadIn = true;
		check(damaged.step(bad).result == ScrewdriverContract.CROSS_THREAD,
			"Snug still rejects a damaged lead-in before the seat");
		var limited = ScrewdriverProcessTests.program().settings; limited.maximumRunDownAngle = 1;
		var tooLong = new TighteningModel(new TighteningProgram(limited,true),ScrewdriverProcessTests.joint());
		check(tooLong.step(input(100)).result == ScrewdriverContract.RUN_DOWN_TOO_LONG,
			"Snug retains the qualified run-down watchdog");
		Sys.println('Screw head-seat stop: $count checks passed');
	}
}
