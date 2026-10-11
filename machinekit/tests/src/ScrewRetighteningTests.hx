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
	}
}
