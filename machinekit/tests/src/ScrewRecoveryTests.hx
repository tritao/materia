import processkit.screwdriving.*;
import robotkit.spatial.*;
import robotkit.tool.ScrewdriverContract;
import robotkit.skill.*;
import robotkit.core.RobotSnapshot;
import processkit.skill.DriveFastening;
import processkit.screwdriving.FasteningRunReport.ScrewDisposition;
import processkit.screwdriving.FasteningRunReport.FasteningPass;

private class RecoveryOutputs implements ScrewdriverOutputs {
	public var startOn = false;
	public var reverseOn = false;
	public var vacuumOn = false;
	public function new() {}
	public function selectProgram(slot:Int):Void {}
	public function start(on:Bool):Void startOn = on;
	public function reverse(on:Bool):Void reverseOn = on;
	public function vacuum(on:Bool):Void vacuumOn = on;
}
private class RecoveryMotion implements ScrewdrivingMotion {
	public final targets:Array<Transform3> = [];
	public final contacts:Array<Bool> = [];
	public var problem:Null<String> = null;
	var done = false;
	var idle = true;
	var stopping = false;
	public var stopWait = 0;
	public var stopRequests = 0;
	public var synchronizations = 0;
	public function new() {}
	public function move(target:Transform3, contact:Bool):Void {targets.push(target); contacts.push(contact); done = false; idle = false;}
	public function update(dt:Float):Void {
		if (stopping) {if (stopWait-- <= 0) {stopping = false; idle = true;} }
		else {done = true; idle = true;}
	}
	public function completed():Bool return done;
	public function failure():Null<String> return problem;
	public function synchronize():Void synchronizations++;
	public function stop():Void {stopRequests++; stopping = true; done = false; idle = false;}
	public function stopped():Bool return idle && problem == null;
	public function abort():Void {done = false; idle = false;}
}
private class FailedEntrySearch implements ScrewEntrySearch {
	public var problem:Null<String> = null;
	var active = false;
	public function new() {}
	public function start():Void active = true;
	public function update(screwHeld:Bool):Void {active = false; problem = screwHeld ? "registered spiral exhausted" : "screwdriver lost the held screw during force search";}
	public function running():Bool return active;
	public function isComplete():Bool return false;
	public function failureReason():Null<String> return problem;
	public function entryPose():Null<Transform3> return null;
	public function cancel():Void active = false;
}

private class AttemptSkillFixture implements Skill {
	final runner:ScrewdrivingRunner;
	public function new(runner:ScrewdrivingRunner) this.runner = runner;
	public function start():Void {}
	public function update(snapshot:RobotSnapshot, dt:Float):SkillStatus return status();
	public function cancel():Void runner.abort();
	public function status():SkillStatus {
		var failure = runner.failure();
		return failure != null ? Failed(failure) : (runner.completed() ? Succeeded : Running);
	}
	public function result():Null<SkillResult> return null;
}
private class RecoverySkillFixture implements Skill {
	final runner:ScrewdrivingRunner;
	final outputs:RecoveryOutputs;
	var angle = 0.3;
	public function new(runner:ScrewdrivingRunner, outputs:RecoveryOutputs) {this.runner = runner; this.outputs = outputs;}
	public function start():Void runner.recover(new Transform3(new Vec3(0.4,0.1,0.3),Quat.identity()));
	public function update(snapshot:RobotSnapshot, dt:Float):SkillStatus {
		if (outputs.reverseOn) angle = Math.max(0,angle-0.1);
		var feedback = ScrewdriverSensor.idle(); feedback.angle = angle; feedback.vacuum = outputs.vacuumOn;
		feedback.strokeMm = 8; runner.update(dt,feedback);
		return status();
	}
	public function cancel():Void runner.abort();
	public function status():SkillStatus {
		var failure = runner.failure();
		return failure != null ? Failed(failure) : (runner.recoveryCompleted() ? Succeeded : Running);
	}
	public function result():Null<SkillResult> return null;
}

/** Recovery follows measured reverse feedback and never promotes a rejected tightening to OK. */
class ScrewRecoveryTests {
	static var count = 0;
	static function check(value:Bool, message:String):Void {count++; if (!value) throw message;}
	static function refuses(run:Void -> Void):Void {
		var refused = false; try run() catch (_:Dynamic) refused = true;
		check(refused,"Invalid screw recovery must be refused");
	}
	static function plan():ScrewPlan return new ScrewPlan("recovery-probe",1,ScrewdriverProcessTests.program(),
		new Transform3(new Vec3(0.1,0,0.2),Quat.identity()),new Transform3(new Vec3(0.2,0,0.2),Quat.identity()),1,2);
	static function failed(motion:RecoveryMotion, outputs:RecoveryOutputs):ScrewdrivingRunner {
		var runner = new ScrewdrivingRunner(motion,outputs,() -> true), request = plan();
		runner.run(request);
		var feedback = ScrewdriverSensor.idle(); feedback.strokeMm = request.strokeMm;
		for (_ in 0...20) if (!outputs.startOn) {
			feedback.vacuum = outputs.vacuumOn; runner.update(0.01,feedback);
		}
		feedback.vacuum = true; feedback.result = ScrewdriverContract.CROSS_THREAD; feedback.angle = 0.3; feedback.torqueNm = 1;
		runner.update(0.01,feedback); runner.update(0.01,feedback);
		return runner;
	}
	static function reading(angle:Float):ScrewdriverReading {
		var result = ScrewdriverSensor.idle(); result.vacuum = true; result.angle = angle;
		result.strokeMm = plan().strokeMm; result.result = ScrewdriverContract.CROSS_THREAD;
		return result;
	}
	public static function run():Void {
		count = 0;
		runEntrySearchFailure();
		var motion = new RecoveryMotion(), outputs = new RecoveryOutputs(), runner = failed(motion,outputs);
		var reject = new Transform3(new Vec3(0.4,0.1,0.3),Quat.identity());
		check(runner.failure() != null && outputs.vacuumOn && !outputs.startOn,"Failed tightening retains its screw with spindle stopped");
		refuses(() -> runner.recover(null));
		runner.recover(reject);
		var feedback = reading(0.3); feedback.speed = 1;
		runner.update(0.01,feedback);
		check(!outputs.startOn && !outputs.reverseOn && outputs.vacuumOn,"Recovery waits for measured spindle stop before reversing");
		feedback.speed = 0; feedback.angle = 0.31; runner.update(0.01,feedback);
		check(outputs.startOn && outputs.reverseOn && outputs.vacuumOn,"Recovery reverses the same held screw after stop");
		feedback.angle = 0.305; runner.update(0.01,feedback);
		check(runner.failure() == null && (@:privateAccess runner.previousReverseAngle) == 0.305,"Reversal starts from the fresh stopped angle while retaining the original NOK result");
		feedback.angle = 0.2; feedback.strokeMm += 1; runner.update(0.01,feedback);
		check(Math.abs(motion.targets[motion.targets.length-1].translation.z-(plan().entry.translation.z+(plan().strokeMm-1)/1000)) < 1e-9,
			"Reverse withdrawal follows measured compression backwards");
		feedback.angle = 0; feedback.speed = -1; runner.update(0.01,feedback);
		check(!outputs.startOn && !outputs.reverseOn && outputs.vacuumOn,"Measured zero ends reverse without releasing the screw");
		var beforeStop = motion.targets.length; runner.update(0.01,feedback);
		check(motion.targets.length == beforeStop,"Withdrawal waits for actual reverse spindle stop");
		feedback.speed = 0; runner.update(0.01,feedback);
		check(Math.abs(motion.targets[motion.targets.length-1].translation.z-(plan().entry.translation.z-plan().approach)) < 1e-9,
			"A backed-out screw retracts on the qualified work axis");
		runner.update(0.01,feedback);
		check(motion.targets[motion.targets.length-1] == reject && outputs.vacuumOn,"The actual held screw travels to the qualified reject pose");
		runner.update(0.01,feedback);
		check(!outputs.vacuumOn && !runner.recoveryCompleted(),"Vacuum releases only after reaching the reject bin");
		feedback.vacuum = false; runner.update(0.01,feedback); runner.update(0.01,feedback);
		var original = runner.tighteningResult;
		check(runner.recoveryCompleted() && !runner.completed() && original != null && original.outcome == ScrewdriverContract.CROSS_THREAD && original.angle == 0.3,
			"Recovery preserves the original NOK and measured driven angle rather than claiming a tightened screw");
		refuses(() -> runner.recover(reject));
		var risingOutputs = new RecoveryOutputs(), rising = failed(new RecoveryMotion(),risingOutputs);
		rising.recover(reject); rising.update(0.01,reading(0.3)); rising.update(0.01,reading(0.4));
		check(rising.failure() != null && !risingOutputs.startOn && risingOutputs.vacuumOn,"An increasing reverse angle stops recovery and retains vacuum");
		var lostOutputs = new RecoveryOutputs(), lost = failed(new RecoveryMotion(),lostOutputs);
		lost.recover(reject); lost.update(0.01,reading(0.3)); var noVacuum = reading(0.2); noVacuum.vacuum = false;
		lost.update(0.01,noVacuum);
		check(lost.failure() != null && !lostOutputs.startOn,"Losing vacuum while reversing is a recovery fault");
		var stuckOutputs = new RecoveryOutputs(), stuck = failed(new RecoveryMotion(),stuckOutputs);
		stuck.recover(reject); stuck.update(0.01,reading(0.3)); stuck.update(6,reading(0.3));
		check(stuck.failure() != null && !stuckOutputs.startOn,"Reverse has a watchdog based on the measured driven angle");
		var faultMotion = new RecoveryMotion(), faulted = failed(faultMotion,new RecoveryOutputs()); faultMotion.problem = "Arm planner fault";
		refuses(() -> faulted.recover(reject));
		var emptyOutputs = new RecoveryOutputs(), emptyMotion = new RecoveryMotion();
		var empty = new ScrewdrivingRunner(emptyMotion,emptyOutputs,() -> false); empty.run(plan());
		for (_ in 0...10) if (empty.running()) empty.update(0.01,ScrewdriverSensor.idle());
		empty.recover(reject); empty.update(0.01,ScrewdriverSensor.idle()); empty.update(0.01,ScrewdriverSensor.idle());
		check(empty.recoveryCompleted() && !emptyOutputs.startOn && emptyMotion.targets.length == 3,"An empty nest retracts without reversing or inventing a rejected body");
		runTransportLoss(true);
		runTransportLoss(false);
		runReplacementSkill(false);
		runReplacementSkill(true);
		runOrchestrationFaults();

		Sys.println('Screw recovery: $count checks passed');
		runRemainingBudget();
	}
	static function runEntrySearchFailure():Void {
		var motion = new RecoveryMotion(), outputs = new RecoveryOutputs(), search = new FailedEntrySearch();
		var request = plan(), runner = new ScrewdrivingRunner(motion, outputs, () -> true, (_) -> search);
		runner.run(request);
		var feedback = ScrewdriverSensor.idle(); feedback.strokeMm = request.strokeMm;
		for (_ in 0...12) if (search.problem == null) {
			feedback.vacuum = outputs.vacuumOn;
			runner.update(0.01, feedback);
		}
		check(search.problem == "registered spiral exhausted" && runner.running() && outputs.vacuumOn &&
			!outputs.startOn && runner.tighteningResult == null,
			"A failed force search stops tightening while retaining its measured screw");
		feedback.vacuum = true; runner.update(0.01, feedback);
		var safe = ScrewPlan.along(request.entry, -request.approach);
		var retreat = motion.targets[motion.targets.length - 1];
		var result = runner.tighteningResult;
		check(runner.failure() != null && runner.failure().indexOf("registered spiral exhausted") >= 0 &&
			motion.synchronizations == 1 &&
			retreat.translation.x == safe.translation.x && retreat.translation.z == safe.translation.z &&
			!motion.contacts[motion.contacts.length - 1] && outputs.vacuumOn && !outputs.startOn && result != null &&
			result.outcome == ScrewdriverContract.NO_START && result.diagnostic == "registered spiral exhausted",
			"A stopped force-search failure retracts safely and records a diagnostic no-start result");
		var reject = new Transform3(new Vec3(0.4,0.1,0.3),Quat.identity());
		runner.recover(reject); feedback.angle = 0; feedback.speed = 0; runner.update(0.01, feedback);
		var recoveryApproach = motion.targets[motion.targets.length-1];
		check(!outputs.startOn && !outputs.reverseOn && outputs.vacuumOn &&
			recoveryApproach.translation.x == safe.translation.x && recoveryApproach.translation.z == safe.translation.z,
			"No-start recovery skips spindle reversal and returns to its safe approach");
		runner.update(0.01, feedback);
		check(motion.targets[motion.targets.length-1] == reject && outputs.vacuumOn,
			"The held screw travels to the reject bin only after safe retraction");
		runner.update(0.01, feedback);
		check(!outputs.vacuumOn && !runner.recoveryCompleted(),"No-start recovery releases vacuum only at the reject bin");
		feedback.vacuum = false; runner.update(0.01, feedback); runner.update(0.01, feedback);
		var finalResult = runner.tighteningResult;
		check(runner.recoveryCompleted() && !runner.completed() && finalResult != null &&
			finalResult.outcome == ScrewdriverContract.NO_START && finalResult.diagnostic == "registered spiral exhausted",
			"Recovered no-start remains an identified failed attempt and never reports a tightened screw");
		var report = new FasteningRunReport(request.fastening);
		report.record(0, "force-search-screw", cast(result)); report.recovered(); report.finish();
		check(report.failed && report.attempts[0].disposition == Rejected &&
			report.attempts[0].result.diagnostic == "registered spiral exhausted",
			"Fastening accounting retains the force-search diagnostic and rejected screw identity");
	}
	static function runRemainingBudget():Void {
		count = 0;
		var created = 0, outputs = new RecoveryOutputs();
		var attempt:Int->processkit.skill.DriveFastening.ScrewAttempt = (index) -> {
			if (index != 0 || created != 0) throw "Final pass requested a supply after exhausting its shared budget";
			created++;
			var drive = failed(new RecoveryMotion(),outputs);
			return {screw:"installed-reserve",runner:drive,skill:new AttemptSkillFixture(drive)};
		};
		var recover:ScrewdrivingRunner->Skill = (drive) -> new RecoverySkillFixture(drive,outputs);
		refuses(() -> {var invalid = new DriveFastening("recovery-probe",attempt,recover,0);});
		refuses(() -> {var invalid = new DriveFastening("recovery-probe",attempt,recover,3);});
		var execution = new DriveFastening("recovery-probe",attempt,recover,1);
		execution.start();
		for (_ in 0...100) {var status = execution.update(null,0.01);}
		check(execution.status() == Succeeded && created == 1 && execution.report.finished && execution.report.failed,
			"An exhausted supply budget recovers the installed body's NOK and continues without another attempt");
		check(execution.report.attempts.length == 1 && execution.report.attempts[0].screw == "installed-reserve" &&
			execution.report.attempts[0].result.outcome == ScrewdriverContract.CROSS_THREAD && execution.report.attempts[0].disposition == Rejected,
			"A one-attempt final run retains its exact rejected body and measured failure");
		check(execution.report.installed() == null,"Exhausted final recovery does not fabricate an installed body");
		Sys.println('Screw remaining supply budget: $count checks passed');
		runSkippedFinal();
	}
	static function runSkippedFinal():Void {
		count = 0;
		var snug = new FasteningRunReport("recovery-probe",SnugPass);
		var result:processkit.screwdriving.ScrewdrivingRunner.TighteningResult =
			{fastening:"recovery-probe",torqueNm:1.0,angle:0.3,outcome:ScrewdriverContract.CROSS_THREAD,diagnostic:null};
		snug.record(0,"original",result); snug.recovered();
		snug.record(1,"reserve",result); snug.recovered(); snug.finish();
		var finalPass = new processkit.skill.SkippedFinalFastening(snug);
		finalPass.start();
		check(finalPass.update(null,0.01) == Succeeded && finalPass.runReport().finished && finalPass.runReport().failed &&
			finalPass.runReport().pass == FinalPass && finalPass.runReport().skipped != null,
			"Failed snug explicitly skips final and lets the mission continue with its reason");
		check(finalPass.runReport().attempts.length == 0 && finalPass.runReport().installed() == null && snug.attempts.length == 2,
			"Skipping final fabricates no torque, angle, installed body or third supply");
		var pending = new FasteningRunReport("recovery-probe",SnugPass);
		refuses(() -> {var invalid = new processkit.skill.SkippedFinalFastening(pending);});
		snug.stop("controller fault");
		refuses(() -> {var invalid = new processkit.skill.SkippedFinalFastening(snug);});
		var single = new FasteningRunReport("recovery-probe");
		single.record(0,"original",result); single.recovered(); single.finish();
		refuses(() -> {var invalid = new processkit.skill.SkippedFinalFastening(single);});
		Sys.println('Screw skipped final: $count checks passed');
	}
	static function runTransportLoss(pickup:Bool):Void {
		var motion = new RecoveryMotion(), outputs = new RecoveryOutputs();
		var runner = new ScrewdrivingRunner(motion,outputs,() -> true), requested = plan();
		runner.run(requested);
		var feedback = ScrewdriverSensor.idle(); feedback.strokeMm = requested.strokeMm;
		var targetCount = pickup ? 3 : 4;
		for (_ in 0...10) if (motion.targets.length < targetCount) {
			feedback.vacuum = outputs.vacuumOn; runner.update(0.01,feedback);
		}
		motion.stopWait = 3; feedback.vacuum = false;
		runner.update(0.01,feedback);
		check(motion.stopRequests == 1 && runner.running() && runner.failure() == null,
			"Transport loss requests a controlled arm stop before finishing the failed attempt");
		refuses(() -> runner.recover(Transform3.identity()));
		for (_ in 0...3) runner.update(0.01,feedback);
		check(runner.running() && runner.failure() == null && motion.targets.length == targetCount,
			"A stop request cannot stand in for measured rest or start a recovery move");
		runner.update(0.01,feedback);
		check(!runner.running() && runner.failure() != null && motion.failure() == null && !motion.completed(),
			"Confirmed rest finishes the lost-screw attempt without fabricating an arm fault or a completed program");
		runner.recover(Transform3.identity()); runner.update(0.01,feedback);
		var from = pickup ? requested.pickupPose() : requested.entry;
		var retreat = motion.targets[motion.targets.length-1];
		check(retreat.translation.x == from.translation.x && retreat.translation.z == from.translation.z-requested.approach &&
			!motion.contacts[motion.contacts.length-1] && !outputs.reverseOn,
			"Lost-body recovery uses checked travel to the qualified retreat, without reversing an absent screw");
		runner.update(0.01,feedback);
		var result = runner.tighteningResult;
		check(runner.recoveryCompleted() && motion.targets.length == targetCount+1 && result != null && result.outcome == ScrewdriverContract.SCREW_LOST,
			"Lost-screw recovery retains the measured failure without inventing a body delivered to the bin");
		var faultyMotion = new RecoveryMotion(), faultyOutputs = new RecoveryOutputs();
		var faulty = new ScrewdrivingRunner(faultyMotion,faultyOutputs,() -> true); faulty.run(requested);
		for (_ in 0...10) if (faultyMotion.targets.length < targetCount) {
			feedback.vacuum = faultyOutputs.vacuumOn; faulty.update(0.01,feedback);
		}
		faultyMotion.stopWait = 3; feedback.vacuum = false; faulty.update(0.01,feedback);
		faultyMotion.problem = "Runtime stop rejection"; faulty.update(0.01,feedback);
		check(faulty.failure() != null && faultyMotion.failure() == "Runtime stop rejection",
			"A real fault while stopping remains an arm fault and cannot be acknowledged as recovered");
		refuses(() -> faulty.recover(Transform3.identity()));
	}
	static function runReplacementSkill(exhausted:Bool):Void {
		var created = 0;
		var devices:Array<{runner:ScrewdrivingRunner,outputs:RecoveryOutputs}> = [];
		var skill = new DriveFastening("recovery-probe",(index) -> {
			check(index == created && created < 2,"The retry factory receives exactly one replacement index"); created++;
			var device = new RecoveryOutputs(), motion = new RecoveryMotion();
			var drive:ScrewdrivingRunner;
			if (index == 0 || exhausted) drive = failed(motion,device);
			else {
				drive = new ScrewdrivingRunner(motion,device,() -> true); drive.run(plan());
				var feedback = reading(0); feedback.result = ScrewdriverContract.NONE;
				for (_ in 0...20) if (!device.startOn) {feedback.vacuum = device.vacuumOn; drive.update(0.01,feedback);}
				feedback.result = ScrewdriverContract.OK; feedback.torqueNm = 9.9; feedback.angle = 4*Math.PI+0.5;
				for (_ in 0...10) if (drive.running()) {feedback.vacuum = device.vacuumOn; drive.update(0.01,feedback);}
			}
			devices.push({runner:drive,outputs:device});
			return {screw:index == 0 ? "original" : "replacement",runner:drive,skill:new AttemptSkillFixture(drive)};
		},(drive) -> {
			var selected = [for (device in devices) if (device.runner == drive) device.outputs];
			if (selected.length != 1) throw "Recovery lost its exact attempt outputs";
			return new RecoverySkillFixture(drive,selected[0]);
		});
		skill.start();
		for (_ in 0...100) {var status = skill.update(null,0.01);}
		check(skill.status() == Succeeded && skill.report.finished && skill.report.failed == exhausted && created == 2,
			"A recovered attempt retries once and reports either OK or exhausted fastening failure without stopping the mission");
		check(skill.report.attempts.length == 2 && skill.report.attempts[0].result.outcome == ScrewdriverContract.CROSS_THREAD &&
			skill.report.attempts[0].result.angle == 0.3,"The run report preserves the original measured failed attempt");
		check(skill.report.attempts[0].screw == "original" && skill.report.attempts[1].screw == "replacement" &&
			(exhausted ? skill.report.installed() == null : skill.report.installed() == "replacement"),
			"Reports retain both supply identities and name only the body that was installed");
		var last = skill.report.attempts[1];
		check(last.result.outcome == (exhausted ? ScrewdriverContract.CROSS_THREAD : ScrewdriverContract.OK) &&
			(exhausted ? last.result.angle == 0.3 : last.result.torqueNm == 9.9),"Replacement results remain measured rather than promoted to OK");
	}

	static function runOrchestrationFaults():Void {
		var recoveries = 0;
		var motionFault = new DriveFastening("recovery-probe",(_) -> {
			var motion = new RecoveryMotion(), outputs = new RecoveryOutputs();
			var drive = new ScrewdrivingRunner(motion,outputs,() -> true); drive.run(plan());
			motion.problem = "Arm planner fault"; drive.update(0.01,ScrewdriverSensor.idle());
			return {screw:"original",runner:drive,skill:new AttemptSkillFixture(drive)};
		},(drive) -> {recoveries++; return new AttemptSkillFixture(drive);});
		motionFault.start(); var motionStatus = motionFault.update(null,0.01);
		check(motionFault.report.fault == "Fastening \"recovery-probe\": Arm planner fault" && recoveries == 0 && motionFault.report.attempts.length == 0,
			"An arm fault stops the mission without inventing a result or attempting recovery");
		var created = 0;
		var recoveryFault = new DriveFastening("recovery-probe",(_) -> {
			created++; var drive = failed(new RecoveryMotion(),new RecoveryOutputs());
			return {screw:"original",runner:drive,skill:new AttemptSkillFixture(drive)};
		},(drive) -> new AttemptSkillFixture(drive));
		recoveryFault.start(); var failedAttempt = recoveryFault.update(null,0.01); var failedRecovery = recoveryFault.update(null,0.01);
		var fault = recoveryFault.report.fault;
		check(fault != null && fault.indexOf("recovery:") == 0 && created == 1 && !recoveryFault.report.finished,
			"A recovery fault stops the mission before a replacement starts");
		check(recoveryFault.report.attempts[0].disposition == RecoveryFailed,"The report distinguishes failed recovery from a rejected screw");
		var cancelledOutputs = new RecoveryOutputs();
		var cancelled = new DriveFastening("recovery-probe",(_) -> {
			var drive = failed(new RecoveryMotion(),cancelledOutputs);
			return {screw:"original",runner:drive,skill:new AttemptSkillFixture(drive)};
		},(drive) -> new AttemptSkillFixture(drive));
		cancelled.start(); cancelled.cancel();
		check(cancelled.status() == Cancelled && cancelledOutputs.vacuumOn && !cancelledOutputs.startOn,
			"Cancellation retains the held body and stops spindle outputs");
		var report = new processkit.screwdriving.FasteningRunReport("recovery-probe");
		var result:processkit.screwdriving.ScrewdrivingRunner.TighteningResult =
			{fastening:"recovery-probe",torqueNm:1,angle:0.3,outcome:ScrewdriverContract.CROSS_THREAD};
		report.record(0,"same-body",result); report.recovered();
		refuses(() -> report.record(1,"same-body",result));

	}

	static function main():Void run();
}
