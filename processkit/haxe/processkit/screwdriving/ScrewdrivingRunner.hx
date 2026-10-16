package processkit.screwdriving;

import robotkit.tool.ScrewdriverContract;
import robotkit.spatial.Transform3;

/** A measured result belongs to the fastening, including failed pickup and engagement. */
typedef TighteningResult = {
  var fastening:String;
  var torqueNm:Float;
  /** Unwrapped cycle angle, in radians; not an angle inferred from target torque. */
  var angle:Float;
  var outcome:Int;
  /** Process diagnostic such as a bounded force-search failure. */
  @:optional var diagnostic:Null<String>;
};
private enum ScrewPhase {
  Idle; ToolReceipt; PickupApproach; PickupContact; Sealing; PickupRetract;
  WorkApproach; EntrySearch; EntrySearchStopping; EntrySearchRecoveryRetraction;
  Engage; Driving; Stopping; Releasing; Retracting; Done; Failed;
  FinalApproach; FinalContact; FinalSealing; FinalEngage; ResumeSealing;
  ProcessStop; RecoveryStop; Reversing; ReverseStop; RecoveryRetract; RejectTravel; RejectRelease; RejectRetract; Recovered;
}

/** Picks, drives and releases one screw using position motion and measured tool feedback. */
class ScrewdrivingRunner {
  public final motion:ScrewdrivingMotion;
  public final outputs:ScrewdriverOutputs;
  final present:Void -> Bool;
  final entrySearchFactory:Null<ScrewPlan -> Null<ScrewEntrySearch>>;
  var entrySearch:Null<ScrewEntrySearch>;
  var phase:ScrewPhase = Idle;
  var plan:Null<ScrewPlan> = null;
  var elapsed = 0.0;
  var followDepth = 0.0;
  var searchedEntry:Null<Transform3> = null;
  var entrySearchStartPending = false;
  var entrySearchFailure:Null<String> = null;
  var entrySearchHeld = false;
  var failureMessage:Null<String> = null;
  var reject:Null<Transform3> = null;
  var reverseAngle = 0.0;
  var previousReverseAngle = 0.0;
  var recoveryHeld = false;
  var lossAtPickup = false;
  var interruptedDriving = false;
  static inline var STOP_SPEED:Float = 0.01;
  static inline var ANGLE_RESOLUTION:Float = 1e-6;
  public var tighteningResult(default, null):Null<TighteningResult> = null;

  public function new(motion:ScrewdrivingMotion, outputs:ScrewdriverOutputs, present:Void -> Bool,
	  ?entrySearchFactory:ScrewPlan -> Null<ScrewEntrySearch>) {
    if (motion == null || outputs == null || present == null) throw "Screw runner needs arm motion, tool outputs and presenter presence";
    this.motion = motion; this.outputs = outputs; this.present = present; this.entrySearchFactory = entrySearchFactory;
  }
  public function running():Bool return phase != Idle && phase != Done && phase != Failed && phase != Recovered;
  /** True after the work approach, while the force-search handoff can be prepared. */
  public function searchingForEntry():Bool return phase == EntrySearch;
  /** The force-controlled arm owns axial motion during search and its recovery. */
  public function forceSearchOwnsAxialMotion():Bool return phase == EntrySearch || phase == EntrySearchStopping ||
		phase == EntrySearchRecoveryRetraction;
  public function completed():Bool return phase == Done;
  public function recoveryCompleted():Bool return phase == Recovered;
  public function failure():Null<String> return failureMessage;
  /** Only an interrupted drive retains enough process state to resume in place. */
  public function canResume():Bool return interruptedDriving && phase == Failed && plan != null && tighteningResult == null;
  public function resume():Void {
    if (!canResume()) throw "Screw resume needs an interrupted, unfinished drive";
    var selected:ScrewPlan = cast plan;
    motion.resetAfterSafety();
    outputs.start(false); outputs.reverse(false); outputs.selectProgram(selected.slot); outputs.vacuum(true);
    interruptedDriving = false; failureMessage = null; elapsed = 0; phase = ResumeSealing;
  }
  public function run(requested:ScrewPlan):Void {
    if (running() || requested == null) throw "Screw runner needs an idle runner and qualified plan";
    interruptedDriving = false;
    var entrySearch = entrySearchFactory == null ? null : entrySearchFactory(requested);
    plan = requested; tighteningResult = null; failureMessage = null; elapsed = 0; followDepth = 0;
    this.entrySearch = entrySearch;
    searchedEntry = null; entrySearchStartPending = false; entrySearchFailure = null; entrySearchHeld = false; lossAtPickup = false;
    outputs.start(false); outputs.reverse(false); outputs.vacuum(requested.finalContact == null && requested.toolFed); outputs.selectProgram(requested.slot);
    var contact = requested.finalContact;
    if (contact == null && requested.toolFed) {
      phase = ToolReceipt;
    } else if (contact == null) {
      phase = PickupApproach;
      motion.move(ScrewPlan.along(requested.pickupPose(),-requested.approach),false);
    } else {
      phase = FinalApproach;
      motion.move(ScrewPlan.along(contact.socket,-requested.approach),false);
    }
  }
  /** Recover a process NOK with the same held body and program; motion faults are not recoverable here. */
  public function recover(reject:Transform3):Void {
    var result = tighteningResult;
    if (phase != Failed || result == null || result.outcome == ScrewdriverContract.OK || reject == null)
      throw "Screw recovery needs a failed tightening result and a qualified reject pose";
    if (motion.failure() != null) throw "Screw recovery cannot dismiss an arm motion fault";
    this.reject = reject; failureMessage = null; elapsed = 0;
    reverseAngle = result.angle; previousReverseAngle = result.angle;
    outputs.start(false); outputs.reverse(false); phase = RecoveryStop;
  }
  public function update(durationSeconds:Float, reading:ScrewdriverReading):Void {
    if (!running()) return;
    if (!Math.isFinite(durationSeconds) || durationSeconds <= 0 || reading == null) throw "Screw runner needs positive time and feedback";
    var selected = plan;
    if (selected == null) throw "Running screwdriver has no plan";
    try {
      elapsed += durationSeconds;
      motion.update(durationSeconds);
      var fault = motion.failure();
      if (fault != null) throw fault;
      switch phase {
        case ToolReceipt:
          if (present() && reading.vacuum) {phase = WorkApproach; motion.move(ScrewPlan.along(selected.entry,-selected.approach),false);}
          else if (elapsed > 2) finishFailure(reading,ScrewdriverContract.NO_SCREW);
        case ResumeSealing:
          if (reading.vacuum) {
            // Restore feed only after the safety-cleared tool confirms its actual seal.
            motion.move(ScrewPlan.along(driveEntry(selected),followDepth),true);
            phase = FinalEngage; elapsed = 0;
          } else if (elapsed > 2) finishFailure(reading,ScrewdriverContract.NO_SCREW);
        case FinalApproach:
          if (motion.completed()) {
            var contact = selected.finalContact;
            if (contact == null) throw "Final approach lost its installed socket";
            phase = FinalContact; motion.move(contact.socket,true);
          }
        case FinalContact:
          if (motion.completed()) {outputs.vacuum(true); phase = FinalSealing; elapsed = 0;}
        case FinalSealing:
          if (reading.vacuum) {
            var contact = selected.finalContact;
            if (contact == null) throw "Final recontact lost its retained advancement";
            followDepth = contact.advanceMm/1000+selected.strokeMm/1000;
            phase = FinalEngage; motion.move(ScrewPlan.along(contact.socket,selected.strokeMm/1000),true);
          } else if (elapsed > 2) finishFailure(reading,ScrewdriverContract.NO_START);
        case FinalEngage:
          if (!reading.vacuum) {finishFailure(reading,ScrewdriverContract.SCREW_LOST); return;}
          if (motion.completed()) {outputs.start(true); phase = Driving; elapsed = 0;}
        case PickupApproach:
          if (motion.completed()) { phase = PickupContact; motion.move(selected.pickupPose(),true); }
        case PickupContact:
          if (motion.completed()) {
            if (!present()) { finishFailure(reading,ScrewdriverContract.NO_SCREW); return; }
            outputs.vacuum(true); phase = Sealing; elapsed = 0;
          }
        case Sealing:
          if (reading.vacuum) {
            phase = PickupRetract;
            motion.move(ScrewPlan.along(selected.pickupPose(),-selected.approach),true);
          } else if (elapsed > 2) finishFailure(reading,ScrewdriverContract.NO_SCREW);
        case PickupRetract:
          if (!reading.vacuum) { finishFailure(reading,ScrewdriverContract.SCREW_LOST); return; }
          if (motion.completed()) { phase = WorkApproach; motion.move(ScrewPlan.along(selected.entry,-selected.approach),false); }
        case WorkApproach:
          if (!reading.vacuum) { finishFailure(reading,ScrewdriverContract.SCREW_LOST); return; }
          if (motion.completed()) {
            if (entrySearch != null) {phase = EntrySearch; entrySearchStartPending = true;}
            else {phase = Engage; motion.move(ScrewPlan.along(selected.entry,selected.strokeMm/1000),true);}
          }
        case EntrySearch:
          var search:ScrewEntrySearch = cast entrySearch;
          if (entrySearchStartPending) {
            entrySearchStartPending = false;
            search.start();
            return;
          }
          search.update(reading.vacuum);
          var searchFailure = search.failureReason();
          if (searchFailure != null) {
            if (search.running()) {phase = EntrySearchStopping; elapsed = 0;}
            else beginEntrySearchRecovery(selected, searchFailure, reading.vacuum);
          } else if (search.isComplete()) {
            searchedEntry = search.entryPose();
            var located = searchedEntry;
            if (located == null) throw "Force search completed without its stopped measured entry pose";
            motion.synchronize();
            phase = Engage; motion.move(ScrewPlan.along(located,selected.strokeMm/1000),true);
          }
        case EntrySearchStopping:
          var search:ScrewEntrySearch = cast entrySearch;
          search.update(reading.vacuum);
          if (!search.running()) {
            var reason = search.failureReason();
            beginEntrySearchRecovery(selected, reason == null
              ? "Force search stopped without a measured entry" : reason, reading.vacuum);
          } else if (elapsed > 2) throw "Arm did not stop after force-search failure";
        case EntrySearchRecoveryRetraction:
          if (motion.completed()) {
            var reason = entrySearchFailure;
            if (reason == null) throw "Force-search recovery lost its failure diagnostic";
            var outcome = entrySearchHeld ? ScrewdriverContract.NO_START : ScrewdriverContract.SCREW_LOST;
            tighteningResult = {fastening:selected.fastening,torqueNm:0.0,angle:0.0,outcome:outcome,diagnostic:reason};
            fail('Force search: $reason');
          }
        case Engage:
          if (motion.completed()) { followDepth = selected.strokeMm/1000; outputs.start(true); phase = Driving; elapsed = 0; }
        case Driving:
          if (reading.result != ScrewdriverContract.NONE) {
            outputs.start(false); outputs.reverse(false);
            tighteningResult = {fastening:selected.fastening,torqueNm:reading.torqueNm,angle:reading.angle,outcome:reading.result};
            phase = Stopping; elapsed = 0;
          } else {
            if (elapsed > selected.program.settings.maximumRunDownAngle/selected.program.settings.runDownSpeed+
                selected.program.settings.maximumAngle/selected.program.settings.finalSpeed+10)
              throw 'Fastening "${selected.fastening}" tightening result timed out';
            // Follow measured compression rather than inferring advance from cycle
            // angle: slipping and elastic final rotation cannot move the arm forward.
            var correction = (selected.strokeMm-reading.strokeMm)/1000;
            var band = Math.min(0.0005,(selected.program.settings.maximumStroke-selected.program.settings.minimumStroke)/4000);
            if (motion.completed() && correction > band) {
              followDepth += correction;
              motion.move(ScrewPlan.along(driveEntry(selected),followDepth),true);
            }
          }
        case Stopping:
          if (Math.abs(reading.speed) <= STOP_SPEED && motion.completed()) {
            var result = tighteningResult;
            if (result == null) throw "Stopped screwdriver has no result";
            if (result.outcome != ScrewdriverContract.OK) { fail(resultMessage(result.outcome)); return; }
            outputs.vacuum(false); phase = Releasing; elapsed = 0;
          } else if (elapsed > 2) throw 'Fastening "${selected.fastening}" spindle did not stop';
        case Releasing:
          if (!reading.vacuum) { phase = Retracting; motion.move(ScrewPlan.along(driveEntry(selected),-selected.approach),true); }
          else if (elapsed > 2) throw 'Fastening "${selected.fastening}" vacuum did not release';
        case Retracting:
          if (motion.completed()) phase = Done;
        case ProcessStop:
          if (motion.stopped()) {
            var result = tighteningResult;
            if (result == null) throw "Stopped arm lost its process failure result";
            phase = Failed; failureMessage = 'Fastening "${selected.fastening}": ${resultMessage(result.outcome)}';
          } else if (elapsed > 2) throw "Arm did not stop after screw transport loss";
        case RecoveryStop:
          if (Math.abs(reading.speed) <= STOP_SPEED) {
            var result = tighteningResult;
            if (result == null) throw "Recovery lost its tightening result";
            recoveryHeld = reading.vacuum;
            if (!reading.vacuum) {
              if (result.outcome == ScrewdriverContract.NO_START) {
                result = {fastening:result.fastening,torqueNm:result.torqueNm,angle:result.angle,
                  outcome:ScrewdriverContract.SCREW_LOST,diagnostic:result.diagnostic};
                tighteningResult = result;
              }
              if (result.outcome != ScrewdriverContract.NO_SCREW && result.outcome != ScrewdriverContract.SCREW_LOST)
                throw "Failed screw lost vacuum before recovery";
              phase = RecoveryRetract;
              var from = !selected.toolFed && (result.outcome == ScrewdriverContract.NO_SCREW || lossAtPickup) ? selected.pickupPose() : driveEntry(selected);
              motion.move(ScrewPlan.along(from,-selected.approach),result.outcome != ScrewdriverContract.SCREW_LOST);
            } else if (result.outcome == ScrewdriverContract.NO_START) {
              outputs.start(false); outputs.reverse(false);
              phase = RecoveryRetract;
              motion.move(ScrewPlan.along(selected.entry,-selected.approach),false);
            } else {
              if (!Math.isFinite(reading.angle) || reading.angle < 0) throw "Stopped recovery spindle has no valid measured angle";
              reverseAngle = reading.angle; previousReverseAngle = reading.angle;
              outputs.reverse(true); outputs.start(true); phase = Reversing; elapsed = 0;
            }
          } else if (elapsed > 2) throw "Recovery spindle did not stop";
        case Reversing:
          if (!reading.vacuum) throw "Failed screw lost vacuum while reversing";
          if (!Math.isFinite(reading.angle) || reading.angle < 0 || reading.angle > previousReverseAngle+STOP_SPEED*durationSeconds+ANGLE_RESOLUTION)
            throw 'Screw recovery angle did not reverse monotonically: ${reading.angle} after $previousReverseAngle';
          previousReverseAngle = reading.angle;
          if (reading.angle <= 1e-6) {
            outputs.start(false); outputs.reverse(false); phase = ReverseStop; elapsed = 0;
          } else {
            if (elapsed > reverseAngle/selected.program.settings.runDownSpeed+5)
              throw "Screw recovery could not reverse the measured driven angle";
            var correction = (selected.strokeMm-reading.strokeMm)/1000;
            var band = Math.min(0.0005,(selected.program.settings.maximumStroke-selected.program.settings.minimumStroke)/4000);
            if (motion.completed() && correction < -band) {
              followDepth += correction;
              motion.move(ScrewPlan.along(driveEntry(selected),followDepth),true);
            }
          }
        case ReverseStop:
          if (!reading.vacuum) throw "Failed screw lost vacuum before withdrawal";
          if (Math.abs(reading.speed) <= STOP_SPEED && motion.completed()) {
            phase = RecoveryRetract;
            motion.move(ScrewPlan.along(driveEntry(selected),-selected.approach),true);
          } else if (elapsed > 2) throw "Reversed spindle did not stop";
        case RecoveryRetract:
          if (recoveryHeld && !reading.vacuum) throw "Failed screw lost vacuum during withdrawal";
          if (motion.completed()) {
            if (!recoveryHeld) {phase = Recovered; return;}
            var target = reject;
            if (target == null) throw "Screw recovery lost its reject pose";
            motion.move(target,false); phase = RejectTravel;
          }
        case RejectTravel:
          if (!reading.vacuum) throw "Failed screw lost vacuum on the way to the reject bin";
          if (motion.completed()) {outputs.vacuum(false); phase = RejectRelease; elapsed = 0;}
        case RejectRelease:
          if (!reading.vacuum) {
            var target = reject;
            if (target == null) throw "Screw recovery lost its reject pose";
            motion.move(ScrewPlan.along(target,-selected.approach),true); phase = RejectRetract;
          } else if (elapsed > 2) throw "Reject vacuum did not release";
        case RejectRetract:
          if (motion.completed()) phase = Recovered;
        case _:
      }
    } catch (error:Dynamic) fail(Std.string(error));
  }
  public function abort():Void {
    interruptedDriving = phase == Driving && tighteningResult == null;
    outputs.start(false); outputs.reverse(false);
    if (entrySearch != null && entrySearch.running()) entrySearch.cancel();
    motion.abort(); phase = Failed;
    failureMessage = "screwdriving cancelled";
  }
  function finishFailure(reading:ScrewdriverReading, outcome:Int):Void {
    var selected = plan;
    if (selected == null) throw "Failed screwdriver has no fastening";
    tighteningResult = {fastening:selected.fastening,torqueNm:reading.torqueNm,angle:reading.angle,outcome:outcome};
    if (outcome == ScrewdriverContract.SCREW_LOST) {
      lossAtPickup = phase == PickupRetract;
      outputs.start(false); outputs.reverse(false);
      motion.stop(); phase = ProcessStop; elapsed = 0;
    } else fail(resultMessage(outcome));
  }
  static function resultMessage(outcome:Int):String {
    var message = ScrewdriverSensor.faultMessage(outcome);
    if (message == null) throw "Failed screwdriver has no failure result";
    return message;
  }
  function fail(message:String):Void {
    outputs.start(false); outputs.reverse(false);
    if (entrySearch != null && entrySearch.running()) try entrySearch.cancel() catch (_:Dynamic) {}
    try motion.abort() catch (_:Dynamic) {}
    phase = Failed;
    var selected = plan;
    failureMessage = selected == null ? message : 'Fastening "${selected.fastening}": $message';
  }

  /** Search has stopped at rest; retract the still-held screw to its qualified safe approach before failing. */
  function beginEntrySearchRecovery(selected:ScrewPlan, reason:String, screwHeld:Bool):Void {
    if (reason == null || reason.length == 0) throw "Force-search recovery needs its measured failure diagnostic";
    entrySearchFailure = reason;
    entrySearchHeld = screwHeld;
    motion.synchronize();
    phase = EntrySearchRecoveryRetraction;
    elapsed = 0;
    motion.move(ScrewPlan.along(selected.entry, -selected.approach), false);
  }

  function driveEntry(selected:ScrewPlan):Transform3 {
    return searchedEntry == null ? selected.entry : cast searchedEntry;
  }
}
