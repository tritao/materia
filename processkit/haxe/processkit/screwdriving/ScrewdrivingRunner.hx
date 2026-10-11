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
};
private enum ScrewPhase {
  Idle; PickupApproach; PickupContact; Sealing; PickupRetract;
  WorkApproach; Engage; Driving; Stopping; Releasing; Retracting; Done; Failed;
  RecoveryStop; Reversing; ReverseStop; RecoveryRetract; RejectTravel; RejectRelease; RejectRetract; Recovered;
}

/** Picks, drives and releases one screw using position motion and measured tool feedback. */
class ScrewdrivingRunner {
  public final motion:ScrewdrivingMotion;
  public final outputs:ScrewdriverOutputs;
  final present:Void -> Bool;
  var phase:ScrewPhase = Idle;
  var plan:Null<ScrewPlan> = null;
  var elapsed = 0.0;
  var followDepth = 0.0;
  var failureMessage:Null<String> = null;
  var reject:Null<Transform3> = null;
  var reverseAngle = 0.0;
  var previousReverseAngle = 0.0;
  var recoveryHeld = false;
  public var tighteningResult(default, null):Null<TighteningResult> = null;

  public function new(motion:ScrewdrivingMotion, outputs:ScrewdriverOutputs, present:Void -> Bool) {
    if (motion == null || outputs == null || present == null) throw "Screw runner needs arm motion, tool outputs and presenter presence";
    this.motion = motion; this.outputs = outputs; this.present = present;
  }
  public function running():Bool return phase != Idle && phase != Done && phase != Failed && phase != Recovered;
  public function completed():Bool return phase == Done;
  public function recoveryCompleted():Bool return phase == Recovered;
  public function failure():Null<String> return failureMessage;
  public function run(requested:ScrewPlan):Void {
    if (running() || requested == null) throw "Screw runner needs an idle runner and qualified plan";
    plan = requested; tighteningResult = null; failureMessage = null; elapsed = 0; followDepth = 0;
    outputs.start(false); outputs.reverse(false); outputs.vacuum(false); outputs.selectProgram(requested.slot);
    phase = PickupApproach;
    motion.move(ScrewPlan.along(requested.pickup,-requested.approach),false);
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
        case PickupApproach:
          if (motion.completed()) { phase = PickupContact; motion.move(selected.pickup,true); }
        case PickupContact:
          if (motion.completed()) {
            if (!present()) { finishFailure(reading,ScrewdriverContract.NO_SCREW); return; }
            outputs.vacuum(true); phase = Sealing; elapsed = 0;
          }
        case Sealing:
          if (reading.vacuum) {
            phase = PickupRetract;
            motion.move(ScrewPlan.along(selected.pickup,-selected.approach),true);
          } else if (elapsed > 2) finishFailure(reading,ScrewdriverContract.NO_SCREW);
        case PickupRetract:
          if (!reading.vacuum) { finishFailure(reading,ScrewdriverContract.SCREW_LOST); return; }
          if (motion.completed()) { phase = WorkApproach; motion.move(ScrewPlan.along(selected.entry,-selected.approach),false); }
        case WorkApproach:
          if (!reading.vacuum) { finishFailure(reading,ScrewdriverContract.SCREW_LOST); return; }
          if (motion.completed()) {
            phase = Engage; motion.move(ScrewPlan.along(selected.entry,selected.strokeMm/1000),true);
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
              motion.move(ScrewPlan.along(selected.entry,followDepth),true);
            }
          }
        case Stopping:
          if (Math.abs(reading.speed) <= 0.01 && motion.completed()) {
            var result = tighteningResult;
            if (result == null) throw "Stopped screwdriver has no result";
            if (result.outcome != ScrewdriverContract.OK) { fail(resultMessage(result.outcome)); return; }
            outputs.vacuum(false); phase = Releasing; elapsed = 0;
          } else if (elapsed > 2) throw 'Fastening "${selected.fastening}" spindle did not stop';
        case Releasing:
          if (!reading.vacuum) { phase = Retracting; motion.move(ScrewPlan.along(selected.entry,-selected.approach),true); }
          else if (elapsed > 2) throw 'Fastening "${selected.fastening}" vacuum did not release';
        case Retracting:
          if (motion.completed()) phase = Done;
        case RecoveryStop:
          if (Math.abs(reading.speed) <= 0.01) {
            var result = tighteningResult;
            if (result == null) throw "Recovery lost its tightening result";
            recoveryHeld = reading.vacuum;
            if (!reading.vacuum) {
              if (result.outcome != ScrewdriverContract.NO_SCREW && result.outcome != ScrewdriverContract.SCREW_LOST)
                throw "Failed screw lost vacuum before recovery";
              phase = RecoveryRetract;
              motion.move(ScrewPlan.along(result.outcome == ScrewdriverContract.NO_SCREW ? selected.pickup : selected.entry,-selected.approach),true);
            } else {
              outputs.reverse(true); outputs.start(true); phase = Reversing; elapsed = 0;
            }
          } else if (elapsed > 2) throw "Recovery spindle did not stop";
        case Reversing:
          if (!reading.vacuum) throw "Failed screw lost vacuum while reversing";
          if (!Math.isFinite(reading.angle) || reading.angle < 0 || reading.angle > previousReverseAngle+1e-5)
            throw "Screw recovery angle did not reverse monotonically";
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
              motion.move(ScrewPlan.along(selected.entry,followDepth),true);
            }
          }
        case ReverseStop:
          if (!reading.vacuum) throw "Failed screw lost vacuum before withdrawal";
          if (Math.abs(reading.speed) <= 0.01 && motion.completed()) {
            phase = RecoveryRetract;
            motion.move(ScrewPlan.along(selected.entry,-selected.approach),true);
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
    outputs.start(false); outputs.reverse(false); motion.abort(); phase = Failed;
    failureMessage = "screwdriving cancelled";
  }
  function finishFailure(reading:ScrewdriverReading, outcome:Int):Void {
    var selected = plan;
    if (selected == null) throw "Failed screwdriver has no fastening";
    tighteningResult = {fastening:selected.fastening,torqueNm:reading.torqueNm,angle:reading.angle,outcome:outcome};
    fail(resultMessage(outcome));
  }
  static function resultMessage(outcome:Int):String {
    var message = ScrewdriverSensor.faultMessage(outcome);
    if (message == null) throw "Failed screwdriver has no failure result";
    return message;
  }
  function fail(message:String):Void {
    outputs.start(false); outputs.reverse(false);
    try motion.abort() catch (_:Dynamic) {}
    phase = Failed;
    var selected = plan;
    failureMessage = selected == null ? message : 'Fastening "${selected.fastening}": $message';
  }
}
