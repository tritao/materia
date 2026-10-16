package processkit.screwdriving;

import processkit.screwdriving.ScrewdrivingRunner.TighteningResult;
import robotkit.tool.ScrewdriverContract;

enum ScrewDisposition { Tightened; PendingRecovery; Rejected; Missing; Lost; RecoveryFailed; }
enum FasteningPass { SinglePass; SnugPass; FinalPass; }

typedef ScrewAttemptResult = {
  var attempt:Int;
  /** Identified supply occurrence; a replacement remains its own body after seating. */
  var screw:String;
  var result:TighteningResult;
  var disposition:ScrewDisposition;
};

/** Measured attempts and their disposition; exhausting a replacement does not become a motion fault or an OK. */
class FasteningRunReport {
  public final fastening:String;
  public final pass:FasteningPass;
  public final attempts:Array<ScrewAttemptResult> = [];
  public var finished(default, null) = false;
  public var failed(default, null) = false;
  public var fault(default, null):Null<String> = null;
  public var skipped(default, null):Null<String> = null;
  public function new(fastening:String, pass:FasteningPass = SinglePass) {
    if (fastening == null || fastening.length == 0) throw "Fastening report needs its CAD identity";
    this.fastening = fastening;
    this.pass = pass;
  }
  public function reset():Void {attempts.resize(0); finished = false; failed = false; fault = null; skipped = null;}
  /** A skipped final has a reason and no invented torque, angle or body disposition. */
  public function skip(reason:String):Void {
    if (pass != FinalPass || finished || fault != null || attempts.length != 0 || reason == null || reason.length == 0)
      throw "Only an unattempted final pass can be explicitly skipped";
    skipped = reason; finished = true; failed = true;
  }
  public function record(attempt:Int, screw:String, result:TighteningResult):Void {
    if (finished || screw == null || screw.length == 0 || result == null || result.fastening != fastening || attempt != attempts.length || attempt > 1 ||
        result.outcome == ScrewdriverContract.NONE || !Math.isFinite(result.torqueNm) || result.torqueNm < 0 ||
        !Math.isFinite(result.angle) || result.angle < 0 || result.outcome < ScrewdriverContract.OK || result.outcome > ScrewdriverContract.SCREW_LOST)
      throw "Fastening report needs an ordered measured attempt for its exact connection";
    for (previous in attempts) if (previous.screw == screw) throw "A replacement must be a new identified screw";
    attempts.push({attempt:attempt,screw:screw,result:{fastening:result.fastening,torqueNm:result.torqueNm,angle:result.angle,
      outcome:result.outcome,diagnostic:result.diagnostic},
      disposition:result.outcome == ScrewdriverContract.OK ? Tightened : PendingRecovery});
  }
  public function recovered():Void {
    if (finished || attempts.length == 0) throw "Recovery report has no active failed attempt";
    var attempt = attempts[attempts.length-1];
    attempt.disposition = switch attempt.result.outcome {
      case ScrewdriverContract.NO_SCREW: Missing;
      case ScrewdriverContract.SCREW_LOST: Lost;
      case ScrewdriverContract.OK: throw "A tightened screw cannot be reported as rejected";
      case _: Rejected;
    };
  }
  public function finish():Void {
    if (attempts.length == 0) throw "Completed fastening has no measured result";
    finished = true; failed = attempts[attempts.length-1].result.outcome != ScrewdriverContract.OK;
  }
  /** The occurrence actually installed after recovery, rather than the original screw's nominal CAD identity. */
  public function installed():Null<String> {
    if (!finished || failed || attempts.length == 0) return null;
    return attempts[attempts.length-1].screw;
  }
  /** Resume only cancellation of the current unfinished attempt, preserving prior dispositions. */
  public function resumeAfterStop():Void {
    if (finished || fault != "cancelled") throw "Fastening report cannot resume this fault";
    fault = null; failed = false;
  }
  public function stop(message:String):Void {
    fault = message; failed = true;
    if (attempts.length > 0) {
      var last = attempts[attempts.length-1];
      if (last.disposition == PendingRecovery) last.disposition = RecoveryFailed;
    }
  }
}
