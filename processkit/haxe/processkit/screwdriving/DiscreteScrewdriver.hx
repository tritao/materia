package processkit.screwdriving;

import haxe.Int64;
import robotkit.tool.ScrewdriverContract;

/** Discrete controller outputs with fresh measured feedback and explicit readiness.
    A latched link/acquisition fault requires a new device owner; it cannot restart itself. */
class DiscreteScrewdriver implements ScrewdriverOutputs implements ScrewdriverFeedback implements ScrewdriverDeviceStatus {
  final io:ScrewdriverIo;
  final profile:ScrewdriverIoProfile;
  var latest:ScrewdriverReading = ScrewdriverSensor.idle();
  var sequence:Int64 = Int64.ofInt(0);
  var selectedAfter:Int64 = Int64.ofInt(0);
  var acquiredAt = -1.0;
  var firstPoll = -1.0;
  var lastNow = -1.0;
  var slot = 0;
  var controllerReady = false;
  var running = false;
  var reversing = false;
  var fault:Null<String> = null;
  var lastInputs:Array<Bool> = [];
  var lastValues:Array<Float> = [];
  public function new(io:ScrewdriverIo, profile:ScrewdriverIoProfile) {
    if (io == null || profile == null) throw "Discrete screwdriver needs I/O and a commissioned profile";
    this.io = io; this.profile = profile.copy();
    safe();
  }
  public function ready():Bool return fault == null && controllerReady && slot > 0 && Int64.compare(sequence,selectedAfter) > 0;
  public function faultDetail():Null<String> return fault;
  public function reading():ScrewdriverReading return {torqueNm:latest.torqueNm,angle:latest.angle,speed:latest.speed,
    strokeMm:latest.strokeMm,phase:latest.phase,result:latest.result,vacuum:latest.vacuum};
  public function selectProgram(slot:Int):Void {
    if (fault != null || running || reversing) throw "Discrete screwdriver selection requires stopped, healthy outputs";
    var code = profile.code(slot);
    for (i in 0...profile.programBits.length) io.write(profile.programBits[i],(code & (1 << i)) != 0);
    this.slot = slot; selectedAfter = sequence; controllerReady = false;
  }
  public function start(on:Bool):Void {
    if (on && (!ready() || reversing)) throw "Discrete screwdriver start requires fresh readiness and forward direction";
    io.write(profile.start,on); running = on;
  }
  public function reverse(on:Bool):Void {
    if (on && (!ready() || running)) throw "Discrete screwdriver reverse requires fresh readiness and stopped forward drive";
    if (!on && profile.reverseNeedsStart) io.write(profile.start,false);
    io.write(profile.reverse,on);
    if (on && profile.reverseNeedsStart) io.write(profile.start,true);
    reversing = on;
  }
  public function vacuum(on:Bool):Void {
    if (on && fault != null) throw "Discrete screwdriver cannot acquire after an I/O fault";
    io.write(profile.vacuum,on);
  }
  /** Stop drive and reverse independently; preserve the held screw's vacuum. */
  public function safe():Void {
    var error:Null<String> = null;
    try io.write(profile.start,false) catch (failure:Dynamic) error = Std.string(failure);
    try io.write(profile.reverse,false) catch (failure:Dynamic) error = Std.string(failure);
    running = false; reversing = false;
    if (error != null) {fault = error; throw error;}
  }
  public function poll(now:Float):Void {
    if (fault != null) return;
    try {
      if (!Math.isFinite(now) || now < 0 || now < lastNow) throw "Discrete screwdriver owner clock moved backwards";
      lastNow = now; if (firstPoll < 0) firstPoll = now;
      var sample = io.sample();
      if (sample != null) {
        if (sample.clockId != profile.clockId || !Math.isFinite(sample.acquiredAt) || sample.acquiredAt < 0
            || sample.acquiredAt > now || Int64.compare(sample.sequence,Int64.ofInt(0)) <= 0)
          throw "Discrete screwdriver acquisition has invalid clock, time or sequence";
        if (Int64.compare(sample.sequence,sequence) < 0) throw "Discrete screwdriver acquisition sequence moved backwards";
        if (Int64.compare(sample.sequence,sequence) > 0) {
          if (sample.acquiredAt < acquiredAt || !ScrewdriverSensor.valid(sample.values) || sample.inputs == null)
            throw "Discrete screwdriver needs a consistent measured seven-value acquisition";
          for (pin in [profile.ready,profile.ok,profile.nok]) if (pin >= sample.inputs.length)
            throw 'Discrete screwdriver acquisition is missing input $pin';
          var value = ScrewdriverSensor.reading(sample.values);
          var ok = sample.inputs[profile.ok], nok = sample.inputs[profile.nok];
          if (ok && nok || ok != (value.result == ScrewdriverContract.OK) || nok != (value.result > ScrewdriverContract.OK)
              || (ok || nok) && value.phase != ScrewdriverContract.COMPLETE)
            throw "Discrete screwdriver result bits disagree with measured feedback";
          sequence = sample.sequence; acquiredAt = sample.acquiredAt; latest = value;
          lastInputs = sample.inputs.copy(); lastValues = sample.values.copy();
          controllerReady = sample.inputs[profile.ready];
        } else {
          if (sample.acquiredAt != acquiredAt || sample.inputs == null || sample.values == null
              || sample.inputs.length != lastInputs.length || sample.values.length != lastValues.length)
            throw "Discrete screwdriver repeated sequence changed its acquisition";
          for (i in 0...lastInputs.length) if (sample.inputs[i] != lastInputs[i])
            throw "Discrete screwdriver repeated sequence changed its inputs";
          for (i in 0...lastValues.length) if (sample.values[i] != lastValues[i])
            throw "Discrete screwdriver repeated sequence changed its measurements";
        }
      }
      if (now - (acquiredAt < 0 ? firstPoll : acquiredAt) >= profile.maximumAge)
        throw "Discrete screwdriver measured feedback is stale";
    } catch (error:Dynamic) {
      fault = Std.string(error); controllerReady = false;
      try safe() catch (_:Dynamic) {}
    }
  }
}
