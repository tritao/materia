package processkit.feeding;

import haxe.Int64;
import processkit.feeding.ScrewFeederDevice.ScrewFeederReading;

/** Latched acquisition faults stop feeding; a new owner is required to recover. */
class DiscreteScrewFeeder implements ScrewFeederDevice {
  final io:ScrewFeederIo;
  final profile:ScrewFeederIoProfile;
  var latest:Null<ScrewFeederReading> = null;
  var inputs:Array<Bool> = [];
  var enabled = false;
  var pending = false;
  var requestAt = -1.0;
  var now = -1.0;
  var firstPoll = -1.0;
  var enabledAfter = Int64.ofInt(0);
  var requestedAfter = Int64.ofInt(0);
  var fault:Null<String> = null;
  public function new(io:ScrewFeederIo, profile:ScrewFeederIoProfile) {
    if (io == null || profile == null) throw "Screw feeder needs I/O and a commissioned profile";
    this.io = io; this.profile = profile.copy(); safe();
  }
  public function faultDetail():Null<String> return fault;
  public function reading():Null<ScrewFeederReading> {
    var value = latest;
    return value == null ? null : {sequence:value.sequence,acquiredAt:value.acquiredAt,clockId:value.clockId,
      ready:enabled && !pending && fault == null && value.ready && !value.present && value.empty != true && value.jammed != true
        && Int64.compare(value.sequence,enabledAfter) > 0,
      present:value.present,empty:value.empty,jammed:value.jammed};
  }
  public function enable(on:Bool):Void {
    if (!on) {safe(); return;}
    if (fault != null) throw 'Screw feeder fault: $fault';
    if (enabled) return;
    try io.write(profile.enablePin,true) catch (error:Dynamic) {fail(Std.string(error)); throw error;}
    enabled = true;
    var value = latest; enabledAfter = value == null ? Int64.ofInt(0) : value.sequence;
  }
  public function request():Void {
    var value = reading();
    if (value == null || !value.ready || now < 0 || now-value.acquiredAt > profile.maximumAge)
      throw "Screw feeder request needs fresh ready feedback and an empty pickup nest";
    try io.write(profile.requestPin,true) catch (error:Dynamic) {fail(Std.string(error)); throw error;}
    pending = true; requestAt = now; requestedAfter = value.sequence;
  }
  public function safe():Void {
    enabled = false; pending = false;
    var failure:Null<String> = null;
    try io.write(profile.requestPin,false) catch (error:Dynamic) failure = Std.string(error);
    try io.write(profile.enablePin,false) catch (error:Dynamic) failure = Std.string(error);
    if (failure != null) {fault = failure; throw failure;}
  }
  function fail(message:String):Void {
    fault = message;
    try safe() catch (error:Dynamic) fault = message+"; safe outputs failed: "+Std.string(error);
  }
  public function poll(time:Float):Void {
    if (fault != null) return;
    try {
      if (!Math.isFinite(time) || time < 0 || time < now) throw "Screw feeder owner clock moved backwards";
      now = time; if (firstPoll < 0) firstPoll = time;
      var sample = io.sample();
      if (sample != null) {
        if (sample.clockId != profile.clockId || !Math.isFinite(sample.acquiredAt) || sample.acquiredAt < 0
            || sample.acquiredAt > time || Int64.compare(sample.sequence,Int64.ofInt(0)) <= 0 || sample.inputs == null)
          throw "Screw feeder acquisition has invalid source clock, time or sequence";
        var pins = [profile.readyPin,profile.presentPin];
        var emptyPin = profile.emptyPin, jamPin = profile.jamPin;
        if (emptyPin != null) pins.push(emptyPin);
        if (jamPin != null) pins.push(jamPin);
        for (pin in pins) if (pin >= sample.inputs.length) throw "Screw feeder acquisition omitted a commissioned input";
        var prior = latest;
        if (prior != null && Int64.compare(sample.sequence,prior.sequence) < 0) throw "Screw feeder source sequence moved backwards";
        if (prior == null || Int64.compare(sample.sequence,prior.sequence) > 0) {
          if (prior != null && sample.acquiredAt < prior.acquiredAt) throw "Screw feeder acquisition time moved backwards";
          inputs = sample.inputs.copy();
          latest = {sequence:sample.sequence,acquiredAt:sample.acquiredAt,clockId:sample.clockId,
            ready:inputs[profile.readyPin],present:inputs[profile.presentPin],
            empty:emptyPin == null ? null : inputs[emptyPin],jammed:jamPin == null ? null : inputs[jamPin]};
        } else {
          if (sample.acquiredAt != prior.acquiredAt || sample.inputs.length != inputs.length)
            throw "Screw feeder repeated acquisition changed";
          for (i in 0...inputs.length) if (inputs[i] != sample.inputs[i]) throw "Screw feeder repeated acquisition changed";
        }
      }
      var value = latest;
      if (time-(value == null ? firstPoll : value.acquiredAt) > profile.maximumAge) throw "Screw feeder feedback is stale";
      if (value != null && value.jammed == true) throw "Screw feeder jam input is active";
      if (pending && value != null && value.present && Int64.compare(value.sequence,requestedAfter) > 0) {
        io.write(profile.requestPin,false); pending = false;
      }
      if (pending && time-requestAt > profile.deliveryTimeout) throw "Screw feeder delivery timed out";
    } catch (error:Dynamic) {
      fail(Std.string(error));
    }
  }
}
