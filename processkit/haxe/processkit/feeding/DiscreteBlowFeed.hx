package processkit.feeding;

import processkit.feeding.ScrewFeederDevice.ScrewFeederReading;
import processkit.feeding.ScrewFeederIo.ScrewFeederIoSample;

/** Separate metering/air outputs and commissioned optional tube/nose diagnostics, from one atomic acquisition. */
class DiscreteBlowFeed implements ScrewFeederDevice {
  final io:ObservedBlowIo;
  final feeder:DiscreteScrewFeeder;
  final airPin:Int;
  final blockedPin:Null<Int>;
  final noseJamPin:Null<Int>;
  final airPulse:Float;
  final receiverReady:Void -> Bool;
  var now = -1.0;
  var requestAt = -1.0;
  var pending = false;
  var airOn = false;
  var fault:Null<String> = null;
  public function new(source:ScrewFeederIo, profile:ScrewFeederIoProfile, airPin:Int,
      airPulse:Float, blockedPin:Null<Int>, noseJamPin:Null<Int>, receiverReady:Void -> Bool) {
    if (profile == null || source == null || receiverReady == null || airPin < 0 || airPin > 65535
        || airPin == profile.enablePin || airPin == profile.requestPin || !Math.isFinite(airPulse)
        || airPulse <= 0 || airPulse >= profile.deliveryTimeout)
      throw "Blow-feed I/O needs distinct air/metering outputs, commissioned air timing and receiver readiness";
    var inputs = [profile.readyPin,profile.presentPin];
    if (profile.emptyPin != null) inputs.push(profile.emptyPin);
    if (profile.jamPin != null) inputs.push(profile.jamPin);
    for (pin in [blockedPin,noseJamPin]) if (pin != null) {
      if (pin < 0 || pin > 65535 || inputs.indexOf(pin) >= 0) throw "Blow-feed diagnostic inputs must be distinct and bounded";
      inputs.push(pin);
    }
    io = new ObservedBlowIo(source); feeder = new DiscreteScrewFeeder(io,profile);
    this.airPin = airPin; this.airPulse = airPulse; this.blockedPin = blockedPin; this.noseJamPin = noseJamPin;
    this.receiverReady = receiverReady; safe();
  }
  public function faultDetail():Null<String> return fault == null ? feeder.faultDetail() : fault;
  public function reading():Null<ScrewFeederReading> {
    var value = feeder.reading();
    return value == null ? null : {sequence:value.sequence,acquiredAt:value.acquiredAt,clockId:value.clockId,
      ready:value.ready && fault == null && receiverReady(),present:value.present,empty:value.empty,jammed:value.jammed};
  }
  public function enable(on:Bool):Void {
    if (!on) {safe(); return;}
    var error = faultDetail(); if (error != null) throw 'Blow-feed fault: $error';
    feeder.enable(true);
  }
  public function request():Void {
    var value = reading();
    if (value == null || !value.ready) throw "Blow-feed request needs an idle receiver and fresh ready acquisition";
    try {feeder.request(); io.write(airPin,true); airOn = true; pending = true; requestAt = now;}
    catch (error:Dynamic) {fail(Std.string(error)); throw error;}
  }
  public function safe():Void {
    pending = false; requestAt = -1;
    var failure:Null<String> = null;
    try {io.write(airPin,false); airOn = false;} catch (error:Dynamic) failure = Std.string(error);
    try feeder.safe() catch (error:Dynamic) failure = Std.string(error);
    if (failure != null) {fault = failure; throw failure;}
  }
  function fail(message:String):Void {
    fault = message;
    try safe() catch (error:Dynamic) fault = message+"; safe outputs failed: "+Std.string(error);
  }
  public function poll(time:Float):Void {
    if (fault != null) return;
    try {
      feeder.poll(time);
      var error = feeder.faultDetail(); if (error != null) throw error;
      now = time;
      var sample = io.latest;
      if (sample != null) {
        for (pin in [blockedPin,noseJamPin]) if (pin != null && pin >= sample.inputs.length)
          throw "Blow-feed acquisition omitted a commissioned diagnostic input";
        if (blockedPin != null && sample.inputs[blockedPin]) throw "Blow-feed tube is blocked";
        if (noseJamPin != null && sample.inputs[noseJamPin]) throw "Blow-feed receiving nosepiece is jammed";
      }
      var value = feeder.reading();
      if (pending && value != null && value.present) {io.write(airPin,false); airOn = false; pending = false;}
      if (pending && !receiverReady()) throw "Blow-feed receiver lost readiness during delivery";
      if (pending && airOn && time-requestAt >= airPulse) {io.write(airPin,false); airOn = false;}
    } catch (error:Dynamic) fail(Std.string(error));
  }
}
private class ObservedBlowIo implements ScrewFeederIo {
  final source:ScrewFeederIo;
  public var latest(default,null):Null<ScrewFeederIoSample> = null;
  public function new(source:ScrewFeederIo) this.source = source;
  public function write(pin:Int,on:Bool):Void source.write(pin,on);
  public function sample():Null<ScrewFeederIoSample> {
    var value = source.sample();
    latest = value == null ? null : {sequence:value.sequence,acquiredAt:value.acquiredAt,clockId:value.clockId,
      inputs:value.inputs == null ? null : value.inputs.copy()};
    return latest;
  }
}
