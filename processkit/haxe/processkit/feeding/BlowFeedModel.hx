package processkit.feeding;

import processkit.feeding.ScrewFeederDevice.ScrewFeederReading;

/** Metered bulk transport into an idle tool; air pulse and receipt are separate observations. */
class BlowFeedModel implements ScrewFeederDevice {
  public final transport:ScrewFeederModel;
  final receiverReady:Void -> Bool;
  final airPulse:Float;
  final receiptTimeout:Float;
  var now = -1.0;
  var requestedAt = -1.0;
  var inflight = false;
  var fault:Null<String> = null;
  var blocked = false;
  var noseJam = false;
  var transitLoss = false;
  public var airOn(default,null):Bool = false;
  public function new(inventory:Array<String>, capacity:Int, trackDelay:Float, deliveryDelay:Float,
      timeout:Float, airPulse:Float, clockId:String, receiverReady:Void -> Bool,
      received:Void -> Bool, deposit:String -> Void) {
    if (receiverReady == null || !Math.isFinite(airPulse) || airPulse <= 0 || airPulse >= timeout)
      throw "Blow feed needs receiver readiness and a commissioned finite air pulse shorter than its timeout";
    this.receiverReady = receiverReady; this.airPulse = airPulse; receiptTimeout = timeout;
    transport = new ScrewFeederModel(inventory,capacity,trackDelay,deliveryDelay,timeout,clockId,received,(identity) -> {if (!transitLoss) deposit(identity);});
  }
  public function faultDetail():Null<String> return fault == null ? transport.faultDetail() : fault;
  public function reading():Null<ScrewFeederReading> {
    var value = transport.reading();
    return value == null ? null : {sequence:value.sequence,acquiredAt:value.acquiredAt,clockId:value.clockId,
      ready:value.ready && fault == null && receiverReady(),present:value.present,empty:value.empty,
      jammed:blocked || noseJam || value.jammed == true};
  }
  public function enable(on:Bool):Void {
    if (!on) {safe(); return;}
    var error = faultDetail(); if (error != null) throw 'Blow-feed fault: $error';
    transport.enable(true);
  }
  public function request():Void {
    var value = reading();
    if (value == null || !value.ready) throw "Blow-feed request needs an idle receiving tool and fresh buffered supply";
    transport.request(); requestedAt = now; airOn = true; inflight = true;
  }
  public function safe():Void {airOn = false; requestedAt = -1; inflight = false; transport.safe();}
  public function setTransitLoss(on:Bool):Void transitLoss = on;
  public function setBlocked(on:Bool):Void blocked = on;
  public function setNoseJam(on:Bool):Void noseJam = on;
  /** The host confirms the delivered identity installed or physically discarded clear of the receiver. */
  public function confirmConsumed(identity:String):Void transport.confirmPickup(identity);
  public function recover():Void {
    if (blocked || noseJam || airOn || !receiverReady()) throw "Blow-feed recovery needs cleared tube/nosepiece and a safe idle receiver";
    transport.recover(); fault = null;
  }
  public function reset():Void {safe(); transport.reset(); fault = null; blocked = false; noseJam = false; transitLoss = false; now = -1;}
  public function poll(time:Float):Void {
    if (!Math.isFinite(time) || time < 0 || time < now) {fault = "Blow-feed owner clock regressed"; safe(); return;}
    now = time;
    if (fault == null && (blocked || noseJam)) {fault = blocked ? "Blow-feed tube is blocked" : "Blow-feed receiving nosepiece is jammed"; safe();}
    if (inflight && !receiverReady() && transport.delivered == null) {fault = "Blow-feed receiver lost readiness during delivery"; safe();}
    transport.poll(time);
    var received = transport.reading();
    if (inflight && received != null && received.present) {inflight = false; airOn = false;}
    if (requestedAt >= 0 && time-requestedAt >= airPulse) airOn = false;
    if (inflight && time-requestedAt > receiptTimeout) fault = "Blow-feed receipt timed out: screw missing at nosepiece";
    if (faultDetail() != null) safe();
  }
}
