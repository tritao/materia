package processkit.feeding;

import haxe.Int64;
import processkit.feeding.ScrewFeederDevice.ScrewFeederReading;

/** Buffered bulk transport and metered delivery; pickup and nest occupancy are measured by the host. */
class ScrewFeederModel implements ScrewFeederDevice {
  final initial:Array<String>;
  final stock:Array<String> = [];
  final buffer:Array<String> = [];
  final identities:Map<String,Bool> = [];
  final occupied:Void -> Bool;
  final deposit:String -> Void;
  final trackDelay:Float;
  final deliveryDelay:Float;
  final deliveryTimeout:Float;
  final capacity:Int;
  final clockId:String;
  var enabled = false;
  var pending = false;
  var requestedAt = -1.0;
  var trackAt = -1.0;
  var time = -1.0;
  var sequence = Int64.ofInt(0);
  var fault:Null<String> = null;
  var jammed = false;
  var latest:Null<ScrewFeederReading> = null;
  public var delivered(default,null):Null<String> = null;
  public var deliveries(default,null):Int = 0;
  public function new(inventory:Array<String>, capacity:Int, trackDelay:Float, deliveryDelay:Float,
      deliveryTimeout:Float, clockId:String, occupied:Void -> Bool, deposit:String -> Void) {
    if (capacity < 1 || capacity > 256 || !Math.isFinite(trackDelay) || trackDelay <= 0
        || !Math.isFinite(deliveryDelay) || deliveryDelay <= 0 || !Math.isFinite(deliveryTimeout)
        || deliveryTimeout <= deliveryDelay || clockId == null || clockId.length == 0 || occupied == null || deposit == null)
      throw "Screw feeder model needs explicit buffering, transport timing, source clock and physical nest callbacks";
    if (inventory == null) throw "Screw feeder initial inventory is missing";
    initial = inventory.copy();
    this.capacity = capacity; this.trackDelay = trackDelay; this.deliveryDelay = deliveryDelay;
    this.deliveryTimeout = deliveryTimeout; this.clockId = clockId; this.occupied = occupied; this.deposit = deposit;
    if (inventory.length > 0) refill(inventory);
  }
  /** Restore the authored inventory with the physical session's reset poses; never replay a request. */
  public function reset():Void {
    safe(); stock.resize(0); buffer.resize(0);
    for (id in identities.keys()) identities.remove(id);
    for (id in initial) {identities.set(id,true); stock.push(id);}
    delivered = null; deliveries = 0; fault = null; jammed = false; latest = null; time = -1;
  }
  public function refill(inventory:Array<String>):Void {
    if (inventory == null || inventory.length == 0) throw "Screw feeder refill needs identified inventory";
    var found:Map<String,Bool> = [];
    for (id in inventory) {
      if (id == null || id.length == 0 || identities.exists(id) || found.exists(id))
        throw "Screw feeder inventory needs distinct, never previously delivered identities";
      found.set(id,true);
    }
    for (id in inventory) {identities.set(id,true); stock.push(id);}
  }
  public function nextIdentity():Null<String> return buffer.length == 0 ? null : buffer[0];
  public function buffered():Int return buffer.length;
  public function bulk():Int return stock.length;
  public function faultDetail():Null<String> return fault;
  public function setJam(on:Bool):Void jammed = on;
  public function recover():Void {
    if (enabled || jammed || occupied() || delivered != null) throw "Screw feeder recovery needs disabled, cleared equipment and a confirmed empty nest";
    fault = null; latest = null;
  }
  public function enable(on:Bool):Void {
    if (!on) {safe(); return;}
    if (fault != null) throw 'Screw feeder fault: $fault';
    enabled = true;
  }
  public function safe():Void {enabled = false; pending = false; requestedAt = -1; trackAt = -1;}
  public function request():Void {
    var value = reading();
    if (value == null || !value.ready || !enabled || pending || occupied() || delivered != null || buffer.length == 0)
      throw "Screw feeder request needs a buffered screw and a confirmed empty pickup nest";
    pending = true; requestedAt = time;
  }
  /** Losing presence alone is not a pickup: the host must identify the body held by the tool. */
  public function confirmPickup(identity:String):Void {
    if (delivered == null || identity != delivered || occupied())
      throw "Screw feeder pickup needs the delivered identity held clear of the nest";
    delivered = null;
  }
  public function reading():Null<ScrewFeederReading> {
    var value = latest;
    return value == null ? null : {sequence:value.sequence,acquiredAt:value.acquiredAt,clockId:value.clockId,
      ready:enabled && fault == null && !pending && value.ready,present:value.present,empty:value.empty,jammed:value.jammed};
  }
  public function poll(now:Float):Void {
    if (!Math.isFinite(now) || now < 0 || now < time) {fault = "Screw feeder model clock moved backwards"; safe(); return;}
    var physicalPresence:Bool;
    try physicalPresence = occupied() catch (error:Dynamic) {fault = Std.string(error); safe(); return;}
    time = now;
    try {
      if (fault == null && jammed) throw "Screw feeder jam is active";
      if (enabled && fault == null) {
        if (stock.length > 0 && buffer.length < capacity) {
          if (trackAt < 0) trackAt = now;
          if (now-trackAt >= trackDelay) {var item = stock.shift(); buffer.push(item); trackAt = now;}
        } else trackAt = -1;
        if (pending) {
          if (physicalPresence) throw "Screw feeder cannot deliver into an occupied nest";
          if (now-requestedAt > deliveryTimeout) throw "Screw feeder delivery timed out";
          if (now-requestedAt >= deliveryDelay) {
            if (buffer.length == 0) throw "Screw feeder lost its buffered screw";
            var identity = buffer[0];
            deposit(identity); delivered = identity; deliveries++;
            var released = buffer.shift(); pending = false;
          }
        }
      }
    } catch (error:Dynamic) {fault = Std.string(error); safe();}
    try physicalPresence = occupied() catch (error:Dynamic) {fault = Std.string(error); safe(); return;}
    sequence = Int64.add(sequence,Int64.ofInt(1));
    latest = {sequence:sequence,acquiredAt:now,clockId:clockId,
      ready:enabled && fault == null && !pending && !physicalPresence && delivered == null && buffer.length > 0,
      present:physicalPresence,empty:stock.length == 0 && buffer.length == 0,jammed:jammed};
  }
}
