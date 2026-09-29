package robotd;

import haxe.Int64;
import haxe.io.Bytes;
import nativekit.ffi.NativeKit;
import nativekit.ffi.NativeKitTypes;
import robotkit.protocol.RobotFrame;
import robotkit.protocol.RobotMessageType;
import robotkit.protocol.StreamSubscription;
import robotkit.transport.NativeTransport;

/** The sole outbound RKF1 family classification. New families register here. */
class OutboundPolicy {
  public static function capabilities():Array<String> {
    return [OutboundFamily.Essential, OutboundFamily.Sensor, OutboundFamily.Camera,
      OutboundFamily.Observation];
  }

  public static function family(messageType:Int):OutboundFamily {
    if (messageType == RobotMessageType.SensorFrame) return OutboundFamily.Sensor;
    if (messageType == RobotMessageType.CameraFrame) return OutboundFamily.Camera;
    if (messageType == RobotMessageType.ImageDetectionObservation) return OutboundFamily.Observation;
    return OutboundFamily.Essential;
  }
}

private class BulkSlot {
  public final key:String;
  public final family:OutboundFamily;
  public var frame:RobotFrame;
  public function new(key:String, family:OutboundFamily, frame:RobotFrame) {
    this.key = key;
    this.family = family;
    this.frame = frame;
  }
}

/** Per-session latest-wins queue. The transport writer remains the only I/O thread. */
class OutboundScheduler {
  public static inline final QUEUE_FULL:Int = -9;
  public static inline final MIN_ESSENTIAL_RESERVE:Int = 65536;
  final transport:TransportHandle;
  final budgetBytes:Int;
  final capacityBytes:Int;
  final slots:Map<String, BulkSlot> = new Map<String, BulkSlot>();
  final keys:Array<String> = [];
  final lastSequence:Map<String, Int64> = new Map<String, Int64>();
  final drops:Map<String, Int> = new Map<String, Int>();
  final loggedDrops:Map<String, Int> = new Map<String, Int>();
  final oversizeLogged:Map<String, Bool> = new Map<String, Bool>();
  var sequenceResets:Int = 0;
  var loggedSequenceResets:Int = 0;
  final rates:Map<String, Float> = new Map<String, Float>();
  final lastOfferNs:Map<String, Int64> = new Map<String, Int64>();
  final lastSentNs:Map<String, Int64> = new Map<String, Int64>();
  var legacy:Bool = true;
  var nextKey:Int = 0;
  var lastLogNs:Int64 = Int64.ofInt(0);

  public function new(transport:TransportHandle, ?budgetBytes:Int = 0) {
    this.transport = transport;
    var capacity = Int64.toInt(NativeTransport.sendQueue(transport).capacity);
    capacityBytes = capacity;
    this.budgetBytes = budgetBytes == 0 ? Std.int(capacity / 2) : budgetBytes;
    if (capacity <= MIN_ESSENTIAL_RESERVE || this.budgetBytes <= 0 ||
        this.budgetBytes > capacity - MIN_ESSENTIAL_RESERVE)
      throw "bulk send budget must leave the minimum essential reserve";
  }

  public function dropCount(family:OutboundFamily):Int {
    var value = drops.get(family);
    return value == null ? 0 : value;
  }

  public function configure(subscriptions:Array<StreamSubscription>):Void {
    var validated:Map<String, Float> = new Map<String, Float>();
    if (subscriptions != null) for (item in subscriptions) {
      if (item == null || item.maxRateHz < 0 || !Math.isFinite(item.maxRateHz))
        throw "invalid RKF1 stream subscription";
      if (item.family == OutboundFamily.Essential || item.family == OutboundFamily.Sensor ||
          item.family == OutboundFamily.Camera || item.family == OutboundFamily.Observation)
        validated.set(item.family, item.maxRateHz);
    }
    rates.clear();
    lastOfferNs.clear();
    lastSentNs.clear();
    legacy = subscriptions == null || subscriptions.length == 0;
    if (legacy) return;
    for (family in validated.keys()) rates.set(family, validated.get(family));
    // A repeated Hello cannot leave unsubscribed bulk data queued.
    for (key in keys.copy()) {
      var slot = slots.get(key);
      if (slot != null && !subscribed(slot.family)) {
        slots.remove(key);
        keys.remove(key);
      }
    }
    nextKey = 0;
  }

  public function subscribed(family:OutboundFamily):Bool {
    return family == OutboundFamily.Essential || legacy || rates.exists(family);
  }

  /** Records each sensor sequence even when filtered, so old frames are not retried. */
  public function shouldOffer(family:OutboundFamily, sensorId:String, sequence:Int64,
      nowNs:Int64):Bool {
    var key = family + ":" + sensorId;
    if (!accepts(family, sensorId, sequence)) return false;
    if (!subscribed(family)) {
      rememberSequence(key, sequence);
      return false;
    }
    var rate = rates.get(family);
    if (rate != null && rate > 0) {
      var previous = lastOfferNs.get(key);
      var interval = 1000000000.0 / rate;
      if (!slots.exists(key) && previous != null &&
          Int64.toFloat(Int64.sub(nowNs, previous)) < interval) {
        rememberSequence(key, sequence);
        return false;
      }
      if (previous == null) lastOfferNs.set(key, nowNs);
      else if (!slots.exists(key)) {
        var elapsed = Int64.toFloat(Int64.sub(nowNs, previous));
        var steps = Math.ffloor(elapsed / interval);
        lastOfferNs.set(key, Int64.add(previous, Int64.fromFloat(steps * interval)));
      }
    }
    return true;
  }

  public function accepts(family:OutboundFamily, sensorId:String, sequence:Int64):Bool {
    var previous = lastSequence.get(family + ":" + sensorId);
    return previous == null || Int64.compare(sequence, previous) != 0;
  }

  function rememberSequence(key:String, sequence:Int64):Void {
    var previous = lastSequence.get(key);
    if (previous != null && Int64.compare(sequence, previous) < 0) {
      sequenceResets++;
      lastOfferNs.remove(key);
      lastSentNs.remove(key);
    }
    lastSequence.set(key, sequence);
  }

  public function offer(family:OutboundFamily, sensorId:String, sequence:Int64,
      frame:RobotFrame):Void {
    if (OutboundPolicy.family(frame.messageType) != family)
      throw "outbound frame family does not match its policy";
    if (family == OutboundFamily.Essential) throw "essential frames cannot enter bulk scheduler";
    var key = family + ":" + sensorId;
    if (!accepts(family, sensorId, sequence)) return;
    var oldKey = (family == OutboundFamily.Camera ? OutboundFamily.Sensor : OutboundFamily.Camera)
      + ":" + sensorId;
    var old = family == OutboundFamily.Observation ? null : slots.get(oldKey);
    if (old != null) {
      slots.remove(oldKey);
      var oldIndex = keys.indexOf(oldKey);
      if (oldIndex >= 0) {
        keys.splice(oldIndex, 1);
        if (oldIndex < nextKey) nextKey--;
      }
      countDrop(old.family);
    }
    rememberSequence(key, sequence);
    var slot = slots.get(key);
    if (slot == null) {
      slots.set(key, new BulkSlot(key, family, frame));
      keys.push(key);
    } else {
      slot.frame = frame;
      countDrop(family);
    }
  }

  /** False means a non-capacity transport failure; the caller closes the session. */
  public function flush(?maxFrames:Int = 64):Bool {
    var attempts = keys.length;
    var sent = 0;
    while (attempts > 0 && keys.length > 0) {
      if (sent >= maxFrames) break;
      attempts--;
      if (nextKey >= keys.length) nextKey = 0;
      var key = keys[nextKey];
      var slot = slots.get(key);
      if (slot == null) throw "outbound slot index is inconsistent";
      var rate = rates.get(slot.family);
      var lastSent = lastSentNs.get(key);
      var nowNs = NativeKit.nk_time_now_ns();
      if (rate != null && rate > 0 && lastSent != null &&
          Int64.toFloat(Int64.sub(nowNs, lastSent)) < 1000000000.0 / rate) {
        nextKey++;
        continue;
      }
      var queue = NativeTransport.sendQueue(transport);
      var frameBytes = RobotFrame.HEADER_BYTES + slot.frame.payload.length;
      for (attachment in slot.frame.attachments) frameBytes += 4 + attachment.length;
      if (frameBytes > capacityBytes - MIN_ESSENTIAL_RESERVE) {
        countDrop(slot.family);
        if (!oversizeLogged.exists(key)) {
          oversizeLogged.set(key, true);
          Sys.println('robotd: oversized outbound ${slot.family} frame $key ($frameBytes bytes) dropped');
        }
        slots.remove(key);
        keys.splice(nextKey, 1);
        continue;
      }
      if (Int64.compare(queue.queuedBytes, Int64.ofInt(budgetBytes)) >= 0 ||
          Int64.compare(Int64.add(queue.queuedBytes, Int64.ofInt(frameBytes)),
            Int64.ofInt(capacityBytes - MIN_ESSENTIAL_RESERVE)) > 0) {
        nextKey++;
        continue;
      }
      var result = NativeTransport.sendStatus(transport, slot.frame.encode());
      if (result == QUEUE_FULL) {
        nextKey++;
        continue;
      }
      if (result != Result.Ok) return false;
      if (rate != null && rate > 0) {
        if (lastSent == null) lastSentNs.set(key, nowNs);
        else {
          var interval = 1000000000.0 / rate;
          var elapsed = Int64.toFloat(Int64.sub(nowNs, lastSent));
          var steps = Math.ffloor(elapsed / interval);
          lastSentNs.set(key, Int64.add(lastSent, Int64.fromFloat(steps * interval)));
        }
      }
      sent++;
      slots.remove(key);
      keys.splice(nextKey, 1);
      // The next key now occupies this position; the next flush starts there.
    }
    return true;
  }

  public function logDrops(nowNs:Int64, sessionId:Int64):Void {
    if (Int64.compare(Int64.sub(nowNs, lastLogNs), Int64.ofInt(1000000000)) < 0)
      return;
    lastLogNs = nowNs;
    var parts:Array<String> = [];
    for (family in [OutboundFamily.Sensor, OutboundFamily.Camera, OutboundFamily.Observation]) {
      var total = dropCount(family);
      var logged = loggedDrops.get(family);
      var delta = total - (logged == null ? 0 : logged);
      if (delta > 0) parts.push('$family=$delta');
      loggedDrops.set(family, total);
    }
    if (sequenceResets > loggedSequenceResets) {
      parts.push('sequence_resets=${sequenceResets - loggedSequenceResets}');
      loggedSequenceResets = sequenceResets;
    }
    if (parts.length > 0)
      Sys.println('robotd: session $sessionId outbound ' + parts.join(' '));
  }

  function countDrop(family:OutboundFamily):Void {
    drops.set(family, dropCount(family) + 1);
  }
}
