package robotd;

import haxe.Int64;
import haxe.io.Bytes;
import nativekit.ffi.NativeKitTypes;
import robotkit.protocol.RobotFrame;
import robotkit.protocol.RobotMessageType;
import robotkit.transport.NativeTransport;

/** The sole outbound RKF1 family classification. New families register here. */
class OutboundPolicy {
  public static function family(messageType:Int):OutboundFamily {
    if (messageType == RobotMessageType.SensorFrame) return OutboundFamily.Sensor;
    if (messageType == RobotMessageType.CameraFrame) return OutboundFamily.Camera;
    return OutboundFamily.Essential;
  }
}

private class BulkSlot {
  public final key:String;
  public final family:OutboundFamily;
  public var bytes:Bytes;
  public function new(key:String, family:OutboundFamily, bytes:Bytes) {
    this.key = key;
    this.family = family;
    this.bytes = bytes;
  }
}

/** Per-session latest-wins queue. The transport writer remains the only I/O thread. */
class OutboundScheduler {
  public static inline final QUEUE_FULL:Int = -9;
  final transport:TransportHandle;
  final budgetBytes:Int;
  final slots:Map<String, BulkSlot> = new Map<String, BulkSlot>();
  final keys:Array<String> = [];
  final lastSequence:Map<String, Int64> = new Map<String, Int64>();
  final drops:Map<String, Int> = new Map<String, Int>();
  var nextKey:Int = 0;
  var lastLogNs:Int64 = Int64.ofInt(0);

  public function new(transport:TransportHandle, ?budgetBytes:Int = 0) {
    this.transport = transport;
    var capacity = Int64.toInt(NativeTransport.sendQueue(transport).capacity);
    this.budgetBytes = budgetBytes == 0 ? Std.int(capacity / 2) : budgetBytes;
    if (this.budgetBytes <= 0 || this.budgetBytes >= capacity)
      throw "bulk send budget must be positive and smaller than transport capacity";
  }

  public function dropCount(family:OutboundFamily):Int {
    var value = drops.get(family);
    return value == null ? 0 : value;
  }

  public function accepts(sensorId:String, sequence:Int64):Bool {
    var previous = lastSequence.get(sensorId);
    return previous == null || Int64.compare(sequence, previous) > 0;
  }

  public function offer(family:OutboundFamily, sensorId:String, sequence:Int64,
      frame:RobotFrame):Void {
    if (OutboundPolicy.family(frame.messageType) != family)
      throw "outbound frame family does not match its policy";
    if (family == OutboundFamily.Essential) throw "essential frames cannot enter bulk scheduler";
    var key = family + ":" + sensorId;
    if (!accepts(sensorId, sequence)) return;
    var oldKey = (family == OutboundFamily.Camera ? OutboundFamily.Sensor : OutboundFamily.Camera)
      + ":" + sensorId;
    var old = slots.get(oldKey);
    if (old != null) {
      slots.remove(oldKey);
      var oldIndex = keys.indexOf(oldKey);
      if (oldIndex >= 0) {
        keys.splice(oldIndex, 1);
        if (oldIndex < nextKey) nextKey--;
      }
      countDrop(old.family);
    }
    lastSequence.set(sensorId, sequence);
    var bytes = frame.encode();
    var slot = slots.get(key);
    if (slot == null) {
      slots.set(key, new BulkSlot(key, family, bytes));
      keys.push(key);
    } else {
      slot.bytes = bytes;
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
      var queue = NativeTransport.sendQueue(transport);
      if (Int64.compare(Int64.add(queue.queuedBytes, Int64.ofInt(slot.bytes.length)),
          Int64.ofInt(budgetBytes)) > 0) {
        nextKey++;
        continue;
      }
      var result = NativeTransport.sendStatus(transport, slot.bytes);
      if (result == QUEUE_FULL) {
        nextKey++;
        continue;
      }
      if (result != Result.Ok) return false;
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
    if (dropCount(OutboundFamily.Sensor) + dropCount(OutboundFamily.Camera) > 0)
      Sys.println('robotd: session $sessionId outbound drops sensor=${dropCount(OutboundFamily.Sensor)} camera=${dropCount(OutboundFamily.Camera)}');
  }

  function countDrop(family:OutboundFamily):Void {
    drops.set(family, dropCount(family) + 1);
  }
}
