package processkit;

import sys.thread.Mutex;
import sys.thread.Lock;
import sys.thread.Thread;
import processkit.tool.WeldSensor;
import processkit.tool.WeldSensor.WeldReading;
import processkit.tool.WeldFault;

/** One transport owner, independent of planning. Its mailbox holds only the latest desired state. */
class WelderDeviceOwner implements WelderOutputs implements WelderFeedback {
  final device:PolledWelderDevice;
  final clock:Void->Float;
  final period:Float;
  final mutex = new Mutex();
  final ended = new Lock();
  var closing = false;
  var closed = false;
  var inhibited = true;
  var stopPending = true;
  var desiredArc = false;
  var desiredWire = 0.0;
  var desiredVoltage = 0.0;
  var changed = true;
  var initialized = false;
  var failure:Null<String> = null;
  var latest:WeldReading = {arc:false, currentA:0.0, voltageV:0.0, touch:false, fault:0, powerW:0.0};

  public function new(device:PolledWelderDevice, clock:Void->Float, period:Float = 0.005) {
    if (device == null || clock == null || !Math.isFinite(period) || period <= 0.0 || period > 0.05)
      throw "Welder owner needs a device, monotonic clock and bounded polling period";
    this.device = device; this.clock = clock; this.period = period;
    #if wasm
    throw "Welder device owner requires a threaded host";
    #else
    Thread.create(run);
    #end
  }

  public function ready():Bool {
    mutex.acquire(); var value = initialized && failure == null && !closed && !closing; mutex.release(); return value;
  }
  public function faultDetail():Null<String> {
    mutex.acquire(); var value = failure; mutex.release(); return value;
  }
  public function reading():WeldReading {
    mutex.acquire(); var value = copy(latest);
    if (failure != null && value.fault == WeldFault.None) value.fault = WeldFault.ArcLost;
    mutex.release(); return value;
  }
  /** Explicit rearming only; clearing device feedback never replays an old ignition request. */
  public function reset():Void {
    mutex.acquire();
    if (!initialized || latest.fault != WeldFault.None || closing || closed || latest.arc) {
      mutex.release(); throw "Welder owner cannot reset until its device is ready and safe";
    }
    failure = null; inhibited = false; desiredArc = false; desiredWire = 0.0; changed = true;
    mutex.release();
  }
  public function setArc(on:Bool):Void {
    mutex.acquire();
    if (on && (inhibited || failure != null || closing || closed)) {
      mutex.release(); throw "Welder owner is inhibited";
    }
    desiredArc = on; changed = true; mutex.release();
  }
  public function setWireSpeed(value:Float):Void {
    if (!Math.isFinite(value) || value < 0.0) throw "Invalid welding wire speed";
    mutex.acquire(); desiredWire = inhibited || closing || closed ? 0.0 : value; changed = true; mutex.release();
  }
  public function setVoltage(value:Float):Void {
    if (!Math.isFinite(value) || value < 0.0) throw "Invalid welding voltage";
    mutex.acquire(); desiredVoltage = value; changed = true; mutex.release();
  }
  public function safe():Void {
    mutex.acquire(); inhibit(); mutex.release();
  }
  /** Wait for acknowledged off, or close the link after a bounded shutdown attempt. */
  public function close():Void {
    mutex.acquire();
    if (closed) { mutex.release(); return; }
    if (closing) { mutex.release(); throw "Welder owner close already in progress"; }
    closing = true; inhibit(); mutex.release();
    ended.wait();
  }
  function inhibit():Void {
    inhibited = true; desiredArc = false; desiredWire = 0.0; stopPending = true; changed = true;
  }
  function run():Void {
    var until = -1.0;
    var previous = -1.0;
    try {
      while (true) {
        // Device calls are nonblocking. Holding this mutex also makes stop atomic
        // against applying the mailbox: no copied ignition can outlive a stop.
        mutex.acquire();
        try {
          var now = clock();
          if (!Math.isFinite(now) || now < 0.0 || now < previous) throw "Welder owner clock moved backwards";
          previous = now;
          if (closing && until < 0.0) until = now + 2.0;
          if (stopPending) { device.safe(); stopPending = false; }
          if (changed && device.ready() && failure == null) {
            var output = device.outputs();
            output.setVoltage(desiredVoltage);
            output.setWireSpeed(inhibited ? 0.0 : desiredWire);
            output.setArc(!inhibited && desiredArc);
            changed = false;
          }
          device.poll(now);
          latest = copy(device.feedback().reading());
          if (!WeldSensor.valid(WeldSensor.values(latest))) throw "Welder owner received malformed feedback";
          initialized = device.ready();
          var detail = device.faultDetail();
          if (detail != null || latest.fault != WeldFault.None) {
            if (failure == null) {
              failure = detail == null ? "Welder device fault" : detail; inhibit();
            }
          }
          var done = closing && (device.safeAcknowledged() || now >= until);
          mutex.release();
          if (done) break;
        } catch (error:Dynamic) { mutex.release(); throw error; }
        Sys.sleep(period);
      }
    } catch (error:Dynamic) {
      mutex.acquire(); failure = Std.string(error); inhibit();
      latest = {arc:false, currentA:0.0, voltageV:0.0, touch:false, fault:WeldFault.ArcLost, powerW:0.0};
      mutex.release();
      try device.safe() catch (_:Dynamic) {}
    }
    try device.close() catch (_:Dynamic) {}
    mutex.acquire(); closed = true; mutex.release(); ended.release();
  }
  static function copy(value:WeldReading):WeldReading return {
    arc:value.arc, currentA:value.currentA, voltageV:value.voltageV, touch:value.touch,
    fault:value.fault, powerW:value.powerW
  };
}
