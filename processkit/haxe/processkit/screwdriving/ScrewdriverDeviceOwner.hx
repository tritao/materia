package processkit.screwdriving;

import sys.thread.Mutex;
import sys.thread.Lock;
import sys.thread.Thread;
import processkit.screwdriving.ScrewdriverAcquisitionSource.ScrewdriverAcquisition;

private enum ScrewdriverRequest {
  Program(slot:Int);
  Start(on:Bool);
  Reverse(on:Bool);
  Vacuum(on:Bool);
}

/** One controller owner keeps acquisition and watchdog polling independent of planning.
    A stop discards pending drive requests. Faults require a new owner, never replay. */
class ScrewdriverDeviceOwner implements ScrewdriverOutputs implements ScrewdriverFeedback implements ScrewdriverDeviceControl implements ScrewdriverAcquisitionSource {
  final device:PolledScrewdriverDevice;
  final clock:Void->Float;
  final period:Float;
  final publish:Null<ScrewdriverAcquisition->Void>;
  final mutex = new Mutex();
  final ended = new Lock();
  final requests:Array<ScrewdriverRequest> = [];
  var latest:ScrewdriverReading = ScrewdriverSensor.idle();
  var source:Null<ScrewdriverAcquisition> = null;
  var available = false;
  var armed = false;
  var stopPending = true;
  var closing = false;
  var closed = false;
  var failure:Null<String> = null;

  public function new(device:PolledScrewdriverDevice, clock:Void->Float, period:Float = 0.005,
      publish:Null<ScrewdriverAcquisition->Void> = null) {
    if (device == null || clock == null || !Math.isFinite(period) || period <= 0 || period > 0.05)
      throw "Screwdriver owner needs a device, monotonic clock and bounded polling period";
    this.device = device; this.clock = clock; this.period = period; this.publish = publish;
    #if wasm
    throw "Screwdriver device owner requires a threaded host";
    #else
    Thread.create(run);
    #end
  }
  public function ready():Bool {
    mutex.acquire(); var value = available && failure == null && !closing && !closed;
    mutex.release(); return value;
  }
  public function faultDetail():Null<String> {
    mutex.acquire(); var value = failure; mutex.release(); return value;
  }
  public function reading():ScrewdriverReading {
    mutex.acquire(); var value = copy(latest); mutex.release(); return value;
  }
  public function acquisition():Null<ScrewdriverAcquisition> {
    mutex.acquire(); var value = source;
    var result:Null<ScrewdriverAcquisition> = value == null ? null : copyAcquisition(value);
    mutex.release(); return result;
  }
  function enqueue(request:ScrewdriverRequest, active:Bool):Void {
    mutex.acquire();
    if (closed || closing || failure != null || active && !available || requests.length >= 64) {
      mutex.release(); throw "Screwdriver owner cannot accept this output request";
    }
    requests.push(request);
    switch request {case Program(_): available = false; armed = true; case _:}
    mutex.release();
  }
  public function selectProgram(slot:Int):Void {
    if (slot <= 0) throw "Screwdriver owner needs a positive program slot";
    enqueue(Program(slot),false);
  }
  public function start(on:Bool):Void enqueue(Start(on),on);
  public function reverse(on:Bool):Void enqueue(Reverse(on),on);
  public function vacuum(on:Bool):Void enqueue(Vacuum(on),on);
  /** Discard queued outputs and stop drive, retaining the held screw's vacuum. */
  public function safe():Void {
    mutex.acquire(); inhibit(); mutex.release();
  }
  function inhibit():Void {
    requests.splice(0,requests.length); stopPending = true; available = false; armed = false;
  }
  public function close():Void {
    mutex.acquire();
    if (closed) {mutex.release(); return;}
    if (closing) {mutex.release(); throw "Screwdriver owner close already in progress";}
    closing = true; inhibit(); mutex.release(); ended.wait();
  }
  function run():Void {
    var previous = -1.0;
    var until = -1.0;
    try {
      while (true) {
        mutex.acquire();
        try {
          var now = clock();
          if (!Math.isFinite(now) || now < 0 || now < previous) throw "Screwdriver owner clock moved backwards";
          previous = now;
          if (closing && until < 0) until = now + 2.0;
          if (stopPending) {device.safe(); stopPending = false;}
          var output = device.outputs();
          while (requests.length > 0) {
            var request = requests.shift();
            if (request != null) switch request {
              case Program(slot): output.selectProgram(slot);
              case Start(on): output.start(on);
              case Reverse(on): output.reverse(on);
              case Vacuum(on): output.vacuum(on);
            }
          }
          device.poll(now);
          latest = copy(device.feedback().reading());
          if (!ScrewdriverSensor.valid(ScrewdriverSensor.values(latest))) throw "Screwdriver owner received malformed feedback";
          var detail = device.faultDetail();
          if (detail != null) throw detail;
          var acquired = device.acquisition();
          if (acquired != null) {
            source = copyAcquisition(acquired);
            if (publish != null) publish(copyAcquisition(acquired));
          }
          available = armed && device.ready();
          var stopped = device.safeAcknowledged();
          if (closing && !stopped && now >= until) failure = "Screwdriver shutdown was not acknowledged";
          var done = closing && (stopped || now >= until);
          mutex.release(); if (done) break;
        } catch (error:Dynamic) {mutex.release(); throw error;}
        Sys.sleep(period);
      }
    } catch (error:Dynamic) {
      mutex.acquire(); failure = Std.string(error); inhibit(); mutex.release();
      try device.safe() catch (_:Dynamic) {}
    }
    try device.close() catch (_:Dynamic) {}
    mutex.acquire(); closed = true; available = false; mutex.release(); ended.release();
  }
  static function copy(value:ScrewdriverReading):ScrewdriverReading return {
    torqueNm:value.torqueNm,angle:value.angle,speed:value.speed,strokeMm:value.strokeMm,
    phase:value.phase,result:value.result,vacuum:value.vacuum
  };
  static function copyAcquisition(value:ScrewdriverAcquisition):ScrewdriverAcquisition return {
    sequence:value.sequence,acquiredAt:value.acquiredAt,clockId:value.clockId,values:value.values.copy()
  };
}
