package processkit.screwdriving;

import haxe.Int64;
import robotkit.runtime.RobotRuntime;
import robotkit.execution.FiredProcessEvent;
import processkit.screwdriving.ScrewdriverProcessDevice.ScrewdriverChannelNames;
import processkit.screwdriving.ScrewdriverAcquisitionSource.ScrewdriverAcquisition;

/** Binds commissioned controller channels and measured frames to the existing mission runtime.
    The owner publishes acquisitions independently of planning, without synthesizing sensor time. */
class ScrewdriverRuntimeBinding implements processkit.ProcessDevice {
  public final owner:ScrewdriverDeviceOwner;
  public final device:ScrewdriverProcessDevice;
  final runtime:RobotRuntime;
  final sensor:String;
  final channels:ScrewdriverChannelNames;
  var sequence:Int64 = Int64.ofInt(0);

  public function new(runtime:RobotRuntime, controller:PolledScrewdriverDevice, clock:Void->Float,
      sensor:String, channels:ScrewdriverChannelNames, slot:Int, period:Float = 0.005) {
    if (runtime == null || controller == null || clock == null || sensor == null || sensor.length == 0
        || channels == null || slot <= 0) throw "Screwdriver binding needs its runtime, controller, sensor, channels and slot";
    if (!runtime.hasExternalSensor(sensor,"tool_screwdriver"))
      throw 'Screwdriver sensor "$sensor" is not declared as an external tool sensor';
    var names = [channels.start,channels.direction,channels.program,channels.vacuum];
    for (i in 0...names.length) {
      var found = false;
      for (declaration in runtime.channels) if (declaration.id == names[i]) {
        found = switch declaration.safeValue {case Analog(_): i == 2; case Digital(_): i != 2; case _: false;};
      }
      if (!found) throw 'Screwdriver channel "${names[i]}" is undeclared or has the wrong value kind';
      for (j in 0...i) if (names[i] == names[j]) throw "Screwdriver binding needs distinct channels";
    }
    this.runtime = runtime; this.sensor = sensor;
    this.channels = {start:channels.start,direction:channels.direction,program:channels.program,vacuum:channels.vacuum};
    owner = new ScrewdriverDeviceOwner(controller,clock,period,publish);
    device = new ScrewdriverProcessDevice(owner,owner,this.channels,slot,owner);
  }
  public function prepare():Void device.prepare();
  public function ready():Bool return device.ready();
  public function fault():Null<String> return device.fault();
  public function safe():Void device.safe();
  /** Route each newly fired record once; other tools retain their own owners. */
  public function apply(records:Array<FiredProcessEvent>):Void {
    device.apply([for (record in records) if (record.channel == channels.start || record.channel == channels.direction
      || record.channel == channels.program || record.channel == channels.vacuum) record]);
  }
  public function close():Void owner.close();
  function publish(acquired:ScrewdriverAcquisition):Void {
    if (Int64.compare(acquired.sequence,sequence) < 0) throw 'Screwdriver sensor "$sensor" acquisition sequence regressed';
    if (Int64.compare(acquired.sequence,sequence) == 0) return;
    var ns = acquired.acquiredAt * 1e9;
    if (!Math.isFinite(ns) || ns < 0 || ns >= 9223372036854775808.0)
      throw 'Screwdriver sensor "$sensor" acquisition time cannot be represented in nanoseconds';
    runtime.publishSensorFrame(sensor,acquired.values,acquired.sequence,Int64.fromFloat(ns),acquired.clockId);
    sequence = acquired.sequence;
  }
}
