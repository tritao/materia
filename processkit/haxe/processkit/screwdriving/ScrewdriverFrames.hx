package processkit.screwdriving;

import haxe.Int64;
import robotkit.core.RobotSnapshot;
import robotkit.tool.ScrewdriverContract;

/** Reads fresh frames from one declared tool, with the same timeout for driving and recovery. */
class ScrewdriverFrames {
  final sensor:String;
  var silent = 0.0;
  var lastSequence:Null<Int64> = null;
  public function new(sensor:String) {
    if (sensor == null || sensor.length == 0) throw "Screwdriver feedback needs a named sensor";
    this.sensor = sensor;
  }
  public function reset():Void {silent = 0; lastSequence = null;}
  public function read(snapshot:RobotSnapshot, durationSeconds:Float):Null<ScrewdriverReading> {
    if (!Math.isFinite(durationSeconds) || durationSeconds <= 0) throw "Screwdriver feedback needs positive time";
    var reading:Null<ScrewdriverReading> = null, matches = 0;
    for (index in 0...snapshot.sensors.length) {
      var frame = snapshot.sensors.get(index);
      if (frame.sensorId == sensor) {
        if (++matches > 1) throw 'Screwdriver sensor "$sensor" has duplicate frames';
        if (frame.kind != ScrewdriverContract.SENSOR_KIND) throw 'Sensor "$sensor" is not a screwdriver';
        var previous = lastSequence;
        if (previous != null && Int64.compare(frame.sequence,previous) < 0)
          throw 'Screwdriver sensor "$sensor" sequence regressed';
        if (previous == null || Int64.compare(frame.sequence,previous) > 0) {
          reading = ScrewdriverSensor.reading(frame.values.toArray()); lastSequence = frame.sequence;
        }
      }
    }
    if (reading == null) {
      silent += durationSeconds;
      if (silent > 2) throw 'Screwdriver sensor "$sensor" has not reported a fresh frame';
    } else silent = 0;
    return reading;
  }
}
