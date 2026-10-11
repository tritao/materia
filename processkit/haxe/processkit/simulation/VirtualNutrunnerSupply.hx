package processkit.simulation;

import haxe.Int64;
import robotkit.runtime.Simulation;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.VirtualDeviceOptions;
import processkit.screwdriving.ScrewdriverReading;
import processkit.screwdriving.ScrewdriverSensor;
import processkit.screwdriving.ScrewdriverProcessDevice.ScrewdriverChannelNames;
import processkit.simulation.SimulationScrewdriverSupply.ScrewdriverControlState;
import processkit.simulation.SimulationScrewdriverSupply.ScrewdriverJointState;

/** RKD6 nutrunner with the canonical CAD joint plant as its hardware-in-loop load.
    GPIO comes from the executed board; ADC frames retain their acquisition ticks and sequence. */
class VirtualNutrunnerSupply implements SimulationScrewdriverSupply {
  final simulation:Simulation;
  final runtime:RobotRuntime;
  final robotIndex:Int;
  final sensorId:String;
  final sensorSlot:Int;
  final feedJoint:Int;
  final spindleJoint:Int;
  final indices:Array<Int>;
  final periodNs:Int64;
  var physicalStep:Null<Int64> = null;
  var submittedStep:Null<Int64> = null;
  var sequence:Int64 = Int64.ofInt(0);
  var publicationSequence:Int64 = Int64.ofInt(0);
  var latest:ScrewdriverReading = ScrewdriverSensor.idle();

  public function new(simulation:Simulation, runtime:RobotRuntime, robotIndex:Int, sensorId:String,
      blueprint:RobotRuntimeBlueprint, channels:ScrewdriverChannelNames, feedJoint:Int, spindleJoint:Int, sensorSlot:Int = 0) {
    if (simulation == null || runtime == null || robotIndex < 0 || sensorId == null || sensorId.length == 0
        || blueprint == null || channels == null || feedJoint < 0 || spindleJoint < 0 || feedJoint == spindleJoint
        || sensorSlot < 0 || sensorSlot >= 8) throw "Virtual nutrunner needs its controller, physical joints and named sensor slot";
    this.simulation = simulation; this.runtime = runtime; this.robotIndex = robotIndex; this.sensorId = sensorId;
    this.feedJoint = feedJoint; this.spindleJoint = spindleJoint; this.sensorSlot = sensorSlot;
    indices = channelIndices(blueprint,channels);
    periodNs = Int64.fromFloat(simulation.fixedTimestepSeconds * 1e9);
  }
  static function channelIndices(blueprint:RobotRuntimeBlueprint, channels:ScrewdriverChannelNames):Array<Int> {
    var result:Array<Int> = [];
    for (name in [channels.start,channels.direction,channels.program,channels.vacuum]) {
      var found = -1;
      for (i in 0...blueprint.channels.length) if (blueprint.channels[i].id == name) found = i;
      if (found < 0) throw 'Virtual nutrunner channel "$name" is undeclared';
      result.push(found);
    }
    return result;
  }
  public static function configure(options:VirtualDeviceOptions, blueprint:RobotRuntimeBlueprint,
      channels:ScrewdriverChannelNames, maximumAge:Float, sensorSlot:Int = 0):Void {
    if (options == null || blueprint == null || channels == null || !Math.isFinite(maximumAge) || maximumAge <= 0
        || sensorSlot < 0 || sensorSlot >= 8) throw "Virtual nutrunner needs valid channels, acquisition age and sensor slot";
    var indices = channelIndices(blueprint,channels);
    options.peripheralKind = 3; options.externalSensorSlots = [sensorSlot];
    options.peripheralParameters = [sensorSlot,indices[0],indices[1],indices[2],indices[3],maximumAge];
  }
  function digital(index:Int):Bool {
    var value = simulation.virtualDeviceChannel(robotIndex,index);
    if (value != 0 && value != 1) throw 'Virtual nutrunner digital channel $index is malformed';
    return value != 0;
  }
  public function controls():ScrewdriverControlState return {start:digital(indices[0]),reverse:digital(indices[1]),
    program:simulation.virtualDeviceChannel(robotIndex,indices[2]),vacuum:digital(indices[3])};
  public function joints():ScrewdriverJointState {
    var feed = simulation.jointSample(robotIndex,feedJoint), spindle = simulation.jointSample(robotIndex,spindleJoint);
    if (Int64.compare(feed.get_step_index(),spindle.get_step_index()) != 0 || feed.get_simulation_time() != spindle.get_simulation_time())
      throw "Virtual nutrunner feed and spindle came from different physics snapshots";
    physicalStep = feed.get_step_index();
    return {feedPosition:feed.get_position(),spindlePosition:spindle.get_position(),spindleSpeed:spindle.get_velocity()};
  }
  public function publish(reading:ScrewdriverReading):Void {
    var step = physicalStep, previous = submittedStep;
    if (step == null || previous != null && Int64.compare(step,previous) <= 0)
      throw "Virtual nutrunner needs a new physical joint acquisition";
    var values = ScrewdriverSensor.values(reading);
    if (!ScrewdriverSensor.valid(values)) throw "Virtual nutrunner needs a measured seven-value acquisition";
    for (i in 0...values.length) simulation.setVirtualDeviceInput(robotIndex,i,values[i]);
    // The native profile advances its oscillator to this completed physical tick before capture.
    simulation.setVirtualDeviceInput(robotIndex,7,Int64.toFloat(step) * Int64.toFloat(periodNs));
    submittedStep = step;
    var value = simulation.virtualDeviceSensor(robotIndex,sensorSlot), sample = value.get_sample();
    if (Int64.compare(sample.get_sequence(),sequence) < 0) throw "Virtual nutrunner acquisition sequence regressed";
    if (Int64.compare(sample.get_sequence(),sequence) > 0) {
      var acquired = [for (i in 0...sample.get_value_count()) value.get_values(i)];
      if (!ScrewdriverSensor.valid(acquired)) throw "Malformed virtual nutrunner sensor frame";
      runtime.snapshot();
      publicationSequence = Int64.add(publicationSequence,Int64.ofInt(1));
      runtime.publishSensorFrame(sensorId,acquired,publicationSequence,sample.get_source_timestamp_ns(),runtime.endpoint.sourceClockId());
      sequence = sample.get_sequence(); latest = ScrewdriverSensor.reading(acquired);
    }
  }
  public function reading():ScrewdriverReading return ScrewdriverSensor.reading(ScrewdriverSensor.values(latest));
  public function safe():Void simulation.stopVirtualDevice(robotIndex);
  public function reset():Void {
    physicalStep = null; submittedStep = null; sequence = Int64.ofInt(0); latest = ScrewdriverSensor.idle();
  }
}
