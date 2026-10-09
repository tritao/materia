package robotkit.runtime;

import haxe.Int64;
import robotkit.tool.HeaterSensor;
import robotkit.tool.HeaterSensor.HeaterReading;

/**
 * One virtual heater: where the host reaches it, how its controller is tuned and what its thermal plant is. The
 * defaults are a 100k NTC 3950 thermistor under a 4.7k pull-up on a 3.3 V ADC and Marlin-like watchdog windows; the
 * values of the plant are assumed per part and set by whoever builds the machine.
 */
class HeaterSetup {
  public final name:String;
  /** The external `tool_heater` sensor the readings are published to, and the board's slot (0 to 7) that feeds it. */
  public final sensorId:String;
  public final sensorSlot:Int;
  /** The analog channel the target (degrees Celsius) is set on, and the fan channel that blows on this heater, if any. */
  public final targetChannel:String;
  public var fanChannel:Null<String> = null;
  /** The deployment actuator of the extruder this heater feeds (pushing it forward while cold is a fault), or -1. */
  public var extruderActuator:Int = -1;
  public var r0Ohm:Float = 100000.0;
  public var t0Celsius:Float = 25.0;
  public var beta:Float = 3950.0;
  public var pullupOhm:Float = 4700.0;
  public var vRef:Float = 3.3;
  public var kp:Float = 0.08;
  public var ki:Float = 0.004;
  public var kd:Float = 0.5;
  public var maxDuty:Float = 1.0;
  public var maxTemperature:Float = 275.0;
  /** The material's minimum extrusion temperature; 0 leaves cold extrusion unchecked. */
  public var minExtrusionTemperature:Float = 0.0;
  public var bandCelsius:Float = 3.0;
  public var pidRangeCelsius:Float = 10.0;
  public var gainWindowSeconds:Float = 20.0;
  public var gainMinCelsius:Float = 2.0;
  public var hysteresisCelsius:Float = 15.0;
  public var hysteresisWindowSeconds:Float = 10.0;
  /** The thermal plant: heat capacity (J/K), power at full duty (W), loss to the room (W/K), extra loss with the fan on (W/K). */
  public var heatCapacity:Float = 10.0;
  public var power:Float = 40.0;
  public var loss:Float = 0.06;
  public var fanLoss:Float = 0.0;
  public var ambient:Float = 22.0;

  public function new(name:String, sensorId:String, sensorSlot:Int, targetChannel:String) {
    this.name = name; this.sensorId = sensorId; this.sensorSlot = sensorSlot; this.targetChannel = targetChannel;
  }
}

/**
 * The heaters of a virtual RKD6 device: the board runs the controller code it would run on hardware (PID, watchdog,
 * safe state) against a lumped thermal plant, and this class configures it, publishes what it reports to the runtime's
 * `tool_heater` sensors, and lets an experiment reach the plant (a disconnected element, a thermistor that has fallen
 * off, a draught).
 */
class VirtualHeaters {
  /** `input` kinds of `setInput`. */
  public static inline var ELEMENT:Int = 0;
  public static inline var SENSOR_ATTACHED:Int = 1;
  public static inline var BLAST:Int = 2;
  public static inline var SENSOR_OPEN:Int = 3;
  public static inline var SENSOR_SHORT:Int = 4;
  static inline var INPUTS_PER_HEATER:Int = 8;
  static inline var PARAMETERS_PER_HEATER:Int = 26;

  final simulation:Simulation;
  final runtime:RobotRuntime;
  final robotIndex:Int;
  final setups:Array<HeaterSetup>;
  final sequences:Array<Int64>;
  final publications:Array<Int64>;
  final latest:Array<HeaterReading>;

  public function new(simulation:Simulation, runtime:RobotRuntime, robotIndex:Int, setups:Array<HeaterSetup>) {
    this.simulation = simulation; this.runtime = runtime; this.robotIndex = robotIndex; this.setups = setups.copy();
    sequences = [for (_ in setups) Int64.ofInt(0)];
    publications = [for (_ in setups) Int64.ofInt(0)];
    latest = [for (_ in setups) {celsius: 0.0, targetCelsius: 0.0, duty: 0.0, atTarget: false, fault: 0}];
  }

  /** Sets up the board's heater profile (peripheral 2) and the sensor slots it feeds; before the robot is added. */
  public static function configure(options:VirtualDeviceOptions, blueprint:RobotRuntimeBlueprint, setups:Array<HeaterSetup>):Void {
    if (setups.length < 1 || setups.length > 2) throw "A virtual device has one or two heaters";
    function channelIndex(name:String):Int {
      for (i in 0...blueprint.channels.length) if (blueprint.channels[i].id == name) return i;
      throw 'Virtual heater channel "$name" is undeclared';
    }
    var parameters:Array<Float> = [setups.length];
    var slots:Array<Int> = [];
    for (setup in setups) {
      slots.push(setup.sensorSlot);
      parameters = parameters.concat([setup.sensorSlot, channelIndex(setup.targetChannel),
        setup.fanChannel == null ? -1 : channelIndex(setup.fanChannel), setup.extruderActuator,
        setup.r0Ohm, setup.t0Celsius, setup.beta, setup.pullupOhm, setup.vRef, setup.kp, setup.ki, setup.kd, setup.maxDuty,
        setup.maxTemperature, setup.minExtrusionTemperature, setup.bandCelsius, setup.pidRangeCelsius,
        setup.gainWindowSeconds, setup.gainMinCelsius, setup.hysteresisCelsius, setup.hysteresisWindowSeconds,
        setup.heatCapacity, setup.power, setup.loss, setup.fanLoss, setup.ambient]);
    }
    options.peripheralKind = 2;
    options.externalSensorSlots = slots;
    options.peripheralParameters = parameters;
  }

  /** Publishes the device's latest report of each heater to its sensor, and returns the readings. */
  public function observe():Array<HeaterReading> {
    for (index in 0...setups.length) {
      var value = simulation.virtualDeviceSensor(robotIndex, setups[index].sensorSlot);
      var sample = value.get_sample();
      if (Int64.compare(sample.get_sequence(), sequences[index]) <= 0) continue;
      var values = [for (i in 0...sample.get_value_count()) value.get_values(i)];
      if (!HeaterSensor.valid(values)) throw 'Malformed virtual heater sensor "${setups[index].sensorId}"';
      publications[index] = Int64.add(publications[index], Int64.ofInt(1));
      // The board and the runtime share one oscillator; observe the runtime first so a reset has advanced its clock epoch.
      runtime.snapshot();
      runtime.publishSensorFrame(setups[index].sensorId, values, publications[index], sample.get_source_timestamp_ns(),
        runtime.endpoint.sourceClockId());
      sequences[index] = sample.get_sequence();
      latest[index] = HeaterSensor.reading(values);
    }
    return latest.copy();
  }

  public function reading(name:String):HeaterReading return latest[indexOf(name)];

  /** The robot's board was restarted (a new session): publication restarts from nothing. */
  public function reset():Void {
    for (index in 0...setups.length) {
      sequences[index] = Int64.ofInt(0);
      latest[index] = {celsius: 0.0, targetCelsius: 0.0, duty: 0.0, atTarget: false, fault: 0};
    }
  }

  /** Reaches the plant of heater `name`: see the kinds above. */
  public function setInput(name:String, kind:Int, value:Float):Void
    simulation.setVirtualDeviceInput(robotIndex, indexOf(name) * INPUTS_PER_HEATER + kind, value);

  function indexOf(name:String):Int {
    for (i in 0...setups.length) if (setups[i].name == name) return i;
    throw 'No virtual heater "$name"';
  }
}
