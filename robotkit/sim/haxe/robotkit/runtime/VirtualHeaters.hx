package robotkit.runtime;

import haxe.Int64;
import robotkit.tool.HeaterSensor;
import robotkit.tool.HeaterSensor.HeaterReading;

/**
 * Everything a heater's board and thermal plant are told, none of it defaulted: the thermistor and its front end, the PID
 * and its band and watchdog, and the plant (its heat capacity, power, loss to the room and the fan's). The caller states
 * every one, from the part or the job, so a heater cannot run on numbers nobody chose.
 */
typedef HeaterTuning = {
  /** The thermistor: its resistance at `t0Celsius`, and beta. */
  final r0Ohm:Float;
  final t0Celsius:Float;
  final beta:Float;
  /** The controller's ADC front end: the pull-up to the reference voltage. */
  final pullupOhm:Float;
  final vRef:Float;
  final kp:Float;
  final ki:Float;
  final kd:Float;
  final maxDuty:Float;
  final maxTemperature:Float;
  final bandCelsius:Float;
  final pidRangeCelsius:Float;
  final gainWindowSeconds:Float;
  final gainMinCelsius:Float;
  final hysteresisCelsius:Float;
  final hysteresisWindowSeconds:Float;
  /** The thermal plant: heat capacity (J/K), power at full duty (W), loss to the room (W/K), extra loss with the fan on (W/K), the room (C). */
  final heatCapacity:Float;
  final power:Float;
  final loss:Float;
  final fanLoss:Float;
  final ambient:Float;
}

/** One heater of the virtual device: where it is wired, and its `HeaterTuning`. */
class HeaterSetup {
  public final name:String;
  /** The external `tool_heater` sensor the readings are published to, and the board's slot (0 to 7) that feeds it. */
  public final sensorId:String;
  public final sensorSlot:Int;
  /** The analog channel the target (degrees Celsius) is set on, and the fan channel that blows on this heater, if any. */
  public final targetChannel:String;
  public var fanChannel:Null<String> = null;
  /** The extruder this heater feeds (pushing filament while cold is a fault), or null. */
  public var extruder:Null<HeaterExtruder> = null;
  /** The material's minimum extrusion temperature; 0 leaves cold extrusion unchecked. */
  public var minExtrusionTemperature:Float = 0.0;
  /** The thermistor, controller and plant, as the caller stated them. */
  public final tuning:HeaterTuning;

  public function new(name:String, sensorId:String, sensorSlot:Int, targetChannel:String, tuning:HeaterTuning) {
    this.name = name; this.sensorId = sensorId; this.sensorSlot = sensorSlot; this.targetChannel = targetChannel;
    this.tuning = tuning;
  }
}

/** The deployment actuator that drives an extruder, and which way its step count goes when it pushes filament in. */
class HeaterExtruder {
  public final actuator:Int;
  /** +1 when the actuator's steps count up as filament is pushed in, -1 when they count down (a motor geared the other way round). */
  public final feed:Int;

  public function new(actuator:Int, feed:Int) {
    if (actuator < 0) throw "An extruder's actuator is a deployment index";
    if (feed != 1 && feed != -1) throw "An extruder's feed direction is +1 or -1";
    this.actuator = actuator; this.feed = feed;
  }

  /** The extruder on a device channel whose actuator is `ratio` times the extruder joint (pushing filament turns the joint forward). */
  public static function onChannel(channel:Int, ratio:Float):HeaterExtruder {
    if (!(ratio != 0.0 && Math.isFinite(ratio))) throw "An extruder's channel ratio must be finite and non-zero";
    return new HeaterExtruder(channel, ratio > 0.0 ? 1 : -1);
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
      var tuning = setup.tuning;
      slots.push(setup.sensorSlot);
      parameters = parameters.concat([setup.sensorSlot, channelIndex(setup.targetChannel),
        setup.fanChannel == null ? -1 : channelIndex(setup.fanChannel),
        setup.extruder == null ? -1 : setup.extruder.actuator, setup.extruder == null ? 0 : setup.extruder.feed,
        tuning.r0Ohm, tuning.t0Celsius, tuning.beta, tuning.pullupOhm, tuning.vRef, tuning.kp, tuning.ki, tuning.kd, tuning.maxDuty,
        tuning.maxTemperature, setup.minExtrusionTemperature, tuning.bandCelsius, tuning.pidRangeCelsius,
        tuning.gainWindowSeconds, tuning.gainMinCelsius, tuning.hysteresisCelsius, tuning.hysteresisWindowSeconds,
        tuning.heatCapacity, tuning.power, tuning.loss, tuning.fanLoss, tuning.ambient]);
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
    simulation.setVirtualDeviceInput(robotIndex, 2, indexOf(name) * INPUTS_PER_HEATER + kind, value);

  function indexOf(name:String):Int {
    for (i in 0...setups.length) if (setups[i].name == name) return i;
    throw 'No virtual heater "$name"';
  }
}
