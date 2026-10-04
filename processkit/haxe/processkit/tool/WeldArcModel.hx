package processkit.tool;

import robotkit.tool.*;

/** The welder behind a torch: the supply's limits and the wire. */
typedef WeldArcConfig = {
  var maxCurrentA:Float;
  /** Arc power delivered per watt drawn from the mains, in (0, 1]. */
  var efficiency:Float;
  var wireDiameterMm:Float;
  /** The wire's extension past the contact tip, in millimetres. */
  var stickoutMm:Float;
};

/** What the model is told each step. */
typedef WeldArcInput = {
  /** The arc channel. */
  var arcCommanded:Bool;
  /** Wire feed speed setpoint, in metres per minute. */
  var wireSpeed:Float;
  /** Voltage setpoint, the arc voltage at the nominal arc length, in volts. */
  var voltageSet:Float;
  /** The supply is on and has nothing wrong with its mains, gas or cooling. */
  var supplyReady:Bool;
  /** Distance from the wire tip to the nearest grounded surface, in metres: negative inside the metal, infinite with none near. */
  var tipDistance:Float;
  /** Distance along the wire from its tip to grounded metal ahead, in metres, or infinite. */
  var wireDistance:Float;
};

/**
 * The electrical and process behaviour of a MIG/MAG welder, without any geometry: it is told how far the wire
 * tip is from the grounded work and what the controller commands, and says what the circuit does. Time
 * advances by `step`, so the model is deterministic and runs the same under any physics backend.
 *
 * **Arc.** With the arc commanded on, the supply ready and the wire fed (`MIN_WIRE_SPEED`), the arc strikes when
 * the work is within `STRIKE_REACH` ahead of the wire tip, as the wire advances to scratch it, and is
 * established after `IGNITION_DELAY` of that. It burns while the arc length stays within `MAX_ARC_LENGTH`. The arc
 * length is the tip's distance from the work plus `NOMINAL_ARC_LENGTH`: a torch held at its tool point, the
 * programmed wire tip on the joint, burns an arc of nominal length. It goes out when the torch is pulled away
 * (arc lost, a fault), when the arc is commanded off, or when the wire stops feeding (burnback).
 *
 * **Current** follows the wire speed by the burn-off law: the wire melts at a rate
 * `MR = α·I·A0/A + β·l·I²·(A0/A)²` (mm/s), the first term the heat at the arc root, the second the resistive
 * heating of the stickout `l` (mm) of wire of cross-section `A` (mm²), against the reference 1.2 mm wire's `A0`. α
 * and β are calibrated for mild steel solid wire to 250 A at 8 m/min, 1.2 mm wire, 15 mm stickout. The supply
 * cannot deliver more than its rating, so the current is capped there. For a given wire speed the current is the
 * positive root of that quadratic.
 *
 * **Voltage** is the setpoint plus `ARC_VOLTAGE_GRADIENT` per millimetre of arc length beyond the nominal, so
 * lifting the torch raises it, never below `MIN_ARC_VOLTAGE`. With the arc off it is the sensing voltage
 * (`SENSE_VOLTAGE`, open circuit), which collapses to nothing when the wire touches the work.
 *
 * **Touch** closes while the wire tip touches grounded work (within `TOUCH_TOLERANCE`) with the arc off and the
 * supply ready.
 *
 * **Faults** latch and clear once the arc is commanded off (a wire stuck in the pool also needs the torch freed).
 * - `NoArc`: the arc was commanded for `NO_ARC_TIMEOUT` without striking;
 * - `ArcLost`: the arc went out while commanded on, other than by burnback;
 * - `WireStuck`: the arc was commanded off while the wire touched the work and was still feeding, so no burnback
 *   freed it. A process avoids it by stopping the wire a moment before the arc, or by retracting.
 * A fault turns the arc off.
 *
 * **Power** drawn from the mains is the arc power divided by the supply's efficiency: `I·U/η`.
 *
 * Units: metres for distances, m/min for wire speed, volts, amperes, watts, seconds.
 */
class WeldArcModel {
  /** Wire feed speeds below this cannot strike or hold an arc, in metres per minute. */
  public static inline var MIN_WIRE_SPEED:Float = 1.0;
  /** How far ahead of the wire tip the work may be for the wire to scratch-start on it, in metres. */
  public static inline var STRIKE_REACH:Float = 0.005;
  /** Time from the strike conditions being met to the arc being established, in seconds. */
  public static inline var IGNITION_DELAY:Float = 0.08;
  /** How long the arc may be commanded without lighting before the supply faults, in seconds. */
  public static inline var NO_ARC_TIMEOUT:Float = 1.0;
  /** The arc's length with the wire tip on the work, in metres. */
  public static inline var NOMINAL_ARC_LENGTH:Float = 0.003;
  /** The longest arc that burns, in metres. */
  public static inline var MAX_ARC_LENGTH:Float = 0.012;
  /** The wire tip touches the work within this distance, in metres. */
  public static inline var TOUCH_TOLERANCE:Float = 0.0005;
  /** Open-circuit sensing voltage with the arc off, in volts. */
  public static inline var SENSE_VOLTAGE:Float = 24.0;
  /** Rise of the arc voltage with arc length, in volts per metre (1.2 V/mm). */
  public static inline var ARC_VOLTAGE_GRADIENT:Float = 1200.0;
  public static inline var MIN_ARC_VOLTAGE:Float = 10.0;
  /**
   * Burn-off constants of 1.2 mm solid steel wire (the reference wire): the arc-root term in mm/s per ampere, and the
   * stickout heating term in mm/s per ampere² per millimetre of stickout. Other wires scale with their cross-section.
   */
  static inline var BURN_ALPHA:Float = 0.27;
  static inline var BURN_BETA:Float = 7.0e-5;
  static inline var REFERENCE_AREA_MM2:Float = 1.1309733552923256;

  public final config:WeldArcConfig;
  /** The latest reading. */
  public var reading(default, null):WeldReading = {arc: false, currentA: 0.0, voltageV: 0.0, touch: false, fault: 0, powerW: 0.0};

  var arc = false;
  var fault:Int = WeldFault.None;
  var ignition = 0.0;
  var waiting = 0.0;

  public function new(config:WeldArcConfig) {
    if (config == null || !(config.maxCurrentA > 0) || !(config.efficiency > 0 && config.efficiency <= 1) ||
        !(config.wireDiameterMm > 0) || !(config.stickoutMm > 0))
      throw "A weld arc model needs a positive rating, wire and stickout and an efficiency in (0, 1]";
    this.config = config;
  }

  /** Current for a wire speed in metres per minute, in amperes: the burn-off law, capped at the supply's rating. */
  public function currentFor(wireSpeed:Float):Float {
    if (!(wireSpeed > 0)) return 0.0;
    var area = Math.PI * config.wireDiameterMm * config.wireDiameterMm / 4.0;
    var rate = wireSpeed * 1000.0 / 60.0;
    // MR = (α·A0/A)·I + (β·l·A0²/A²)·I²: the positive root.
    var scale = REFERENCE_AREA_MM2 / area;
    var linear = BURN_ALPHA * scale;
    var quadratic = BURN_BETA * config.stickoutMm * scale * scale;
    var current = (-linear + Math.sqrt(linear * linear + 4.0 * quadratic * rate)) / (2.0 * quadratic);
    return Math.min(current, config.maxCurrentA);
  }

  /** The arc voltage for an arc of length `arcLength` metres at setpoint `voltageSet`. */
  public static function arcVoltage(voltageSet:Float, arcLength:Float):Float
    return Math.max(MIN_ARC_VOLTAGE, voltageSet + ARC_VOLTAGE_GRADIENT * (arcLength - NOMINAL_ARC_LENGTH));

  /** Forgets the arc and any fault, as a power cycle does. */
  public function reset():Void {
    arc = false;
    fault = WeldFault.None;
    ignition = 0.0;
    waiting = 0.0;
    reading = {arc: false, currentA: 0.0, voltageV: 0.0, touch: false, fault: 0, powerW: 0.0};
  }

  /** Advances the circuit by `dt` seconds under `input` and returns the new reading. */
  public function step(dt:Float, input:WeldArcInput):WeldReading {
    if (!(dt >= 0)) throw "A weld step needs a non-negative time step";
    var touching = input.tipDistance <= TOUCH_TOLERANCE;
    var feeding = input.wireSpeed >= MIN_WIRE_SPEED;
    var arcLength = Math.max(0.0, input.tipDistance) + NOMINAL_ARC_LENGTH;
    if (!input.arcCommanded) {
      if (arc && touching && feeding) fault = WeldFault.WireStuck;
      arc = false;
      ignition = 0.0;
      waiting = 0.0;
      // A fault clears once the arc is off; a stuck wire also needs the torch freed.
      if (fault != WeldFault.None && !(fault == WeldFault.WireStuck && touching)) fault = WeldFault.None;
    } else if (fault == WeldFault.None) {
      if (arc) {
        if (!input.supplyReady || arcLength > MAX_ARC_LENGTH) {
          arc = false;
          fault = WeldFault.ArcLost;
        } else if (!feeding) {
          // Burnback: with the wire stopped the arc eats it back to the contact tip and goes out; the process ended.
          arc = false;
          waiting = 0.0;
        }
      } else {
        waiting += dt;
        var reach = touching || input.wireDistance <= STRIKE_REACH;
        if (input.supplyReady && feeding && reach) ignition += dt;
        else ignition = 0.0;
        if (ignition >= IGNITION_DELAY) {
          arc = true;
          waiting = 0.0;
        } else if (waiting > NO_ARC_TIMEOUT) fault = WeldFault.NoArc;
      }
    }
    if (fault != WeldFault.None) arc = false;
    var current = arc ? currentFor(input.wireSpeed) : 0.0;
    var voltage = arc ? arcVoltage(input.voltageSet, arcLength) : input.supplyReady && !touching ? SENSE_VOLTAGE : 0.0;
    reading = {arc: arc, currentA: current, voltageV: voltage,
      touch: touching && !arc && input.supplyReady, fault: fault,
      powerW: arc ? current * voltage / config.efficiency : 0.0};
    return reading;
  }
}
