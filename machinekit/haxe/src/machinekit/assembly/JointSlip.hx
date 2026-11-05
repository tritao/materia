package machinekit.assembly;

import machinekit.standard.PropertyClass;
import machinekit.standard.SocketHeadCapScrew;

/** A load the machine declares in newtons (shear) or newton metres (torque): the design peak and, optionally, a momentary one (collision, emergency stop). */
typedef LoadCase = {peak:Float, ?momentary:Float};

/**
 * The load a bolted joint carries, declared by the machine that knows it. `between` names the two members the joint joins (any order); `face`, where
 * those two meet in more than one plane, names a face id on either side. Every bolted and dowelled joint of the pair is one connection.
 * `torque` is about the centroid of the connection's holes (screws and pins), N m; `shear` is a force across the joint, N. All optional
 * values default to the constants of `JointSlip`.
 */
typedef JointLoad = {
  var label:String;
  var between:Array<String>;
  @:optional var face:String;
  @:optional var torque:LoadCase;
  @:optional var shear:LoadCase;
  /** Friction coefficient of the faces, if not `JointSlip.FRICTION`. */
  @:optional var friction:Float;
  /** Share of the proof load the screws are tightened to, if not `JointSlip.UTILISATION`. */
  @:optional var utilisation:Float;
  /** Bearing yield (MPa) of the weakest part a dowel bears on, if not `JointSlip.BEARING_YIELD`. */
  @:optional var bearingYield:Float;
}

/** What one connection's slip check found. Capacities and loads are N m for a torque, N for a shear. */
typedef JointSlipResult = {
  var label:String;
  var kind:String;
  var joints:Array<String>;
  var screws:Int;
  var pins:Int;
  /** Preload of one screw (the smallest in the connection), N. */
  var preload:Float;
  var friction:Float;
  var frictionCapacity:Float;
  /** Dowel capacity: each pin's shear and bearing least value, shared out as the pattern geometry requires; 0 with no pins. */
  var pinCapacity:Float;
  /** Capacity of the shoulders or keys that locate the connection against a shear (`LocatingKey`), N; 0 with none. */
  var keyCapacity:Float;
  /** "pins" where the connection has dowels, "key" for a shoulder or key, "pins+key" for both (friction not counted), else "friction". */
  var primary:String;
  var capacity:Float;
  var peak:Float;
  var momentary:Null<Float>;
  var peakFactor:Float;
  var momentaryFactor:Null<Float>;
  var passes:Bool;
}

/** One hole of a connection in world coordinates, mm. */
typedef SlipPoint = {x:Float, y:Float, z:Float};

/** A bolted joint as the slip check sees it. */
typedef SlipScrew = {id:String, size:String, propertyClass:PropertyClass, points:Array<SlipPoint>};

/** A shoulder or key as the slip check sees it: the height it stands, the length across the thrust and the thickness of its root, mm. */
typedef SlipKey = {id:String, height:Float, length:Float, thickness:Float};

/** A dowelled joint as the slip check sees it: the pin diameter, the shortest engagement in either part, and the holes. */
typedef SlipPin = {id:String, diameter:Float, engagement:Float, points:Array<SlipPoint>};

/**
 * Slip of bolted joints that transmit torque or shear (BOM_PLAN F1). The convention is the machine-design one (Shigley, VDI 2230):
 *
 * - A joint without dowels is a friction grip. Its capacity is the clamp force the screws leave times the friction coefficient: `mu * F * n` for a
 *   shear, `mu * sum(F * r)` for a torque (r from the centroid of the holes). The screws are tightened to `UTILISATION` of the proof load of their
 *   property class (ISO 898-1 proof stress, the stress area of the thread).
 * - A joint with dowels is located and loaded through the pins: the pins carry the shear and the friction is NOT added. A pin only takes load after
 *   the joint has slipped through its clearance, and friction and pin shear are not additive in service; the friction capacity is reported but is
 *   the reserve, not part of the factor. A pin's capacity is the lesser of its shear (single shear at the joint plane, `PIN_SHEAR_STRENGTH`) and the
 *   bearing on the part (`BEARING_YIELD` over diameter times the shortest engagement); a torque is shared by the pins in proportion to their distance
 *   from the centroid, so the farthest pin governs.
 * - A shoulder or key (`LocatingKey`) locates a shear the way a dowel does: its capacity is the lesser of the bearing of the step (`BEARING_YIELD` times
 *   height times length) and the shear of its root (`KEY_SHEAR_STRENGTH` times thickness times length); the friction is not counted. It carries no torque.
 * - The factor is capacity over load. The design peak needs `PEAK_FACTOR`, a momentary load (collision, emergency stop) `MOMENTARY_FACTOR`.
 *
 * The coefficient 0.15 is steel on steel, dry, machined (handbooks give 0.12 to 0.15 for ground and 0.15 to 0.2 for milled steel); it is also used for steel
 * on aluminium, where dry values run higher but the surface embeds, so it is deliberately low. Pin shear strength and bearing yield are ASSUMED figures
 * (ISO 8734 pins are hardened to at least 550 HV; the standard's minimum shear force is not entered), labelled so in the report.
 */
class JointSlip {
  public static inline var FRICTION:Float = 0.15;
  public static inline var UTILISATION:Float = 0.7;
  public static inline var PEAK_FACTOR:Float = 1.5;
  public static inline var MOMENTARY_FACTOR:Float = 1.2;
  /** ASSUMED shear strength of a hardened dowel pin, MPa. */
  public static inline var PIN_SHEAR_STRENGTH:Float = 600.0;
  /** ASSUMED shear strength of a milled step in mild or C45 steel (0.58 of a 240 MPa yield), MPa. */
  public static inline var KEY_SHEAR_STRENGTH:Float = 140.0;
  /** ASSUMED bearing yield of the weaker part (6061-T6 aluminium), MPa. */
  public static inline var BEARING_YIELD:Float = 240.0;

  /** Proof stress (or the 0.2 % yield of stainless) of a property class, MPa (ISO 898-1, up to M16; ISO 3506 for A2-70). */
  public static function proofStress(propertyClass:PropertyClass):Float return switch propertyClass {
    case Steel88: 580.0;
    case Steel109: 830.0;
    case Steel129: 970.0;
    case A270: 450.0;
  };

  /** The tensile stress area of a coarse metric thread, mm2: pi/4 ((d2 + d3) / 2)^2 with d2 = d - 0.6495 p, d3 = d - 1.2269 p (ISO 898-1). */
  public static function stressArea(size:String):Float {
    var spec = SocketHeadCapScrew.catalog().get(size);
    var d2 = spec.diameter - 0.6495 * spec.pitch, d3 = spec.diameter - 1.2269 * spec.pitch, mean = (d2 + d3) / 2;
    return Math.PI / 4 * mean * mean;
  }

  /** The preload one screw leaves at `utilisation` of the proof load, N. */
  public static function preload(size:String, propertyClass:PropertyClass, utilisation:Float):Float return utilisation * proofStress(propertyClass) * stressArea(size);

  static function distance(a:SlipPoint, b:SlipPoint):Float return Math.sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));

  /** One result per kind of load `load` declares (torque, shear), against the screws and pins of the joints it meets. */
  public static function check(load:JointLoad, screws:Array<SlipScrew>, pins:Array<SlipPin>, ?keys:Array<SlipKey>):Array<JointSlipResult> {
    var out:Array<JointSlipResult> = [];
    var mu = load.friction == null ? FRICTION : load.friction, utilisation = load.utilisation == null ? UTILISATION : load.utilisation;
    var yieldMpa = load.bearingYield == null ? BEARING_YIELD : load.bearingYield;
    var all:Array<SlipPoint> = [];
    for (s in screws) for (p in s.points) all.push(p);
    for (s in pins) for (p in s.points) all.push(p);
    if (all.length == 0) return out;
    var keyForce = 0.0;
    if (keys != null) for (k in keys) keyForce += Math.min(yieldMpa * k.height * k.length, KEY_SHEAR_STRENGTH * k.thickness * k.length);
    var centre:SlipPoint = {x: 0.0, y: 0.0, z: 0.0};
    for (p in all) {
      centre.x += p.x / all.length;
      centre.y += p.y / all.length;
      centre.z += p.z / all.length;
    }
    var leastPreload = Math.POSITIVE_INFINITY, nScrews = 0, sumF = 0.0, sumFr = 0.0;
    for (s in screws) {
      var f = preload(s.size, s.propertyClass, utilisation);
      leastPreload = Math.min(leastPreload, f);
      for (p in s.points) {
        nScrews++;
        sumF += f;
        sumFr += f * distance(p, centre) / 1000;
      }
    }
    var nPins = 0, sumR2 = 0.0, maxR = 0.0, pinForce = Math.POSITIVE_INFINITY;
    for (s in pins) {
      var shear = PIN_SHEAR_STRENGTH * Math.PI / 4 * s.diameter * s.diameter, bearing = yieldMpa * s.diameter * s.engagement;
      pinForce = Math.min(pinForce, Math.min(shear, bearing));
      for (p in s.points) {
        var r = distance(p, centre) / 1000;
        nPins++;
        sumR2 += r * r;
        maxR = Math.max(maxR, r);
      }
    }
    function result(kind:String, cases:LoadCase):JointSlipResult {
      var torque = kind == "torque";
      var friction = mu * (torque ? sumFr : sumF);
      var pinCapacity = 0.0;
      if (nPins > 0) pinCapacity = torque ? (maxR > 1e-9 ? pinForce * sumR2 / maxR : 0.0) : pinForce * nPins;
      var keyed = torque ? 0.0 : keyForce, located = pinCapacity + keyed;
      var primary = nPins > 0 && keyed > 0 ? "pins+key" : nPins > 0 ? "pins" : keyed > 0 ? "key" : "friction", capacity = located > 0 ? located : friction;
      var peakFactor = capacity / cases.peak, momentaryFactor = cases.momentary == null ? null : capacity / cases.momentary;
      return {
        label: load.label, kind: kind, joints: [for (s in screws) s.id].concat([for (s in pins) s.id]), screws: nScrews, pins: nPins,
        preload: nScrews > 0 ? leastPreload : 0.0, friction: mu, frictionCapacity: friction, pinCapacity: pinCapacity, keyCapacity: keyed, primary: primary,
        capacity: capacity, peak: cases.peak, momentary: cases.momentary, peakFactor: peakFactor, momentaryFactor: momentaryFactor,
        passes: peakFactor >= PEAK_FACTOR - 1e-9 && (momentaryFactor == null || momentaryFactor >= MOMENTARY_FACTOR - 1e-9)
      };
    }
    if (load.torque != null) out.push(result("torque", load.torque));
    if (load.shear != null) out.push(result("shear", load.shear));
    return out;
  }

  static function r(value:Float, places:Float = 10):Float return Math.round(value * places) / places;

  /** One line of the report: the connection, its screws and pins, the capacity that counts and the factors. */
  public static function describe(slip:JointSlipResult):String {
    var unit = slip.kind == "torque" ? "N m" : "N";
    var text = '${slip.label}: ${slip.screws} screws';
    if (slip.screws > 0) text += ' (preload ${Math.round(slip.preload)} N each, mu ${slip.friction}, friction ${Math.round(slip.frictionCapacity)} $unit)';
    if (slip.pins > 0) text += ', ${slip.pins} dowels carry ${Math.round(slip.pinCapacity)} $unit (friction not counted)';
    if (slip.keyCapacity > 0) text += ', a locating key carries ${Math.round(slip.keyCapacity)} $unit (friction not counted)';
    text += '; ${slip.primary} ${Math.round(slip.capacity)} $unit against peak ${r(slip.peak)} = x${r(slip.peakFactor)}';
    var momentary:Null<Float> = slip.momentary, factor:Null<Float> = slip.momentaryFactor;
    if (momentary != null && factor != null) text += ', momentary ${r(cast momentary)} = x${r(cast factor)}';
    return text + (slip.passes ? "" : " FAILS");
  }
}
