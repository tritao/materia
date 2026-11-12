package machinekit.milling;

import cadkit.modeling.Vector;
import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.modeling.Plane;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.motion.ServoMotor;
import machinekit.motion.LeadshineServo;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingPulley;

/** Editable drive selection carried by the actual mounting plate recipe.
 * Geometry is a redesign study, not a supplier-qualified mount or enclosure.
 */
class MillSpindleDrivePart extends MachineComponent {
  /** Shared installed Leadshine selection; belt and carrier remain fabricated parts. */
  public static function installed(headHeight:Float):MillSpindleDrivePart {
    var motor = LeadshineServo.model("ELM2L-1500LA100FT-H MS1AE");
    return new MillSpindleDrivePart(new ComponentValues().setToken("motor", motor.designation).setInteger(
      "motorTeeth",
      60
    ).setInteger(
      "spindleTeeth",
      20
    ).setNumber(
      "centres",
      110
    ).setNumber(
      "guardRearLimit",
      178
    ).setNumber(
      "faceZ",
      251 + motor.rating.shaftLength
    ).setNumber(
      "mountBottom",
      Bt30Cartridge.SEAT + headHeight
    ));
  }
  static final recipe = new machinekit.cache.Once<ComponentType>();
  static final guardRecipe = new machinekit.cache.Once<ComponentType>();
  public static function parameters(role:String = "carrier"):Array<ComponentParameter> return [
    P.choice("role", [role], role),
    P.choice(
      "motor",
      ["ASSUMED-BT30-MOTOR-2200W"].concat(LeadshineServo.catalog().designations()),
      "ELM2M-2900LD180FT-H"
    ),
    P.count("motorTeeth", 96),
    P.count("spindleTeeth", 18),
    P.length("centres", 180),
    P.length("faceZ", 330),
    P.length("mountBottom", 140),
    P.length("guardRearLimit", 0)
  ];
  public static function recipeType(role:String = "carrier"):ComponentType {
    if (role == "guard") {
      return guardRecipe.get(() -> new ComponentType(
        "machinekit.milling.spindle-drive-study-guard",
        parameters("guard"),
        v -> new MillSpindleDrivePart(v)
      ).numbered(v -> 'MILL-SPINDLE-DRIVE-${v.token("role").toUpperCase()}'));
    }
    return recipe.get(() -> new ComponentType(
      "machinekit.milling.spindle-drive-study-part",
      parameters(),
      v -> new MillSpindleDrivePart(v)
    ).numbered(v -> 'MILL-SPINDLE-DRIVE-${v.token("role").toUpperCase()}'));
  }
  public function new(?v:ComponentValues) {
    var role = v != null && v.get("role") != null ? v.token("role") : "carrier";
    if (role != "carrier" && role != "guard") throw "Unknown drive part role";
    v = P.resolve(parameters(role), v);
    if (v.integer("motorTeeth") < 8 || v.integer("motorTeeth") > 160 || v.integer("spindleTeeth") < 8
      || v.integer("spindleTeeth") > 160) throw "Drive study pulleys require 8–160 teeth";
    if (v.number("centres") < 100 || v.number("centres") > 400 || v.number("faceZ") - v.number("mountBottom")
      < 42) throw "Drive study needs 100–400 mm centres and bracing space below its plate";
    super("MILL-SPINDLE-DRIVE-" + v.token("role"), "Selectable spindle drive " + v.token("role"), "steel", false, v);
    // Validate tooth counts against actual shaft bores, not just belt pitch.
    motorPulley();
    spindlePulley();
    if (parameterNumber("guardRearLimit") > 0
      && parameterNumber("guardRearLimit") < centres() + motorPulley().pitchDiameter
      / 2 + 5) throw "Guard rear wall needs at least 5 mm clearance beyond the motor pulley pitch circle";
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
  }
  public function motor():ServoMotor return parameterToken("motor")
    == "ASSUMED-BT30-MOTOR-2200W" ? new Bt30SpindleMotor() : LeadshineServo.model(parameterToken("motor"));
  public function motorTeeth():Int return parameterCount("motorTeeth");
  public function spindleTeeth():Int return parameterCount("spindleTeeth");
  public function speedRatio():Float return motorTeeth() / spindleTeeth();
  public function face():Float return parameterNumber("faceZ");
  function mounting():machinekit.motion.MotorFlange {
    var m = motor(), mount = m.mounting();
    if (mount == null) throw 'Motor ${m.designation} has no mounting dimensions';
    return mount;
  }
  public function motorFace():Float return mounting().face;
  public function flangeThickness():Float return mounting().flangeThickness;
  public function pilotDiameter():Float return mounting().pilotDiameter;
  public function pilotHeight():Float return mounting().pilotHeight;
  public function mountingHoles():Array<{x:Float, y:Float}> return mounting().holes;
  public function holeDiameter():Float return mounting().holeDiameter;
  public function screwSize():String {
    var d = holeDiameter();
    return d < 5 ? "M4" : d < 6 ? "M5" : d < 7 ? "M6" : d < 10 ? "M8" : "M12";
  }
  public function motorPulley():TimingPulley return new TimingPulley(
    HTD5M,
    motorTeeth(),
    motor().rating.shaftDiameter,
    18
  );
  public function spindlePulley():TimingPulley return new TimingPulley(HTD5M, spindleTeeth(), 20, 18);
  /** Solve the exact unequal-pulley pitch path for the nearest whole-tooth belt. */
  public function centres():Float {
    var nominal = parameterNumber("centres");
    var target = Math.round(TimingBelt.twoPulley(HTD5M, spindleTeeth(), motorTeeth(), nominal, 15).length / 5) * 5;
    var lo = nominal - 5, hi = nominal + 5;
    for (_ in 0...40) {
      var mid = (lo + hi) / 2;
      if (TimingBelt.twoPulley(HTD5M, spindleTeeth(), motorTeeth(), mid, 15).length < target) lo = mid;
      else hi = mid;
    }
    return (lo + hi) / 2;
  }
  public function guardRadius():Float {
    var screw = machinekit.standard.SocketHeadCapScrew.metric(screwSize(), 20), boltRadius = 0.0;
    for (p in mountingHoles()) boltRadius = Math.max(boltRadius, Math.sqrt(p.x * p.x + p.y * p.y));
    return Math.max(
      Math.max(60, Math.max(motorPulley().pitchDiameter, spindlePulley().pitchDiameter) / 2 + 10),
      boltRadius + screw.spec.headDiameter / 2 + 9
    );
  }
  public function guardBoltX():Float return guardRadius() + 6;
  public function guardBoltY():Float return 90;
  public function plateWidth():Float return Math.max(Math.max(188, motorFace() + 24), 2 * (guardBoltX() + 8));
  public function plateRear():Float return Math.max(180, parameterNumber("centres") + motorFace() / 2 + 10);
  /** A box `w` x `d` centred on (x, y), from z0 to z1. */
  static function box(b:DocumentBuilder, w:Float, d:Float, z0:Float, z1:Float, x:Float = 0, y:Float = 0):Feature
    return b.boxBetween(x - w / 2, x + w / 2, y - d / 2, y + d / 2, z0, z1);

  /** A hole of radius `r` up Z through (x, y), from z0 to z1. */
  static function hole(b:DocumentBuilder, r:Float, z0:Float, z1:Float, x:Float, y:Float):Feature
    return b.cylinderAlong(new Vector(x, y, 0), Vector.Z(), r, z0, z1);

  /** A slot of radius `r` along Y centred on (x, y), from z0 to z1: it covers the whole-tooth centre correction and the
   * full +/-5 mm belt adjustment. */
  static function slot(b:DocumentBuilder, r:Float, y:Float, z0:Float, z1:Float, x:Float = 0):Feature
    return b.extrude(b.slot(20 + 2 * r, 2 * r, new Plane(new Vector(x, y, z0), Vector.Y(), Vector.Z())), z1 - z0);

  override public function componentType():Null<ComponentType> return recipeType(parameterToken("role"));
  override public function hasGeometry():Bool return true;
  override public function record(b:DocumentBuilder, detail:ComponentDetail):Null<Feature> {
    PartRecordings.recipeInputs(this, b);
    var front = 34.0, rear = plateRear(), width = plateWidth();
    var bottom = parameterNumber("mountBottom"), plate = face() - 12, nominal = parameterNumber("centres");
    var tools:Array<Feature> = [], labels:Array<String> = [];
    function cut(tool:Feature, label:String):Void {
      tools.push(tool);
      labels.push(label);
    }
    function sided(side:Int):String return side < 0 ? "left" : "right";
    if (parameterToken("role") == "carrier") {
      var parts:Array<Feature> = [b.label(box(b, width, rear - front, plate, face(), 0, (front + rear) / 2), "Motor plate")];
      for (side in [-1, 1]) {
        parts.push(b.label(box(b, 32, 38, bottom, bottom + 10, side * 72, 119), "Foot " + sided(side)));
        parts.push(b.label(box(b, 12, 38, bottom, plate + 1, side * 82, 119), "Web " + sided(side)));
      }
      var body = b.label(b.unionAll(parts), "Carrier");
      // Keep the release bridge and cylinder separate from the drive carrier.
      cut(box(b, 144, 104, plate - 1, face() + 1), "release clearance");
      for (side in [-1, 1]) {
        for (y in [110.0, 130.0]) {
          var at = sided(side) + (y < 120 ? " front" : " rear");
          cut(hole(b, 3.3, bottom - 1, bottom + 11, side * 68, y), 'anchor bolt $at');
          cut(hole(b, 5.5, plate - 1, face() + 1, side * 68, y), 'anchor access $at');
        }
        cut(hole(b, 1.65, plate - 1, plate + 8, side * guardBoltX(), guardBoltY()), "guard tap " + sided(side));
        cut(hole(b, 8, plate - 1, face() + 1, side * 60, 35), "reaction pillar clearance " + sided(side));
      }
      cut(slot(b, (motor().rating.shaftDiameter + 2) / 2, nominal, plate - 1, face() + 1), "shaft slot");
      cut(slot(b, (pilotDiameter() + 0.2) / 2, nominal, face() - pilotHeight() - 0.2, face() + 1), "pilot slot");
      var screw = machinekit.standard.SocketHeadCapScrew.metric(screwSize(), 20);
      var i = 0;
      for (p in mountingHoles()) {
        i++;
        cut(slot(b, holeDiameter() / 2, nominal + p.y, plate - 1, face() + 1, -p.x), 'motor screw slot $i');
        // Keep socket heads clear of the support webs below the plate.
        cut(slot(b, (screw.spec.headDiameter + 1) / 2, nominal + p.y, plate - screw.spec.headHeight - 0.5, plate, -p.x), 'motor screw head slot $i');
      }
      return b.label(b.cutAll(body, tools, labels), "Clearances, anchor bolts and motor slots");
    }
    var radius = guardRadius(), centre = centres(), top = face() - 10, limit = parameterNumber("guardRearLimit");
    function capsule(r:Float, z:Float, h:Float, name:String, inner:Bool = false):Feature {
      var body = b.extrude(b.slot(centre + 2 * r, 2 * r, new Plane(new Vector(0, centre / 2, z), Vector.Y(), Vector.Z())), h);
      if (limit <= 0) return b.label(body, name);
      var rear = limit - (inner ? 2 : 0);
      return b.label(b.intersect(body, box(b, 2 * r + 2, rear + r + 1, z - 1, z + h + 1, 0, (rear - r - 1) / 2)), name);
    }
    var wall = b.label(b.cutAll(capsule(radius, 250, top - 250, "Wall"), [capsule(radius - 2, 249, top - 248, "Inside", true)], ["inside"]), "Wall");
    var parts:Array<Feature> = [wall, capsule(radius, top - 2, 2, "Lid")];
    for (side in [-1, 1]) parts.push(b.label(box(b, 20, 20, top - 4, top - 2, side * (radius + 4), guardBoltY()), "Bolt lug " + sided(side)));
    var body = b.label(b.unionAll(parts), "Guard");
    cut(box(b, width + 2, rear - front, top - 3, top + 1, 0, (front + rear) / 2), "plate clearance");
    cut(hole(b, 12, 249, top + 1, 0, 0), "drawbar clearance");
    if (top > MillDrawbarLayout.BRIDGE_BOTTOM - 8) cut(box(b, 144, 104, top - 3, top + 1), "bridge clearance");
    for (x in [-60.0, 60.0]) for (y in [-35.0, 35.0])
      cut(hole(b, 8, 249, top + 1, x, y), "pillar clearance " + (y < 0 ? "front " : "back ") + (x < 0 ? "left" : "right"));
    for (side in [-1, 1]) cut(hole(b, 2.25, top - 5, top + 1, side * guardBoltX(), guardBoltY()), "guard bolt " + sided(side));
    return b.label(b.cutAll(body, tools, labels), "Clearances and guard bolts");
  }
}
