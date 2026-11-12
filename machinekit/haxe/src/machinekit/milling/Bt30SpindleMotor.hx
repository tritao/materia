package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import machinekit.component.*;
import machinekit.motion.ServoMotor;

/** Assumed compact 2.2 kW / 8000 rpm motor, including its mounting flange.
 * This is not the much larger motor in the Automation Technology kit. */
class Bt30SpindleMotor extends ServoMotor {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.milling.bt30-spindle-motor-study",
      [],
      v -> new Bt30SpindleMotor()
    ).numbered(_ -> "BT30-MOTOR-ASSUMED-2200W-8000RPM"));
  }
  public function new() {
    var speed = 8000 * 2 * Math.PI / 60, torque = 2200 / speed;
    super(
      {
        designation: "ASSUMED-BT30-MOTOR-2200W",
        ratedTorque: torque,
        peakTorque: 2 * torque,
        ratedSpeed: speed,
        maxSpeed: speed,
        rotorInertia: 0.002,
        encoderCounts: 4096,
        bodyDiameter: 90,
        bodyLength: 160,
        shaftDiameter: 20,
        shaftLength: 30,
        electrical: ServoMotor.acWinding(torque, speed)
      },
      Assumed
    );
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues().setToken("material", materialSpec());
  override public function mounting():Null<machinekit.motion.MotorFlange> return flange();

  static function flange():machinekit.motion.MotorFlange return {
    face: 110,
    flangeThickness: 10,
    pilotDiameter: 40,
    pilotHeight: 2,
    holeDiameter: 5.5,
    holes: [{x: -30, y: -30}, {x: 30, y: -30}, {x: 30, y: 30}, {x: -30, y: 30}]
  };
  /** Continuous axial envelope, retaining the narrower body above its flange. */
  override public function axialEnvelope(minOffset:Float, maxOffset:Float, includeShaft:Bool = false):Part {
    if (!Math.isFinite(minOffset)
      || !Math.isFinite(maxOffset) || minOffset > maxOffset) throw "Invalid motor axial envelope";
    var parts = [
      Part.cylinderSpan(rating.bodyDiameter / 2, -rating.bodyLength + minOffset, maxOffset),
      Part.box(110, 110, 10 + maxOffset - minOffset).translated(new Vector(0, 0, -10 + minOffset)),
      Part.cylinderSpan(20, minOffset, 2 + maxOffset)
    ];
    if (includeShaft) parts.push(Part.cylinderSpan(rating.shaftDiameter / 2,
      minOffset, rating.shaftLength + maxOffset));
    return Solids.union(parts);
  }

  /**
   * A round body behind a square mounting flange with a pilot boss on its face; the envelope fills the clearance holes and the
   * pilot bore. The drawn body is the motor's mass model.
   */
  override public function record(b:DocumentBuilder, detail:ComponentDetail):Null<Feature> {
    machinekit.component.PartRecordings.recipeInputs(this, b);
    var flange = flange(), side = flange.face, t = flange.flangeThickness, pilot = flange.pilotDiameter / 2;
    var axis = Vector.Z(), origin = new Vector(0, 0, 0);
    var body = b.label(b.cylinderAlong(origin, axis, rating.bodyDiameter / 2, -rating.bodyLength, -t), "Body");
    var plate = b.label(b.boxBetween(-side / 2, side / 2, -side / 2, side / 2, -t, 0), "Flange");
    var boss = b.label(b.cylinderAlong(origin, axis, pilot, 0, flange.pilotHeight), "Pilot");
    var motor = b.label(b.unionAll([body, plate, boss]), "Motor");
    if (detail == Envelope) return motor;
    var tools:Array<Feature> = [b.cylinderAlong(origin, axis, rating.shaftDiameter / 2 + 0.5, -1, flange.pilotHeight + 1)], labels = ["shaft bore"];
    for (h in flange.holes) {
      tools.push(b.cylinderAlong(new Vector(h.x, h.y, 0), axis, flange.holeDiameter / 2, -t - 1, 1));
      labels.push("flange hole " + (h.y < 0 ? "front " : "back ") + (h.x < 0 ? "left" : "right"));
    }
    return b.label(b.cutAll(motor, tools, labels), "Shaft bore and flange holes");
  }

  override public function massGeometry():Null<Part> return null;
}
