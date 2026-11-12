package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Servo housing with its declared mounting flange and pilot. Existing generic
 * servo envelopes stay unchanged; this part is selected by the linear-axis builder.
 * The rotating steel shaft is a separate part on the driven joint. */
class AxisServoMotor extends ServoMotor {
  final source:ServoMotor;
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public function new(designation:String) {
    var selected = ServoMotor.model(designation);
    if (selected.mounting() == null) throw 'Axis servo needs a declared flange: ' + designation;
    super(selected.rating, selected.ratingsBasis);
    source = selected;
  }
  override public function mounting():Null<MotorFlange> return source.mounting();
  /** The flange is another model of the selected motor, not a separate purchase. */
  override function get_bom():BomItem {
    var item = super.get_bom(), purchased = source.bom;
    item.typeId = purchased.typeId;
    item.valuesKey = purchased.valuesKey;
    item.fingerprint = purchased.fingerprint;
    item.procurement = purchased.procurement;
    item.maturity = purchased.maturity;
    item.requirements = machinekit.component.Procurement.ProcurementCodec.copyRequirements(purchased.requirements);
    return item;
  }
  /** The selected motor's body with this flange and pilot added in front of its face. */
  override public function record(b:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature> {
    machinekit.component.PartRecordings.recipeInputs(this, b);
    var motor = source.record(b, detail), face = source.mounting();
    if (motor == null || face == null) throw 'Axis servo lost its mounting interface';
    var flange = b.label(b.boxBetween(-face.face / 2, face.face / 2, -face.face / 2, face.face / 2, -face.flangeThickness, 0), "Mounting flange");
    var holes:Array<cadkit.parametric.Feature> = [for (hole in face.holes)
      b.cylinderAlong(new cadkit.modeling.Vector(hole.x, hole.y, 0), cadkit.modeling.Vector.Z(), face.holeDiameter / 2, -face.flangeThickness - 1, 1)];
    var axis = new cadkit.modeling.Vector(0, 0, 0), up = cadkit.modeling.Vector.Z();
    var pilot = b.label(b.cutAll(b.cylinderAlong(axis, up, face.pilotDiameter / 2, 0, face.pilotHeight),
      [b.cylinderAlong(axis, up, (rating.shaftDiameter + 0.2) / 2, -1, face.pilotHeight + 1)], ["shaft clearance"]), "Pilot");
    return b.label(b.unionAll([motor, b.label(b.cutAll(flange, holes, [for (k in 0...holes.length) 'mount bolt ${k + 1}']), "Mounting flange holes"),
      pilot]), "Motor with mounting interface");
  }

  /** Mass of the base motor's body with this flange and pilot, as before the body was drawn in detail. */
  override public function massGeometry():Null<Part> return withInterface(source.massGeometry());

  function withInterface(base:Part):Part {
    var face = source.mounting();
    if (face == null) {
      base.close();
      throw 'Axis servo lost its mounting interface';
    }
    return Solids.building([], parts -> {parts.push(base);
    var blank = Part.box(face.face, face.face, face.flangeThickness);
    var flange = blank.translated(new cadkit.modeling.Vector(0, 0, -face.flangeThickness));
    blank.close();
    var holes = [for (k => hole in face.holes) Solids.named(
      Part.cylinderSpan(face.holeDiameter / 2, -face.flangeThickness - 1, 1, hole.x, hole.y),
      "mount-bolt" + (k + 1)
    )];
    parts.push(Solids.cut(Solids.named(flange, "mounting-flange"), holes));
    var pilot = Solids.named(Part.cylinder(face.pilotDiameter / 2, face.pilotHeight), "pilot");
    parts.push(Solids.cut(
      pilot,
      [Solids.named(Part.cylinderSpan((rating.shaftDiameter + 0.2) / 2, -1, face.pilotHeight + 1), "shaft-clearance")]
    ));
    return Solids.union(parts);
    });
  }
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.motion.axis-servo-motor",
      [P.choice("designation", [for (row in ServoMotor.ratings()) row.designation], "GENERIC-HV-SERVO-750W")],
      v -> new AxisServoMotor(v.token("designation"))
    ).numbered(ServoMotor.partNumberFor).procured(ServoMotor.procurementFor).requiring(ServoMotor.requirementsFor));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues().setToken(
    "designation",
    source.rating.designation
  ).setToken(
    "material",
    materialSpec()
  );
}
