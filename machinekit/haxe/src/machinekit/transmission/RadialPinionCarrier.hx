package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.motion.AxisMotor;
import materia.assembly.AssemblyFrames as F;

/** Actual motor-carrying lever. Coordinates: X along the lever, Y radial up,
 * Z along the shaft. The motor's own flange supplies every pilot/bolt cutout.
 * Spring/reducer ratings remain assumptions inherited from the design spec. */
class RadialPinionCarrier extends PreloadedPinionMount {
  public final motor:MachineComponent;
  public final axialCentre:Float;
  public static inline final THICKNESS = 15.0;
  public function new(motor:MachineComponent, spec:PreloadedPinionMount, axialCentre:Float) {
    super(spec.values().setToken("layout",
      "radial-fork").setNumber("leverArm", 80).setNumber("springArm", 56).setNumber(
        "thickness",
        THICKNESS
      ).setNumber(
        "pinionAxialOffset",
        axialCentre - THICKNESS
      ));
    if (axialCentre <= THICKNESS) throw "Pinion must lie beyond the motor carrier";
    addFacet(new ProcurementIntent(Buy(null, null, null), PreloadedPinionMount.recipeType().requirements(values())));
    this.motor = motor;
    this.axialCentre = axialCentre;
    setMaterial("steel C45");
    lockManufacturingMaterial("steel C45");
    addConnector("motor", Mount, F.translation(80, 0, 0));
    addConnector("springCup", Mount, F.translation(-56, -10, 7.5));
  }
  override public function componentType():Null<ComponentType> return null;
  /** Canonical boxes; the concentric r14 boss and r7 through bore are separate. */
  public function stockRegions():Array<Array<Float>> {
    var size = AxisMotor.of(motor).face + 12;
    return [
      [80 - size / 2, 80 + size / 2, -size / 2, size / 2, 0, 15],
      [-56.0, 80, 14, 22, 0, 15],
      [-6.0, 6, 0, 22, 0, 15],
      [-68.0, -44, -10, 22, 0, 15],
      [-66.0, -46, -10, 10, -24, 0]
    ];
  }
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var parts = [for (r in stockRegions()) block(r[0], r[1], r[2], r[3], r[4], r[5])];
    parts.insert(3, Part.cylinder(14, 15));
    var stock = Solids.union(parts);
    var cut = AxisMotor.of(motor).mountingCutout(THICKNESS + 2);
    var tool = cut.translated(new Vector(80, 0, -1));
    cut.close();
    return Solids.cut(stock, [tool, Part.cylinder(7, THICKNESS + 2).translated(new Vector(0, 0, -1))]);
  }
  public static function block(x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float):Part {
    var raw = Part.box(x1 - x0, y1 - y0, z1 - z0);
    var placed = raw.translated(new Vector((x0 + x1) / 2, (y0 + y1) / 2, z0));
    raw.close();
    return placed;
  }
}
