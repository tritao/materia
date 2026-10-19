package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class planetary reducer. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class PlanetaryReducer extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.scalar("ratio", 5, 1),
    P.scalar("efficiency", 0.95, 0, 1),
    P.length("diameter", 80),
    P.length("length", 100),
    P.length("bore", 19),
    P.quantity("backlash", 5, "arcmin"),
    P.quantity("torsionalStiffness", 12000, "N m/rad"),
    P.quantity("ratedOutputTorque", 80, "N m"),
    P.quantity("peakOutputTorque", 160, "N m"),
    P.quantity("axialLoad", 1500, "N"),
    P.quantity("radialLoad", 2000, "N"),
    P.quantity("inputInertia", 0.00002, "kg m2")
  ];
  public function new(?v:ComponentValues) {
    super("planetary-reducer", "planetary reducer", "aluminium 6061", inputs(), v);
    positive([
      "ratio",
      "efficiency",
      "diameter",
      "length",
      "bore",
      "torsionalStiffness",
      "ratedOutputTorque",
      "peakOutputTorque",
      "axialLoad",
      "radialLoad",
      "inputInertia"
    ]);
    require(n("bore") < n("diameter") && n("peakOutputTorque") >= n("ratedOutputTorque"), "bore/rating mismatch");
    addConnector("input", Mount, Solids.axial(0, 0, 0));
    addConnector("output", Axis, Solids.axial(0, 0, n("length")));
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.motion.planetary-reducer",
      inputs(),
      v -> new PlanetaryReducer(v)
    );
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.cut(
      Solids.named(Part.cylinder(n("diameter") / 2, n("length")), "housing"),
      [Solids.named(
        Part.cylinderSpan(
          n("bore") / 2,
          -1,
          n("length") + 1
        ),
        "shaft-bore"
      )]
    );
  }
  override public function reducerSpec():Null<ReducerSpec> return {
    ratio:n("ratio"),efficiency:n("efficiency"),inputInertia:n("inputInertia"),inertiaAssumed:true,assumed:true,
    diameter:n("diameter"),length:n("length"),bore:n("bore")
  };
  public function jointTorque(motorTorque:Float):Float return motorTorque * n("ratio") * n("efficiency");
  public function jointSpeed(motorSpeed:Float):Float return motorSpeed / n("ratio");
  public function backlashRadians():Float return n("backlash") * Math.PI /(180 * 60);
  /** SI torsional stiffness on the reducer output shaft. */
  public function outputDeflection(torque:Float):Float return torque / n("torsionalStiffness");
}
