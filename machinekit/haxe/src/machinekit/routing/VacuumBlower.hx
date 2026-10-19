package machinekit.routing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class side-channel vacuum blower. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class VacuumBlower extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.quantity("power", 5500, "W"),
    P.quantity("freeFlow", 300, "m3/h"),
    P.quantity("shutoffPressure", 25000, "Pa"),
    P.length("diameter", 400),
    P.length("length", 500),
    P.length("portDiameter", 75)
  ];
  public function new(?v:ComponentValues) {
    super("vacuum-blower", "side-channel vacuum blower", "aluminium 6061", inputs(), v);
    positive(["power", "freeFlow", "shutoffPressure", "diameter", "length", "portDiameter"]);
    require(n("portDiameter") < n("diameter"), "port exceeds body");
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("vacuum", Mount, Solids.axial(0, 0, n("length")));
    addPort({name: "vacuum", kind: Vacuum, role: Supply, iface: Thread("G2"), required: true, connector: "vacuum"}
    );
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.routing.vacuum-blower",
      inputs(),
      v -> new VacuumBlower(v)
    );
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.named(Part.cylinder(n("diameter") / 2, n("length")), "blower-envelope");
  }
  override public function io():MachineIo return {inputs:["ready", "overload"], outputs:["run"], analogOutputs:[]};

}
