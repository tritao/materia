package machinekit.routing;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Assumed full-bore, spring-return vacuum isolation valve with pneumatic actuator.
 * No pressure/flow or failure behaviour is inferred from this nominal envelope. */
class VacuumZoneValve extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("bore", 50),
    P.length("length", 120),
    P.length("diameter", 90),
    P.quantity("ratedVacuum", 40000, "Pa"),
    P.quantity("flow", 300, "m3/h")
  ];
  public function new(?v:ComponentValues) {
    super("vacuum-zone-valve", "vacuum zone isolation valve", "aluminium 6061", inputs(), v);
    positive(["bore", "length", "diameter", "ratedVacuum", "flow"]);
    require(n("bore") < n("diameter"), "bore consumes valve body");
    addConnector("inlet", Mount, Solids.axial(0, 0, 0));
    addConnector("outlet", Mount, Solids.axial(0, 0, n("length")));
    addPort({name: "inlet", kind: Vacuum, role: Consumer, iface: Thread("G2"), required: true, connector: "inlet"}
    );
    addPort({name: "outlet", kind: Vacuum, role: Supply, iface: Thread("G2"), required: true, connector: "outlet"}
    );
    addPort({name: "actuator", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true}
    );
    addBridge("inlet", "outlet");
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.routing.vacuum-zone-valve",
      inputs(),
      v -> new VacuumZoneValve(v)
    );
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part return Solids.cut(
    Solids.named(
      Part.cylinder(
        n("diameter") / 2,
        n("length")
      ),
      "valve-body"
    ),
    [Solids.named(
      Part.cylinderSpan(
        n("bore") / 2,
        -1,
        n("length") + 1
      ),
      "flow-bore"
    )]
  );
  override public function io():MachineIo return {inputs:[], outputs:["open"], analogOutputs:[]};
}
