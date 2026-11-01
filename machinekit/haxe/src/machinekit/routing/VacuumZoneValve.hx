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
    addPort({name: "inlet", kind: Vacuum, role: Consumer, iface: Thread("G2"), required: true, connector: "inlet"});
    addPort({name: "outlet", kind: Vacuum, role: Supply, iface: Thread("G2"), required: true, connector: "outlet"});
    addPort({name: "actuator", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true});
    addBridge("inlet", "outlet");
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.routing.vacuum-zone-valve",
      inputs(),
      v -> new VacuumZoneValve(v)
    ).numbered(v -> 'VAC-VALVE-ASSUMED-${Dimension.format(v.number("bore"))}MM-'
        + '${Dimension.format(v.number("ratedVacuum"))}PA-${Dimension.format(v.number("flow"))}M3H').procured(_ -> Buy(null, null, null)).requiringInputs(["bore"], ["ratedVacuum", "flow"], []);
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  /** A flanged valve body: end flanges and a waisted body with its bolt circle, on the flow bore. Envelope detail is the cylinder, which also sets the mass. */
  override public function geometry(detail:ComponentDetail = Preview):Part {
    if (detail == Envelope) return envelope();
    var r = n("diameter") / 2, l = n("length"), flange = Math.min(0.18 * l, 12);
    var body = Solids.named(
      Part.revolve([
        {r: n("bore") / 2, z: 0.0},
        {r: r, z: 0.0},
        {r: r, z: flange},
        {r: 0.72 * r, z: flange},
        {r: 0.72 * r, z: l - flange},
        {r: r, z: l - flange},
        {r: r, z: l},
        {r: n("bore") / 2, z: l}
      ]),
      "valve-body"
    );
    var bolts = (r + n("bore") / 2) / 2 + (r - n("bore") / 2) * 0.2;
    return Solids.cut(
      body,
      [for (i in 0...4) Solids.named(
        Part.cylinderSpan(
          Math.min(3.0, (r - n("bore") / 2) * 0.15),
          -0.1,
          l + 0.1,
          bolts * Math.cos(Math.PI / 4 + i * Math.PI / 2),
          bolts * Math.sin(Math.PI / 4 + i * Math.PI / 2)
        ),
        "bolt"
      )]
    );
  }

  override public function massGeometry():Null<Part> return envelope();

  function envelope():Part return Solids.cut(
    Solids.named(Part.cylinder(n("diameter") / 2, n("length")), "valve-body"),
    [Solids.named(Part.cylinderSpan(n("bore") / 2, -1, n("length") + 1), "flow-bore")]
  );
  override public function io():MachineIo return {inputs: [], outputs: ["open"], analogOutputs: []};
}
