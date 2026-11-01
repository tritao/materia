package machinekit.routing;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

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
    addPort({name: "vacuum", kind: Vacuum, role: Supply, iface: Thread("G2"), required: true, connector: "vacuum"});
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.routing.vacuum-blower",
      inputs(),
      v -> new VacuumBlower(v)
    ).numbered(v -> 'VAC-BLOWER-ASSUMED-${Dimension.format(v.number("power"))}W-'
        + '${Dimension.format(v.number("freeFlow"))}M3H-${Dimension.format(v.number("shutoffPressure"))}PA').procured(_ -> Buy(null, null, null)).requiringInputs(["portDiameter"], ["power", "freeFlow", "shutoffPressure"], []);
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  /**
   * A side-channel blower: the impeller housing with its inlet and outlet ports in the front face, the finned motor
   * behind it and the fan cowl. Envelope detail is its cylinder, which also sets the mass.
   */
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var r = n("diameter") / 2, l = n("length"), port = Math.min(n("portDiameter"), 0.4 * r) / 2;
    if (detail == Envelope) return massGeometry();
    var profile = [{r: 0.0, z: 0.0}, {r: r, z: 0.0}, {r: r, z: 0.3 * l}, {r: 0.62 * r, z: 0.3 * l}];
    // Cooling fins along the motor.
    var fins = 8, pitch = 0.6 * l / fins;
    for (i in 0...fins) {
      var z = 0.32 * l + i * pitch;
      profile = profile.concat([
        {r: 0.62 * r, z: z},
        {r: 0.72 * r, z: z},
        {r: 0.72 * r, z: z + pitch * 0.4},
        {r: 0.62 * r, z: z + pitch * 0.4}
      ]);
    }
    profile = profile.concat([
      {r: 0.62 * r, z: 0.94 * l},
      {r: 0.66 * r, z: 0.94 * l},
      {r: 0.66 * r, z: l},
      {r: 0.0, z: l}
    ]);
    var body = Solids.named(Part.revolve(profile), "blower");
    return Solids.cut(
      body,
      [for (x in [-0.5 * r, 0.5 * r]) Solids.named(Part.cylinderSpan(port, -0.1, 0.12 * l, x, 0), "port")]
    );
  }

  override public function massGeometry():Null<Part> return Solids.named(
    Part.cylinder(n("diameter") / 2, n("length")),
    "blower-envelope"
  );
  override public function io():MachineIo return {inputs: ["ready", "overload"], outputs: ["run"], analogOutputs: []};

}
