package machinekit.routing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class ISO30 ATC spindle. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class AtcSpindle extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("diameter", 125),
    P.length("length", 320),
    P.length("taperDiameter", 31.75),
    P.length("taperLength", 45),
    P.quantity("power", 6000, "W"),
    P.quantity("minRpm", 6000, "rpm"),
    P.quantity("maxRpm", 24000, "rpm"),
    P.length("releaseBore", 80),
    P.length("releaseStroke", 6),
    P.quantity("airPressure", 600000, "Pa"),
    P.quantity("clampForce", 2200, "N")
  ];
  public function new(?v:ComponentValues) {
    super("atc-spindle", "ISO30 ATC spindle", "aluminium 6061", inputs(), v);
    positive([
      "diameter",
      "length",
      "taperDiameter",
      "taperLength",
      "power",
      "minRpm",
      "maxRpm",
      "releaseBore",
      "releaseStroke",
      "airPressure",
      "clampForce"
    ]);
    require(
      n("minRpm") <= n("maxRpm") && n("taperDiameter") < n("diameter")
      && n("taperLength") < n("length") && n("taperDiameter") > n("taperLength") * 7 / 24,
      "invalid speed range or taper envelope"
    );
    releaseCylinder();
    require(releaseForce() > n("clampForce"), "release cylinder cannot overcome assumed clamp force");
    addConnector("gauge", Axis, Solids.axial(0, 0, 0));
    addConnector("mount", Mount, Solids.axial(0, 0, n("length") / 2));
    addConnector("release", Mount, Solids.axial(0, 0, n("length")));
    for (name in["release_air", "purge_air"]) addPort({
      name: name,
      kind: Pneumatic,
      role: Consumer,
      iface: PushIn(6),
      required: true
    }
    );
    addPort({name: "motor", kind: ElectricalPower, role: Consumer, iface: Plug("3-phase-PE", 4), required: true}
    );
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.routing.atc-spindle", inputs(), v -> new AtcSpindle(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    return Solids.cut(Solids.named(Part.cylinder(n("diameter") / 2,
      n("length")), "spindle-body"), [Solids.named(Part.revolve([{
      r: 0,
      z: -1
    }, {
      r: n("taperDiameter") / 2,
      z: -1
    }, {
      r: n("taperDiameter") / 2,
      z: 0
    }, {
      r:(n("taperDiameter") - n("taperLength") * 7 / 24) / 2,
      z: n("taperLength")
    }, {
      r: 0,
      z: n("taperLength")
    }
    ]), "iso30-taper")]);
  }
  /** Reference cylinder body, assembled as a separate BOM component in SR3. */
  public function releaseCylinder():machinekit
    .pneumatic.PneumaticCylinder return new machinekit.pneumatic.PneumaticCylinder(
      "ISO15552",
      n("releaseBore"),
      n("releaseStroke"),
      50
    );
  public function releaseForce():Float return Math.PI * Math.pow(n("releaseBore") / 2000, 2) * n("airPressure");
  /** Motor speed/drive feedback belong to the VFD; this spindle owns clamp I/O. */
  override public function io():MachineIo return {inputs:["clamped","released","tool_present"],outputs:["release","purge"],analogOutputs:[]};
}
