package machinekit.structural;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.casting.InsertHole;
import materia.casting.InsertMountHoleRequirement;
import materia.casting.InsertMountHoleRequirement.InsertMountHoleRequirements;

/** A fabricated C45 mounting member, not a cast-in insert. Shared rail/rack/
 * scale receiver and adjustment steel rebuilds from its source drilling.
 * Welded host pads remain separate parts; this member is bolted to them.
 */
class LinearMountSteel extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public final width:Float;
  public final length:Float;
  public final thickness:Float;
  /** Optional integral foot on the -X edge; Z span, then X thickness. */
  public final flangeWidth:Float;
  public final flangeThickness:Float;
  final drilling:Array<InsertHole>;
  final lineage:Array<InsertMountHoleRequirement>;

  static function inputs():Array<ComponentParameter> return [
    P.text("partId", ""),
    P.length("width", 28),
    P.length("length", 500),
    P.choice("stock", ["C45", "weldable-mild-steel"], "C45"),
    P.length("thickness", 20),
    P.length("flangeWidth", 0),
    P.length("flangeThickness", 0),
    P.choice("role", ["receiver", "adjustment-bar", "carrier", "host-pad"], "receiver"),
    P.text("holes", InsertHoles.fromNeutral([])),
    P.text("mountingRequirements", "[]"),
    P.text("compliance", "null")
  ];

  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.structural.linear-mount-steel",
      inputs(),
      v -> new LinearMountSteel(v),
      false,
      false,
      null,
      false
    ).numbered(v -> v.token("partId") != "" ? 'LM-${v.token("partId").toUpperCase()}' : 'LM-${v.token("stock") == "C45" ? "C45" : "MILD"}-${v.token("role").toUpperCase()}-${Dimension.format(v.number("width"))}x${Dimension.format(v.number("length"))}x${Dimension.format(v.number("thickness"))}')
      .procured(v -> Make(v.token("stock") == "C45" ? "CNC machining" : "steel fabrication and machining")));
  }

  public function new(?overrides:ComponentValues) {
    var v = P.resolve(inputs(), overrides, false);
    width = v.number("width");
    length = v.number("length");
    thickness = v.number("thickness");
    flangeWidth = v.number("flangeWidth");
    flangeThickness = v.number("flangeThickness");
    for (value in [flangeWidth,
      flangeThickness]) if (!Math.isFinite(value) || value < 0) throw "Invalid mount flange stock";
    if ((flangeWidth == 0) != (flangeThickness == 0) || flangeWidth > 0 && flangeWidth
      <= thickness) throw "An integral mount foot needs both dimensions and must extend beyond the receiver";
    for (value in [width,
      length, thickness]) if (!Math.isFinite(value) || value <= 0) throw "Invalid linear-mount steel dimensions";
    drilling = InsertHoles.records(v.token("holes"));
    if (v.token("stock") != "C45") for (hole in drilling) if (hole.thread
      != null) throw "Weldable mild-steel pads cannot acquire a C45 tapped-hole policy";
    lineage = InsertMountHoleRequirements.snapshot(haxe.Json.parse(v.token("mountingRequirements")));
    InsertMountHoleRequirements.references(lineage, drilling);
    var box = {x: -width / 2, y: -length / 2, z: 0.0, width: width, depth: length, height: thickness};
    for (hole in drilling) {
      var valid = false;
      try {
        materia.casting.InsertStages.validateServiceHole(hole, box);
        valid = true;
      } catch (_:Dynamic) {
      }
      if (!valid && flangeWidth > 0) {
        var foot = {
          x: -width / 2 - flangeThickness,
          y: -length / 2,
          z: 0.0,
          width: flangeThickness,
          depth: length,
          height: flangeWidth
        };
        materia.casting.InsertStages.validateServiceHole(hole, foot);
        valid = true;
      }
      if (!valid) materia.casting.InsertStages.validateServiceHole(hole, box);
    }
    v.setToken("holes", InsertHoles.fromNeutral(drilling));
    v.setToken("mountingRequirements", haxe.Json.stringify(InsertMountHoleRequirements.records(lineage)));
    var material = v.token("stock") == "C45" ? "steel C45" : "mild steel sheet";
    super("LINEAR-MOUNT-STEEL", v.token("stock") + " linear-mount " + v.token("role"), material, false, v);
    addFacet(new EngineeringMetadata(new ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    lockManufacturingMaterial(material);
    addFacet(InsertHoles.facet(drilling, width, length, thickness));
    var compliance = LinearMountCompliance.decode(v.token("compliance"));
    if (compliance != null) addFacet(compliance);
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("top", Mount, Solids.axial(0, 0, thickness));
    for (hole in drilling) addConnector(
      "hole-" + hole.id,
      Mount,
      materia.assembly.AssemblyFrames.compose(
        hole.frame,
        materia.assembly.AssemblyFrames.fromRotationMatrix(0, 0, 0, [1, 0, 0, 0, 0, 1, 0, -1, 0])
      )
    );
  }

  public static function receiver(
    width:Float,
    length:Float,
    thickness:Float,
    derived:MountHoleDerivation.DerivedMountHoles
  ):LinearMountSteel return new LinearMountSteel(new ComponentValues().setNumber(
    "width",
    width
  ).setNumber(
    "length",
    length
  ).setNumber(
    "thickness",
    thickness
  ).setToken(
    "holes",
    InsertHoles.fromNeutral(derived.holes)
  ).setToken(
    "mountingRequirements",
    haxe.Json.stringify(InsertMountHoleRequirements.records(derived.requirements))
  ));

  public function mountingRequirements():Array<InsertMountHoleRequirement
    > return InsertMountHoleRequirements.copy(lineage);

  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    // Drilling is structural at every detail; no solid blind hole in Envelope.
    var body = Solids.named(Part.box(width, length, thickness), "steel");
    if (flangeWidth > 0) body = Solids.union([
      body,
      Part.box(flangeThickness, length, flangeWidth).translated(new cadkit.modeling.Vector(
        -width / 2 - flangeThickness / 2,
        0,
        0
      ))
    ]);
    return Solids.cut(body, InsertHoles.facet(drilling, width, length, thickness).drillingTools());
  }
}
