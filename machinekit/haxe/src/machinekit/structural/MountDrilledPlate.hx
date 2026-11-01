package machinekit.structural;

import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import cadkit.modeling.Part;
import machinekit.gantry.GantryParts.GantryPlate;

/** Machined host plate. Thin pads remain clearance-only; thick-host taps require
 * their explicit material/engagement policy, never a C45 insert assumption. */
class MountDrilledPlate extends GantryPlate {
  static var recipe:Null<ComponentType>;
  final drilling:Array<materia.casting.InsertHole>;
  final partId:String;
  static function inputs():Array<ComponentParameter> return [
    P.length("width", 100),
    P.length("depth", 500),
    P.length("height", 12),
    P.text("holes", InsertHoles.fromNeutral([])),
    P.text("hostMaterial", "steel"),
    P.text("partId", "")
  ];
  public function new(?overrides:ComponentValues) {
    var v = P.resolve(inputs(), overrides, false);
    super("clearance-drilled host pad", v.number("width"), v.number("depth"), v.number("height"));
    setMaterial(v.token("hostMaterial"));
    partId = v.token("partId");
    drilling = InsertHoles.records(v.token("holes"));
    for (h in drilling) {
      if (h.thread != null) MountHostThreads.validate(h, v.token("hostMaterial"), height);
      materia.casting.InsertStages.validateServiceHole(
        h,
        {x: -width / 2, y: -depth / 2, z: 0.0, width: width, depth: depth, height: height}
      );
    }
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.structural.mount-drilled-plate",
      inputs(),
      v -> new MountDrilledPlate(v),
      false,
      false,
      null,
      false
    ).numbered(v -> v.token("partId") != "" ? 'HOST-PAD-${v.token("partId").toUpperCase()}'
      : 'HOST-PAD-${Dimension.format(v.number("width"))}x${Dimension.format(v.number("depth"))}x${Dimension.format(v.number("height"))}-${materia.project.MaterialLibrary.fromSpec(v.token("hostMaterial")).toUpperCase()}');
    return recipe;
  }
  public function holes():Array<materia.casting.InsertHole
    > return InsertHoles.records(InsertHoles.fromNeutral(drilling));
  override public function componentType():Null<ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues().setNumber("width", width).setNumber(
    "depth",
    depth
  ).setNumber(
    "height",
    height
  ).setToken(
    "holes",
    InsertHoles.fromNeutral(drilling)
  ).setToken(
    "hostMaterial",
    materialSpec()
  ).setToken(
    "partId",
    partId
  );
  override public function geometry(detail:ComponentDetail = Preview):Part return Solids.cut(
    Part.box(width, depth, height),
    InsertHoles.facet(drilling, width, depth, height).drillingTools()
  );
}
