package machinekit.structural;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** A cut rectangular tube with end mounting frames; dimensions are in millimetres. */
class RectTubeMember extends MachineComponent {
  public final profile:RectTube;
  public final length:Float;
  public final drilling:Array<materia.casting.InsertHole>;
  public final partId:String;
  static var recipe:Null<ComponentType>;

  public function new(width:Float, depth:Float, wall:Float, length:Float, ?holes:Array<materia.casting.InsertHole>, ?partId:String) {
    if (!Math.isFinite(width + depth + wall + length)
      || !(length > 0)) throw "Tube member needs finite positive dimensions";
    var section = new RectTube(width, depth, wall);
    super('${section.designation}-L$length', "Cut rectangular structural tube", "steel");
    addFacet(new machinekit.component.EngineeringMetadata(new ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    profile = section;
    this.length = length;
    this.partId = partId == null ? "" : partId;
    drilling = holes == null ? [] : InsertHoles.records(InsertHoles.fromNeutral(holes));
    for (hole in drilling) {
      var axis = materia.assembly.AssemblyFrames.transformVector(hole.frame, 0, 0, 1);
      var normal = [axis.x, axis.y, axis.z], position = [hole.frame.x, hole.frame.y, hole.frame.z];
      var index = Math.abs(axis.x) > 0.9 ? 0 : Math.abs(axis.y) > 0.9 ? 1 : 2;
      if (index == 2) throw "Longitudinal tube drilling needs an explicit end detail";
      var low = [-width / 2, -depth / 2, 0.0], size = [width, depth, length];
      if (normal[index] < 0) low[index] += size[index] - wall;
      size[index] = wall;
      materia.casting.InsertStages.validateServiceHole(
        hole,
        {x: low[0], y: low[1], z: low[2], width: size[0], depth: size[1], height: size[2]}
      );
    }
    addConnector("base", Mount, Solids.axial(0, 0, 0));
    addConnector("end", Mount, Solids.axial(0, 0, length));
  }
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var stock = profile.geometry(length);
    return drilling.length == 0 ? stock : Solids.cut(
      stock,
      InsertHoles.facet(drilling, profile.width, profile.height, length).drillingTools()
    );
  }

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.structural.rect-tube-member",
      [
        ComponentRecipeSupport.length("width", 30),
        ComponentRecipeSupport.length("depth", 30),
        ComponentRecipeSupport.length("wall", 2),
        ComponentRecipeSupport.length("length", 750),
        ComponentRecipeSupport.text("holes", InsertHoles.fromNeutral([])),
        ComponentRecipeSupport.text("partId", "")
      ],
      v -> new RectTubeMember(
        v.number("width"),
        v.number("depth"),
        v.number("wall"),
        v.number("length"),
        InsertHoles.records(v.token("holes")),
        v.token("partId")
      )
    ).numbered(v -> v.token("partId") != "" ? 'TUBE-${v.token("partId").toUpperCase()}'
      : 'TUBE-${Dimension.format(v.number("width"))}x${Dimension.format(v.number("depth"))}x${Dimension.format(v.number("wall"))}-L${Dimension.format(v.number("length"))}'
        + (InsertHoles.records(v.token("holes")).length == 0 ? "" : "-DRILLED"))
      .procured(v -> InsertHoles.records(v.token("holes")).length == 0
        ? Buy(null, null, null) : Make("cutting and drilling"));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues().setNumber("width",
    profile.width).setNumber(
      "depth",
      profile.height
    ).setNumber(
      "wall",
      profile.wall
    ).setNumber(
      "length",
      length
    ).setToken(
      "material",
      materialSpec()
    ).setToken(
      "holes",
      InsertHoles.fromNeutral(drilling)
    ).setToken(
      "partId",
      partId
    );
}
