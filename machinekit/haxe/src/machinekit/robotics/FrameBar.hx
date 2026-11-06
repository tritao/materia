package machinekit.robotics;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;

/** Simple rectangular robot tooling bar along local +Z. */
class FrameBar extends MachineComponent {
  public final width:Float;
  public final depth:Float;
  public final length:Float;

  public function new(width:Float, depth:Float, length:Float) {
    if (!Math.isFinite(width) || width <= 0 || !Math.isFinite(depth)
      || depth <= 0 || !Math.isFinite(length) || length <= 0) throw "Frame bar needs positive dimensions";
    super(
      'FRAME-BAR-${Dimension.format(width)}-${Dimension.format(depth)}-${Dimension.format(length)}',
      "Generic end-effector frame bar",
      "aluminium 6061",
      true
    );
    if (Std.isExactType(this, FrameBar)) addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.width = width;
    this.depth = depth;
    this.length = length;
    addConnector("base", Mount, Solids.axial(0, 0, 0));
    addConnector("end", Mount, Solids.axial(0, 0, length));
  }

  static final recipeTypeCache = new machinekit.cache.Once<ComponentType>();

  public static function recipeType():ComponentType {
    return recipeTypeCache.get(() -> new ComponentType(
      "machinekit.robotics.frame-bar",
      [
        ComponentRecipeSupport.length("width", 20),
        ComponentRecipeSupport.length("depth", 20),
        ComponentRecipeSupport.length("length", 100)
      ],
      v -> new FrameBar(v.number("width"), v.number("depth"), v.number("length")),
      true
    ).procured(_ -> Make("cutting and machining")));
  }

  /** Subclasses must declare their own recipe and saved values. */
  override public function componentType():Null<ComponentType> return Std.isExactType(this,
    FrameBar) ? recipeType() : null;

  override public function values():machinekit
    .component.ComponentValues return new machinekit.component.ComponentValues().setNumber(
      "width",
      width
    ).setNumber(
      "depth",
      depth
    ).setNumber(
      "length",
      length
    ).setToken(
      "material",
      materialSpec()
    );

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(width, depth, length);
}
