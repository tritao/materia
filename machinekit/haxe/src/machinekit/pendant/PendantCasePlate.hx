package machinekit.pendant;

import cadkit.modeling.Part;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.hmi.PanelSolid;

/**
 * The pendant's face plate as an editable recipe. Its inputs are the case's dimensions; the shell and board follow the
 * same values, so editing the plate in the Inspector regenerates the whole pendant. The material is fixed in code.
 */
class PendantCasePlate extends PanelSolid {
  public final size:PendantSize;

  public function new(size:PendantSize) {
    super("PENDANT-PLATE", "pendant face plate", null, "PA12 nylon", b -> {
      // The recipe's inputs: the plate records the case's width and height; thickness, wall and depth shape the whole case.
      for (input in [{name: "thickness", value: size.thickness}, {name: "wall", value: size.wall}, {name: "depth", value: size.depth}])
        b.length(input.name, input.value);
      PendantCaseDocument.recordPlate(b, ReferencePendant.pendantCase(size), true);
    });
    this.size = size;
  }

  static final recipe = new machinekit.cache.Once<ComponentType>();

  public static function recipeType():ComponentType {
    return recipe.get(() -> {
      var reference = ReferencePendant.size();
      return new ComponentType(
        "machinekit.pendant.plate",
        [
          ComponentRecipeSupport.length("width", reference.width),
          ComponentRecipeSupport.length("height", reference.height),
          ComponentRecipeSupport.length("thickness", reference.thickness),
          ComponentRecipeSupport.length("wall", reference.wall),
          ComponentRecipeSupport.length("depth", reference.depth)
        ],
        v -> new PendantCasePlate(sizeOf(v)),
        false,
        false,
        null,
        false
      ).numbered(_ -> "PENDANT-PLATE");
    });
  }

  /** The dimensions a set of recipe inputs describes. */
  public static function sizeOf(values:ComponentValues):PendantSize return {
    width: values.number("width"),
    height: values.number("height"),
    thickness: values.number("thickness"),
    wall: values.number("wall"),
    depth: values.number("depth")
  };

  override public function componentType():Null<ComponentType> return recipeType();

  override public function values():ComponentValues return new ComponentValues().setNumber("width", size.width).setNumber(
    "height",
    size.height
  ).setNumber("thickness", size.thickness).setNumber("wall", size.wall).setNumber("depth", size.depth);
}
