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
    super("PENDANT-PLATE", "pendant face plate", () -> plate(size), "PA12 nylon");
    this.size = size;
  }

  /** The plate as the recorded steps build it, so its faces match the history the scene carries. */
  static function plate(size:PendantSize):Part {
    var recorded = PendantCaseDocument.plate(ReferencePendant.pendantCase(size));
    try {
      var part = recorded.part();
      recorded.document.close();
      return part;
    } catch (error:Dynamic) {
      recorded.document.close();
      throw error;
    }
  }

  static var recipe:Null<ComponentType> = null;

  public static function recipeType():ComponentType {
    if (recipe == null) {
      var reference = ReferencePendant.size();
      recipe = new ComponentType(
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
      );
    }
    return recipe;
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
