package machinekit.gantry;

import machinekit.motion.CableChain;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.component.ComponentDetail;
import materia.assembly.AssemblyFrames as F;

/** Initial routed chain body. The machine owns the axis coordinate and asks
 * the source chain for posed geometry when updating its preview. */
class GantryCableChain extends MachineComponent {
  public final chain:CableChain;
  static var recipe:Null<ComponentType>;
  public final fixedHost:String;
  public final movingHost:String;
  public final axis:String;
  public final endConnector:String;
  public final positionOffset:Float;
  public function new(
    chain:CableChain,
    fixedHost:String = "fixedHost",
    movingHost:String = "movingHost",
    axis:String = "x",
    positionOffset:Float = 0
  ) {
    super(
      chain.designation + "-ROUTED",
      chain.description,
      chain.materialSpec(),
      true,
      chain.values().setToken("fixedHost",
      fixedHost).setToken("movingHost", movingHost).setToken("axis", axis).setNumber(
        "positionOffset",
        positionOffset
      )
    );
    chain.movingFrame(positionOffset);
    this.positionOffset = positionOffset;
    this.chain = chain;
    this.fixedHost = fixedHost;
    this.movingHost = movingHost;
    this.axis = axis;
    this.endConnector = "chain-" + axis + "-end";
    addConnector("fixed", Mount, F.identity());
    addConnector("moving", Mount, chain.movingFrame(positionOffset));
  }
  public static function recipeType():ComponentType {
    if (recipe == null) {
      var inputs = CableChain.recipeType().parameters().filter(p -> p.name != "material");
      inputs = inputs.concat([
        P.text("fixedHost", "fixedHost"),
        P.text("movingHost", "movingHost"),
        P.text("axis", "x"),
        P.length("positionOffset", 0)
      ]);
      recipe = new ComponentType("machinekit.motion.routed-cable-chain", inputs, v -> {var cv = new ComponentValues();
      for (p in CableChain.recipeType().parameters()) if (v.get(p.name) != null) cv.set(p.name, v.get(p.name));
      return new GantryCableChain(
        new CableChain(cv),
        v.token("fixedHost"),
        v.token("movingHost"),
        v.token("axis"),
        v.number("positionOffset")
      );
      });
    }
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):cadkit.modeling.Part return chain.geometryAt(
    positionOffset,
    detail
  );
}
