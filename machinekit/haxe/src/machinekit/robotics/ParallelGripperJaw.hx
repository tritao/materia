package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.CollisionHullFacet;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** One moving carriage. The second jaw uses the same part and the mirrored
 * prismatic coupling; its handed finger seat is derived from `side`. */
class ParallelGripperJaw extends MachineComponent {
  public final spec:ParallelGripperSpec;
  public final side:Int;
  static final recipe = new machinekit.cache.Once<ComponentType>();

  public function new(spec:ParallelGripperSpec, side:Int) {
    if (spec == null || (side != -1 && side != 1)) throw "Gripper jaw needs a size row and side -1 or +1";
    super('GRIPPER-${spec.size}-JAW-${side}', "Reference gripper jaw carriage", "steel");
    this.spec = spec;
    this.side = side;
    addConnector(
      "start",
      Axis,
      AssemblyFrames.alongY(-spec.jawWidth / 2, 0, spec.cylinderAxisZ() - (spec.bodyLength - spec.jawHeight), 1, 0, 0)
    );
    addConnector("cylinderSeat", Axis, AssemblyFrames.alongY(0, 0, 0, 1, 0, 0));
    addConnector(
      "rodMount",
      Mount,
      AssemblyFrames.compose(
        AssemblyFrames.translation(-spec.jawWidth / 2, 0, spec.cylinderAxisZ() - (spec.bodyLength - spec.jawHeight)),
        AssemblyFrames.compose(AssemblyFrames.turnY(Math.PI / 2.0), Solids.axial(0, 0, 0))
      )
    );
    var foot = footSpan();
    addFacet(CollisionHullFacet.fromBoxes([
      [-spec.jawWidth / 2, -spec.bodyDepth / 2, 0, spec.jawWidth / 2, spec.bodyDepth / 2, spec.jawHeight],
      [foot.min, -spec.bodyDepth / 2, -ParallelGripperSpec.GUIDE_TONGUE_HEIGHT, foot.max, spec.bodyDepth / 2, 0]
    ]));
    addConnector("fingerSeat",
      Mount, Solids.axial(-side * (spec.jawWidth - spec.fingerWidth) / 2.0, 0, spec.jawHeight));
  }

  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.robotics.parallel-gripper-jaw",
      [
        ComponentRecipeSupport.count("size", 32),
        ComponentRecipeSupport.length("expectedPartWidth", 60),
        ComponentRecipeSupport.length("openingTravel", 16),
        ComponentRecipeSupport.choice("side", ["left", "right"], "left"),
        ComponentRecipeSupport.optionalLength("fingerWidth")
      ],
      v -> new ParallelGripperJaw(
        new ParallelGripperSpec(
          v.integer("size"),
          v.number("expectedPartWidth"),
          60,
          v.number("openingTravel"),
          false,
          null,
          v.optionalNumber("fingerWidth")
        ),
        v.token("side") == "left" ? -1 : 1
      ),
      true
    ));
  }

  override public function componentType():Null<ComponentType> return Std.isExactType(this,
    ParallelGripperJaw) ? recipeType() : null;

  override public function values():ComponentValues return new ComponentValues().setInteger("size",
    spec.size).setNumber(
      "expectedPartWidth",
      spec.expectedPartWidth
    ).setNumber(
      "openingTravel",
      spec.openingTravel
    ).setToken(
      "side",
      side < 0 ? "left" : "right"
    ).setNumber(
      "fingerWidth",
      spec.fingerWidth
    ).setToken(
      "material",
      materialSpec()
    );

  /**
   * The carriage's foot along x: under the whole jaw and out past its inner face by the guide tongue's reach, so the carriage
   * is one L-shaped piece running on the housing's relieved guide floor.
   */
  function footSpan():{min:Float, max:Float} {
    var outer = side * spec.jawWidth / 2, tip = -side * (spec.jawWidth / 2 + spec.guideTongueReach());
    return {min: Math.min(outer, tip), max: Math.max(outer, tip)};
  }

  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var foot = footSpan();
    return Solids.building([], parts -> {
      parts.push(Solids.named(Part.box(spec.jawWidth, spec.bodyDepth, spec.jawHeight), "jaw"));
      parts.push(Solids.named(Part.box(foot.max - foot.min, spec.bodyDepth, ParallelGripperSpec.GUIDE_TONGUE_HEIGHT)
        .translated(new Vector((foot.min + foot.max) / 2, 0, -ParallelGripperSpec.GUIDE_TONGUE_HEIGHT)), "guide-foot"));
      return Solids.union(parts);
    });
  }
}
