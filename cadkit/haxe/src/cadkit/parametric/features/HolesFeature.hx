package cadkit.parametric.features;

import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.HoleSpec;
import cadkit.parametric.ParametricError;
import materia.assembly.AssemblyFrames;

/**
 * A set of holes cut into a solid in one step: one tool per `HoleSpec`, its faces named by the spec's name, all subtracted
 * by one native cut. The specs are exact (a joint derives them), so the feature holds them as data, not parameters.
 */
class HolesFeature extends Feature {
  public final source:Feature;
  public final holes:Array<HoleSpec>;

  public function new(source:Feature, holes:Array<HoleSpec>) {
    super();
    if (holes.length == 0) throw new ParametricError("a holes feature needs at least one hole");
    for (hole in holes) if (!(hole.diameter > 0) || !(hole.to > hole.from))
      throw new ParametricError('hole ${hole.name ?? "?"} needs a positive diameter and to > from');
    this.source = source;
    this.holes = holes.copy();
  }

  override public function serializationType():String return "holes";

  override public function dependencies():Array<FeatureId> return [source.id];

  override public function dependencyFeatures():Array<Feature> return [source];

  /** The tool for one hole, placed in the part frame and named: the cut `holes` makes for it. */
  public static function tool(hole:HoleSpec, fallbackName:String):Part {
    var cylinder = Part.cylinderSpan(hole.diameter / 2, hole.from, hole.to, hole.x, hole.y);
    var named = cylinder.named(hole.name ?? fallbackName);
    cylinder.close();
    var pose = hole.frame;
    var x = AssemblyFrames.transformVector(pose, 1, 0, 0), z = AssemblyFrames.transformVector(pose, 0, 0, 1);
    var placed = new Part(new Location(new Plane(new Vector(pose.x, pose.y, pose.z), new Vector(x.x, x.y, x.z), new Vector(z.x, z.y,
      z.z))).apply(named.shape));
    named.close();
    return placed;
  }

  override public function evaluate(context:EvaluationContext):EvaluationResult {
    var tools:Array<Part> = [];
    var body = new Part(context.shape(source).cloneShape());
    try {
      var index = 0;
      // Only an unnamed hole takes a number, as MachineComponent.machined numbered them.
      for (hole in holes) tools.push(tool(hole, hole.name != null ? hole.name : 'cut${++index}'));
      var cut = body.subtractAll(tools);
      var shape = cut.shape.cloneShape();
      cut.close();
      body.close();
      for (t in tools) t.close();
      return EvaluationResult.fromShape(shape);
    } catch (error:Dynamic) {
      body.close();
      for (t in tools) t.close();
      throw error;
    }
  }
}
