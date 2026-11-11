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

/** One cylinder a hole is cut with, in the hole's face frame: along +Z from `from` to `to` at the hole's x/y. */
typedef HoleCut = {name:Null<String>, diameter:Float, from:Float, to:Float, recess:Bool};

/**
 * A set of holes cut into a solid in one step: one tool per hole (and one per recess), its faces named by the hole, all
 * subtracted by one native cut. The specs are exact (a joint derives them), so the feature holds them as data.
 *
 * A tool starts `LEAD` mm outside the mouth and runs `LEAD` mm past a through hole's exit (and past the far face for a
 * recess), so no sliver of the face is left where the cutter enters or leaves.
 */
class HolesFeature extends Feature {
  public static inline final LEAD = 1.0;

  public final source:Feature;
  public final holes:Array<HoleSpec>;

  public function new(source:Feature, holes:Array<HoleSpec>) {
    super();
    if (holes.length == 0) throw new ParametricError("a holes feature needs at least one hole");
    for (hole in holes) validate(hole);
    this.source = source;
    this.holes = holes.copy();
  }

  public static function validate(hole:HoleSpec):Void {
    if (!(hole.diameter > 0) || !(hole.depth > 0)) throw new ParametricError('hole ${hole.name ?? "?"} needs a positive diameter and depth');
    var recess = hole.recess;
    if (recess != null && (hole.through != true || !(recess.diameter > hole.diameter) || !(recess.depth > 0) || recess.depth >= hole.depth))
      throw new ParametricError('hole ${hole.name ?? "?"}: a recess needs a through hole and a wider, shallower counterbore');
  }

  override public function serializationType():String return "holes";

  override public function dependencies():Array<FeatureId> return [source.id];

  override public function dependencyFeatures():Array<Feature> return [source];

  /** The cylinders a hole is cut with: the hole itself, then its recess. */
  public static function cuts(hole:HoleSpec):Array<HoleCut> {
    validate(hole);
    var exit = hole.through == true ? hole.depth + LEAD : hole.depth;
    var result:Array<HoleCut> = [{name: hole.name, diameter: hole.diameter, from: -LEAD, to: exit, recess: false}];
    var recess = hole.recess;
    if (recess != null) result.push({name: recess.name, diameter: recess.diameter, from: hole.depth - recess.depth, to: exit, recess: true});
    return result;
  }

  /** Every tool for `holes`, placed in the part frame and named; an unnamed cut is numbered `cut1`, `cut2`, ... in order. */
  public static function tools(holes:Array<HoleSpec>):Array<Part> {
    var result:Array<Part> = [], index = 0;
    try {
      for (hole in holes) for (cut in cuts(hole)) result.push(tool(hole.frame, hole.x, hole.y, cut, cut.name != null ? cut.name : 'cut${++index}'));
    } catch (error:Dynamic) {
      for (part in result) part.close();
      throw error;
    }
    return result;
  }

  static function tool(pose:materia.assembly.AssemblyRecord.AssemblyFrame, x:Float, y:Float, cut:HoleCut, name:String):Part {
    var cylinder = Part.cylinderSpan(cut.diameter / 2, cut.from, cut.to, x, y);
    var named = cylinder.named(name);
    cylinder.close();
    var px = AssemblyFrames.transformVector(pose, 1, 0, 0), pz = AssemblyFrames.transformVector(pose, 0, 0, 1);
    var placed = new Part(new Location(new Plane(new Vector(pose.x, pose.y, pose.z), new Vector(px.x, px.y, px.z),
      new Vector(pz.x, pz.y, pz.z))).apply(named.shape));
    named.close();
    return placed;
  }

  override public function evaluate(context:EvaluationContext):EvaluationResult {
    var body = new Part(context.shape(source).cloneShape());
    var cutters:Array<Part> = [];
    try {
      cutters = tools(holes);
      var cut = body.subtractAll(cutters);
      var shape = cut.shape.cloneShape();
      cut.close();
      body.close();
      for (t in cutters) t.close();
      return EvaluationResult.fromShape(shape);
    } catch (error:Dynamic) {
      body.close();
      for (t in cutters) t.close();
      throw error;
    }
  }
}
