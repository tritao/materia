package cadkit.sheetmetal;

import cadkit.modeling.Vector;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.HoleSpec;
import cadkit.parametric.features.HolesFeature;
import materia.assembly.AssemblyFrames;

/**
 * Holes described by `HoleSpec`, cut in the developed blank so the flat pattern carries them. Each hole is found on the walls
 * it passes through when the sheet is evaluated: a wall whose plane its axis crosses square, inside the wall's outline, where
 * one of the hole's cuts spans the wall's whole thickness; that cut's diameter is cut through the wall. A hole that reaches no
 * wall, or that stops inside one (a blind hole or a partial counterbore), cannot be cut in a blank and is an error.
 */
class SheetHolesFeature extends SheetFeature {
  public final source:SheetFeature;
  public final holes:Array<HoleSpec>;

  public function new(source:SheetFeature, holes:Array<HoleSpec>) {
    super();
    if (holes.length == 0) throw new SheetDiagnosis("sheet.no-holes", "A sheet holes feature needs at least one hole");
    for (hole in holes) HolesFeature.validate(hole);
    this.source = source;
    this.holes = holes.copy();
  }

  override public function serializationType():String return "sheet-holes";

  override public function identityLedger():SheetIdentityLedger return source.identityLedger();

  override public function dependencies():Array<FeatureId> return [source.id];

  override public function dependencyFeatures():Array<Feature> return [source];

  override public function buildSheet(context:EvaluationContext):Sheet {
    var graph = source.sheetForEvaluation(context);
    var folded = graph.evaluate(graph.folded()), thickness = graph.thickness();
    var cuts:Array<SheetDevelopedCut> = [];
    for (index in 0...holes.length) {
      var hole = holes[index], name = hole.name != null ? hole.name : 'hole${index + 1}';
      var o = AssemblyFrames.transformPoint(hole.frame, hole.x, hole.y, 0), d = AssemblyFrames.transformVector(hole.frame, 0, 0, 1);
      var centre = new Vector(o.x, o.y, o.z), axis = new Vector(d.x, d.y, d.z);
      var found = 0;
      for (wall in graph.walls()) {
        var plane = folded.wall(wall.id).plane, along = axis.dot(plane.normal);
        if (Math.abs(Math.abs(along) - 1) > 1e-9) continue;
        // Where the axis crosses the wall's two faces (local z = 0 and z = -thickness), as distances along the hole.
        var z = plane.toLocal(centre).z, s0 = -z / along, s1 = (-thickness - z) / along;
        var low = Math.min(s0, s1), high = Math.max(s0, s1);
        var hit = plane.toLocal(centre.add(axis.scale(s0)));
        if (!wall.outline.contains(new Point2(hit.x, hit.y))) continue;
        var diameter = 0.0, touched = false;
        for (cut in HolesFeature.cuts(hole)) {
          if (cut.to <= low + 1e-9 || cut.from >= high - 1e-9) continue;
          touched = true;
          if (cut.from <= low + 1e-9 && cut.to >= high - 1e-9) diameter = Math.max(diameter, cut.diameter);
        }
        if (!touched) continue;
        if (diameter == 0) throw new SheetDiagnosis("sheet.blind-hole", "A hole that stops inside a sheet wall cannot be cut in the blank",
          [name, "wall:" + wall.id]);
        var owner = "feature:" + id.toInt() + ":hole:" + name + ":wall:" + wall.id;
        cuts.push(new SheetDevelopedCut(wall.id, circle(hit.x, hit.y, diameter / 2, name), owner));
        found++;
      }
      if (found == 0) throw new SheetDiagnosis("sheet.hole-misses", "A hole crosses no sheet wall square to it", [name]);
    }
    return SheetCutGeometry.applyEdits(graph, [], cuts);
  }

  /** A full circle as two half arcs, named by the hole. */
  static function circle(x:Float, y:Float, radius:Float, name:String):WallOutline {
    var centre = new Point2(x, y);
    return new WallOutline([Segment.arc(centre, radius, 0, Math.PI), Segment.arc(centre, radius, Math.PI, Math.PI)], null,
      [[name + ":a", name + ":b"]]);
  }
}
