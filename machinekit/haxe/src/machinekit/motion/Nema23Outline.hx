package machinekit.motion;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.parametric.recording.DocumentBuilder.OutlineEdge;

/**
 * The square NEMA 23 housing section that vendor models share: straight sides `half` from the axis, a convex fillet of radius
 * `fillet` at each side's ends, and a concave relief of radius `relief` about each mounting-hole centre (`hole`, `hole`). Every
 * joint is tangent, so an outline `inset` inwards is the same outline with its sides moved in and its radii changed.
 */
class Nema23Outline {
  public static function edges(half:Float, fillet:Float, hole:Float, relief:Float, inset:Float = 0):Array<OutlineEdge> {
    var a = half - fillet, b = hole - relief - fillet, r = fillet - inset, rr = relief + inset, side = half - inset;
    var result:Array<OutlineEdge> = [];
    for (q in 0...4) {
      var turn = q * Math.PI / 2, c = Math.cos(turn), s = Math.sin(turn);
      function p(x:Float, y:Float):Vector return new Vector(x * c - y * s, x * s + y * c, 0);
      result.push(Straight(p(side, 0), p(side, b)));
      result.push(Arc(p(a, b), r, turn, turn + Math.PI / 2, false));
      result.push(Straight(p(a, b + r), p(hole, hole - rr)));
      result.push(Arc(p(hole, hole), rr, turn - Math.PI / 2, turn - Math.PI, true));
      result.push(Straight(p(hole - rr, hole), p(b + r, a)));
      result.push(Arc(p(b, a), r, turn, turn + Math.PI / 2, false));
      result.push(Straight(p(b, side), p(0, side)));
    }
    return result;
  }

  /** The outline's prism from z = top down by `depth`. */
  public static function prism(b:DocumentBuilder, edges:Array<OutlineEdge>, top:Float, depth:Float):Feature
    return b.extrude(b.outlineSketch(edges, new Plane(new Vector(0, 0, top), Vector.X(), Vector.Z())), depth, Vector.Z().scale(-1));
}
