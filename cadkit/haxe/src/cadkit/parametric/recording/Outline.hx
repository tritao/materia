package cadkit.parametric.recording;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.ParametricError;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.recording.DocumentBuilder.OutlineEdge;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

private typedef OutlinePoint = {
  var at:Vector;
  var edge:String;
  var round:Float;
  var roundName:String;
}

/** One corner as drawn: where the edge before it ends and the edge after it starts, and the arc between when it is rounded. */
private typedef Corner = {
  var enter:Vector;
  var leave:Vector;
  var ?centre:Vector;
  var startAngle:Float;
  var endAngle:Float;
  var clockwise:Bool;
}

/**
 * A closed outline drawn point to point in a plane's own x/y, every edge named: the straight edge into each point, and the arc
 * that rounds a corner. Swept by `turned` or `extruded`, each edge's face is named after it (`DocumentBuilder.sweptFace`), so
 * a face keeps its name when the outline gains or loses other edges. A turned part is the outline of its half-section in
 * (radius, z), revolved about the Z axis.
 *
 * The sketch it records is dimensioned as a part drawing is, from the first point: neighbouring edges share their ends, a
 * horizontal or vertical edge says so, each distinct x and each distinct y is one dimension from the first point (named after
 * the vertical or horizontal edge that lies there, `<edge>.x` / `<edge>.y`, or after the edge into a lone point,
 * `<edge>.end.x`), and a rounded corner is an arc of dimensioned radius (`<round>.radius`) tangent to both its edges. Every
 * dimension is a parameter of the sketch, so a part can be changed by editing one; the sketch is fully defined.
 */
class Outline {
  final points:Array<OutlinePoint> = [];
  var closed = false;

  public function new(x:Float, y:Float) points.push({at: new Vector(x, y, 0), edge: "", round: 0, roundName: ""});

  /** A straight edge called `name` from the last point to (x, y); an edge of no length is left out. */
  public function to(name:String, x:Float, y:Float):Outline {
    if (closed) throw new ParametricError("the outline is already closed");
    var last = points[points.length - 1].at;
    if (Math.abs(last.x - x) < 1e-9 && Math.abs(last.y - y) < 1e-9) return this;
    points.push({at: new Vector(x, y, 0), edge: checkedName(name), round: 0, roundName: ""});
    return this;
  }

  /** Rounds the corner at the last point with an arc called `name` of `radius`, tangent to the edges either side of it. */
  public function round(name:String, radius:Float):Outline {
    if (!(radius > 0)) throw new ParametricError("a rounded corner needs a positive radius");
    if (points.length == 1) throw new ParametricError("the outline's first corner cannot be rounded");
    var point = points[points.length - 1];
    point.round = radius;
    point.roundName = checkedName(name);
    return this;
  }

  /** Closes the outline with a straight edge called `name` back to the first point. */
  public function close(name:String):Outline {
    if (closed) throw new ParametricError("the outline is already closed");
    if (points.length < 3) throw new ParametricError("an outline needs at least three points");
    points[0].edge = checkedName(name);
    closed = true;
    return this;
  }

  /** The outline's edges in order, each named, for `DocumentBuilder.outlineSketch` (every point pinned). */
  public function edges():Array<OutlineEdge> {
    var corners = layout(), n = points.length, result:Array<OutlineEdge> = [];
    for (i in 0...n) {
      var from = corners[(i + n - 1) % n].leave, to = corners[i].enter;
      if (to.subtract(from).length() > 1e-9) result.push(Named(points[i].edge, Straight(from, to)));
      var corner = corners[i], centre = corner.centre;
      if (centre != null)
        result.push(Named(points[i].roundName, Arc(centre, points[i].round, corner.startAngle, corner.endAngle, corner.clockwise)));
    }
    return result;
  }

  /** The outline as a fully dimensioned sketch in `plane`. */
  public function sketch(b:DocumentBuilder, plane:Plane):ConstrainedSketchFeature {
    var corners = layout(), n = points.length, sketch = new ConstrainedSketch(plane);
    var at:Map<String, Vector> = new Map();
    function point(id:String, position:Vector):String {
      sketch.addPoint(new SketchPoint(id, position.x, position.y));
      at.set(id, position);
      return id;
    }
    function place(id:String):Vector {
      var position = at.get(id);
      if (position == null) throw new ParametricError('outline point "$id" is missing');
      return position;
    }
    // Each corner's ends: one point where it is sharp, two tangent points and a centre where it is rounded.
    var enterIds:Array<String> = [], leaveIds:Array<String> = [];
    for (i in 0...n) {
      var corner = corners[i];
      enterIds.push(point(corner.centre == null ? 'p$i' : 'p$i.start', corner.enter));
      leaveIds.push(corner.centre == null ? enterIds[i] : point('p$i.end', corner.leave));
    }
    // Points an edge holds level share a class: x along a vertical edge, y along a horizontal one.
    var xClass:Map<String, String> = new Map(), yClass:Map<String, String> = new Map();
    function find(classes:Map<String, String>, id:String):String {
      var parent = classes.get(id);
      return parent == null || parent == id ? id : find(classes, parent);
    }
    var xLabel:Map<String, String> = new Map(), yLabel:Map<String, String> = new Map();
    function level(classes:Map<String, String>, labels:Map<String, String>, from:String, to:String, label:String):Void {
      var ra = find(classes, from), rb = find(classes, to);
      if (ra != rb) classes.set(rb, ra);
      if (!labels.exists(ra)) labels.set(ra, labels.exists(rb) ? labels.get(rb) : label);
    }
    for (i in 0...n) {
      var from = leaveIds[(i + n - 1) % n], to = enterIds[i], name = points[i].edge;
      var a = place(from), c = place(to);
      if (c.subtract(a).length() <= 1e-9) throw new ParametricError('edge "$name" is used up by the rounded corners at its ends');
      sketch.addEntity(SketchEntity.line(name, from, to));
      if (Math.abs(c.x - a.x) < 1e-9) {
        sketch.addConstraint(SketchConstraint.vertical('$name.vertical', name));
        level(xClass, xLabel, from, to, '$name.x');
      } else if (Math.abs(c.y - a.y) < 1e-9) {
        sketch.addConstraint(SketchConstraint.horizontal('$name.horizontal', name));
        level(yClass, yLabel, from, to, '$name.y');
      }
    }
    // A rounded corner: an arc of dimensioned radius tangent to both edges, each tangent point level with its centre
    // across the edge it ends (a construction line square to that edge), so the arc places the point along the edge.
    var arcHolds:Map<String, Bool> = new Map();
    for (i in 0...n) {
      var corner = corners[i], centre = corner.centre;
      if (centre == null) continue;
      var name = points[i].roundName, before = points[i].edge, after = points[(i + 1) % n].edge;
      var beforeVertical = Math.abs(place(enterIds[i]).x - place(leaveIds[(i + n - 1) % n]).x) < 1e-9;
      var afterVertical = Math.abs(place(enterIds[(i + 1) % n]).x - place(leaveIds[i]).x) < 1e-9;
      var beforeHorizontal = Math.abs(place(enterIds[i]).y - place(leaveIds[(i + n - 1) % n]).y) < 1e-9;
      var afterHorizontal = Math.abs(place(enterIds[(i + 1) % n]).y - place(leaveIds[i]).y) < 1e-9;
      // An arc keeps the angles it is drawn with, so it follows its edges only when they are square to the axes.
      if (!((beforeVertical && afterHorizontal) || (beforeHorizontal && afterVertical)))
        throw new ParametricError('rounded corner "$name" must join a horizontal and a vertical edge');
      var middle = point('p$i.centre', centre);
      sketch.addEntity(SketchEntity.arc(name, middle, points[i].round, corner.startAngle, corner.endAngle, corner.clockwise));
      sketch.addConstraint(SketchConstraint.radius('$name.radius', name, points[i].round));
      sketch.addConstraint(SketchConstraint.tangent('$name.tangent-before', before, name));
      sketch.addConstraint(SketchConstraint.tangent('$name.tangent-after', name, after));
      for (end in [{id: enterIds[i], vertical: beforeVertical, side: "start"}, {id: leaveIds[i], vertical: afterVertical, side: "end"}]) {
        var spoke = '$name.${end.side}-radius';
        sketch.addEntity(SketchEntity.line(spoke, middle, end.id, true));
        sketch.addConstraint(end.vertical ? SketchConstraint.horizontal('$spoke.horizontal', spoke)
          : SketchConstraint.vertical('$spoke.vertical', spoke));
        arcHolds.set(end.id + (end.vertical ? "#y" : "#x"), true);
      }
    }
    // The first point is the datum; every other x and y is one dimension from it.
    var origin = leaveIds[0], datum = place(origin);
    sketch.addConstraint(SketchConstraint.fixed("datum", origin));
    var ends:Array<{id:String, edge:String}> = [for (i in 0...n) {id: enterIds[i], edge: points[i].edge}];
    for (i in 0...n) if (leaveIds[i] != enterIds[i]) ends.push({id: leaveIds[i], edge: points[i].roundName});
    var done:Map<String, Bool> = new Map();
    for (axis in ["x", "y"]) {
      var classes = axis == "x" ? xClass : yClass, labels = axis == "x" ? xLabel : yLabel, home = find(classes, origin);
      for (end in ends) {
        var root = find(classes, end.id);
        if (root == home || done.exists(root) || arcHolds.exists(end.id + "#" + axis)) continue;
        done.set(root, true);
        var label = labels.exists(root) ? labels.get(root) : '${end.edge}.end.$axis', position = place(end.id);
        sketch.addConstraint(axis == "x" ? SketchConstraint.horizontalDistance(label, origin, end.id, position.x - datum.x)
          : SketchConstraint.verticalDistance(label, origin, end.id, position.y - datum.y));
      }
      done = new Map();
    }
    return b.constrainedSketch(sketch, new Map());
  }

  /** The outline's half-section in (radius, z) revolved a full turn about the Z axis; its faces are named by `sweptFace`. */
  public function turned(b:DocumentBuilder):Feature
    return b.revolve(sketch(b, Plane.XZ()), new Vector(0, 0, 0), Vector.Z(), 2 * Math.PI);

  /** The outline in `plane` extruded `depth` along its normal (or `direction`). */
  public function extruded(b:DocumentBuilder, plane:Plane, depth:Float, ?direction:Vector):Feature
    return b.extrude(sketch(b, plane), depth, direction);

  /** Every corner's ends and arc, checking that names are distinct and rounded corners leave their edges some length. */
  function layout():Array<Corner> {
    if (!closed) throw new ParametricError("the outline is not closed");
    var n = points.length, corners:Array<Corner> = [], names:Map<String, Bool> = new Map();
    for (point in points) for (name in point.round > 0 ? [point.edge, point.roundName] : [point.edge]) {
      if (names.exists(name)) throw new ParametricError('the outline names two edges "$name"');
      names.set(name, true);
    }
    for (i in 0...n) {
      var point = points[i], corner = point.at;
      if (point.round == 0) {
        corners.push({enter: corner, leave: corner, startAngle: 0, endAngle: 0, clockwise: false});
        continue;
      }
      var before = points[(i + n - 1) % n].at, after = points[(i + 1) % n].at;
      var d1 = before.subtract(corner).normalized(), d2 = after.subtract(corner).normalized();
      var angle = Math.acos(Math.max(-1, Math.min(1, d1.dot(d2))));
      if (angle < 1e-6 || Math.PI - angle < 1e-6) throw new ParametricError('corner "${point.roundName}" is not a corner');
      var reach = point.round / Math.tan(angle / 2);
      var into = corner.add(d1.scale(reach)), out = corner.add(d2.scale(reach));
      var centre = corner.add(d1.add(d2).normalized().scale(point.round / Math.sin(angle / 2)));
      var turn = (corner.x - before.x) * (after.y - corner.y) - (corner.y - before.y) * (after.x - corner.x);
      corners.push({enter: into, leave: out, centre: centre, startAngle: Math.atan2(into.y - centre.y, into.x - centre.x),
        endAngle: Math.atan2(out.y - centre.y, out.x - centre.x), clockwise: turn < 0});
    }
    for (i in 0...n) {
      var before = points[(i + n - 1) % n].at, at = points[i].at;
      var from = corners[(i + n - 1) % n].leave, to = corners[i].enter;
      if (to.subtract(from).length() > at.subtract(before).length() + 1e-9 || to.subtract(from).dot(at.subtract(before)) < -1e-12)
        throw new ParametricError('the rounded corners either side of edge "${points[i].edge}" overlap');
    }
    return corners;
  }

  /** Names keep to the characters a face name carries as they are: letters, digits, `-`, `_` and `.`. */
  static function checkedName(name:String):String {
    if (name == null || name.length == 0) throw new ParametricError("an outline edge needs a name");
    for (i in 0...name.length) {
      var c = name.charCodeAt(i);
      if (!((c >= "a".code && c <= "z".code) || (c >= "A".code && c <= "Z".code) || (c >= "0".code && c <= "9".code)
        || c == "-".code || c == "_".code || c == ".".code)) throw new ParametricError('outline edge name "$name" has a character a face name escapes');
    }
    return name;
  }
}
