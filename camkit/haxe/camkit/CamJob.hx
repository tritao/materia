package camkit;

import toolpathkit.path.MoveKind;
import cadkit.Face;
import toolpathkit.tool.Tool;
import cnckit.CncChannels;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import toolpathkit.path.SpindleDirection;

/** 2.5D toolpath producer. All inputs here are metres and metres/second. */
class CamJob {
  public final safeZ:Float;
  public final spindleRpm:Float;
  var current:Point3;
  var selectedTool:Int = -1;
  var operationNumber:Int = 0;
  var ops:Array<ToolpathOp> = [];
  var tools:Array<Tool> = [];

  public function new(safeZ:Float, spindleRpm:Float, ?initial:Point3) {
    if (!Math.isFinite(safeZ) || !Math.isFinite(spindleRpm) ||
        spindleRpm <= 0.0)
      throw "CAM needs finite safe Z and positive spindle RPM";
    this.safeZ = safeZ;
    this.spindleRpm = spindleRpm;
    current = initial == null ? new Point3(0.0, 0.0, 0.0) : initial;
  }

  /** Offset closed profile; `outside` and `inside` use the cutter radius. */
  public function profile(contour:CamContour, tool:Tool, depth:Float,
      feed:Float, ?side:String = "outside", ?stepDown:Float = 0.002,
      ?plungeFeed:Null<Float>, ?featureRef:String):CamJob {
    require(contour, tool, depth, feed);
    var entryFeed = checkedEntryFeed(feed, plungeFeed);
    if (side != "outside" && side != "inside" && side != "on")
      throw 'Unknown CAM profile side "$side"';
    var levels = depthLevels(contour.z, depth, stepDown);
    if (side != "on") offsetPath(contour, tool.diameter * 0.5,
      levels[0], side == "outside");
    var span = nextSpan(featureRef);
    selectTool(tool, span);
    for (level in levels) {
      if (side == "on") cutLoop(contour, level, feed, entryFeed, span);
      else cutOffset(contour, tool.diameter * 0.5, level, feed,
        entryFeed, span, side == "outside");
    }
    return this;
  }

  /** Profile a planar CAD face, completing each hole before releasing its outer edge. */
  public function profileFace(face:Face, tool:Tool, depth:Float,
      feed:Float, ?stepDown:Float = 0.002, ?unit:String = "mm",
      ?chordToleranceMetres:Float = 0.00005,
      ?plungeFeed:Null<Float>):CamJob {
    var boundaries = CamContour.fromFaceBoundaries(face, unit,
      chordToleranceMetres);
    var outer = boundaries[0];
    require(outer, tool, depth, feed);
    checkedEntryFeed(feed, plungeFeed);
    depthLevels(outer.z, depth, stepDown);
    offsetPath(outer, tool.diameter * 0.5, depth, true);
    for (i in 1...boundaries.length) {
      require(boundaries[i], tool, depth, feed);
      depthLevels(boundaries[i].z, depth, stepDown);
      offsetPath(boundaries[i], tool.diameter * 0.5, depth, false);
    }
    for (i in 1...boundaries.length)
      profile(boundaries[i], tool, depth, feed, "inside", stepDown,
        plungeFeed, faceRef(face));
    profile(outer, tool, depth, feed, "outside", stepDown, plungeFeed,
      faceRef(face));
    return this;
  }

  /** Clear a pocket with a separate plunge feed and ramps where spans allow. */
  public function pocket(contour:CamContour, tool:Tool, depth:Float,
      feed:Float, stepOver:Float, ?stepDown:Float = 0.002,
      ?plungeFeed:Null<Float>, ?featureRef:String):CamJob {
    require(contour, tool, depth, feed);
    if (!Math.isFinite(stepOver) || stepOver <= 0.0 ||
        stepOver > tool.diameter)
      throw "CAM pocket step-over must be positive and no larger than tool diameter";
    var entryFeed = checkedEntryFeed(feed, plungeFeed);
    var levels = depthLevels(contour.z, depth, stepDown);
    if (hasConcaveCorner(contour)) {
      var radius = tool.diameter * 0.5;
      var passes = CamPocketPlanner.plan(contour, radius, stepOver);
      offsetPath(contour, radius, levels[0], false);
      var span = nextSpan(featureRef);
      selectTool(tool, span);
      for (index in 0...levels.length) {
        var level = levels[index];
        var prior = index == 0 ? contour.z : levels[index - 1];
        cutPocketPasses(passes, level, prior, contour.z, feed,
          entryFeed, span);
        cutPocketBoundary(contour, radius, level, prior, feed,
          entryFeed, span, false);
      }
      return this;
    }
    var span = nextSpan(featureRef);
    selectTool(tool, span);
    var inset = tool.diameter * 0.5;
    var rounds = 0;
    var rings:Array<CamContour> = [];
    while (rounds < 10000) {
      var path:CamContour;
      try path = contour.inset(inset)
      catch (error:Dynamic) {
        var message = Std.string(error);
        if (message.indexOf("exhausted") >= 0 ||
            message.indexOf("zero area") >= 0 ||
            message.indexOf("distinct vertices") >= 0 ||
            message.indexOf("self-intersects") >= 0) break;
        throw error;
      }
      rings.push(path);
      inset += stepOver;
      rounds++;
    }
    if (rounds == 0) throw "Tool does not fit inside CAM pocket";
    if (rounds >= 10000) throw "CAM pocket exceeds 10000 clearing rings";
    for (index in 0...levels.length) {
      var level = levels[index];
      var prior = index == 0 ? contour.z : levels[index - 1];
      for (ring in rings) cutPocketRing(ring, level, prior, contour.z,
        feed, entryFeed, span);
    }
    return this;
  }

  /** Clear a face around its internal islands, then finish every boundary. */
  public function pocketFace(face:Face, tool:Tool, depth:Float,
      feed:Float, stepOver:Float, ?stepDown:Float = 0.002,
      ?unit:String = "mm", ?chordToleranceMetres:Float = 0.00005,
      ?plungeFeed:Null<Float>):CamJob {
    var boundaries = CamContour.fromFaceBoundaries(face, unit,
      chordToleranceMetres);
    var outer = boundaries[0];
    if (boundaries.length == 1)
      return pocket(outer, tool, depth, feed, stepOver, stepDown,
        plungeFeed, faceRef(face));
    require(outer, tool, depth, feed);
    if (!Math.isFinite(stepOver) || stepOver <= 0.0 ||
        stepOver > tool.diameter)
      throw "CAM pocket step-over must be positive and no larger than tool diameter";
    var entryFeed = checkedEntryFeed(feed, plungeFeed);
    var islands = boundaries.slice(1), radius = tool.diameter * 0.5;
    var levels = depthLevels(outer.z, depth, stepDown);
    for (island in islands) require(island, tool, depth, feed);
    var passes = CamPocketPlanner.plan(outer, radius, stepOver, islands);
    var outerPath = offsetPath(outer, radius, levels[0], false);
    for (island in islands) {
      validatePathAgainstContour(island, outerPath, radius, false);
      var islandPath = offsetPath(island, radius, levels[0], true);
      validatePathAgainstContour(outer, islandPath, radius, true);
      for (other in islands) if (other != island)
        validatePathAgainstContour(other, islandPath, radius, false);
    }
    var span = nextSpan(faceRef(face));
    selectTool(tool, span);
    for (index in 0...levels.length) {
      var level = levels[index];
      var prior = index == 0 ? outer.z : levels[index - 1];
      cutPocketPasses(passes, level, prior, outer.z, feed,
        entryFeed, span);
      for (island in islands)
        cutPocketBoundary(island, radius, level, prior, feed,
          entryFeed, span, true);
      cutPocketBoundary(outer, radius, level, prior, feed,
        entryFeed, span, false);
    }
    return this;
  }

  function cutPocketPasses(passes:Array<{y:Float, left:Float, right:Float}>,
      level:Float, prior:Float, surface:Float, feed:Float,
      plungeFeed:Float, span:Provenance):Void {
    for (pass in passes) {
      var start = new Point3(pass.left, pass.y, level);
      var end = new Point3(pass.right, pass.y, level);
      enterPocket(start, end, prior, surface, feed, plungeFeed, span);
      feedTo(end, feed, span);
      rapid(new Point3(end.x, end.y, safeZ), span);
    }
  }

  function cutPocketRing(contour:CamContour, level:Float, prior:Float,
      surface:Float, feed:Float, plungeFeed:Float, span:Provenance):Void {
    var first = contour.vertices[0], next = contour.vertices[1];
    var start = new Point3(first.x, first.y, level);
    enterPocket(start, new Point3(next.x, next.y, level), prior,
      surface, feed, plungeFeed, span);
    for (i in 1...contour.vertices.length) {
      var point = contour.vertices[i];
      feedTo(new Point3(point.x, point.y, level), feed, span);
    }
    feedTo(start, feed, span);
    rapid(new Point3(start.x, start.y, safeZ), span);
  }

  function cutPocketBoundary(contour:CamContour, radius:Float,
      level:Float, prior:Float, feed:Float, plungeFeed:Float,
      span:Provenance, outside:Bool):Void {
    var path = offsetPath(contour, radius, level, outside);
    var start = GeometryTools.pointAt(path[0], 0.0);
    var next = GeometryTools.pointAt(path[0],
      GeometryTools.length(path[0]));
    enterPocket(start, next, prior, contour.z, feed, plungeFeed, span);
    for (geometry in path) feedGeometry(geometry, feed, span);
    rapid(new Point3(start.x, start.y, safeZ), span);
  }

  /** Approach above stock, then ramp and retrace or plunge at its own feed. */
  function enterPocket(start:Point3, firstEnd:Point3, prior:Float,
      surface:Float, feed:Float, plungeFeed:Float, span:Provenance):Void {
    rapidToSafeXY(start.x, start.y, span);
    rapid(new Point3(start.x, start.y,
      Math.min(safeZ, surface + 0.001)), span);
    feedTo(new Point3(start.x, start.y, prior), plungeFeed, span);
    var drop = prior - start.z;
    var slope = Math.min(0.1, plungeFeed / feed);
    var run = drop / slope;
    var dx = firstEnd.x - start.x, dy = firstEnd.y - start.y;
    var length = Math.sqrt(dx * dx + dy * dy);
    if (drop > 1e-10 && run <= length + 1e-12) {
      var fraction = Math.min(1.0, run / length);
      var rampEnd = new Point3(start.x + dx * fraction,
        start.y + dy * fraction, start.z);
      feedTo(rampEnd, feed, span);
      // The ramp leaves a shallow wedge. Recut it at the requested depth.
      feedTo(start, feed, span);
    } else feedTo(start, plungeFeed, span);
  }

  static function checkedEntryFeed(feed:Float,
      requested:Null<Float>):Float {
    var value:Float = requested == null ? feed * 0.25 : requested;
    if (!Math.isFinite(value) || value <= 0.0 || value > feed)
      throw "CAM plunge feed must be positive and no greater than cutting feed";
    return value;
  }

  static function hasConcaveCorner(contour:CamContour):Bool {
    var vertices = contour.vertices;
    var orientation = contour.signedArea > 0.0 ? 1.0 : -1.0;
    for (i in 0...vertices.length) {
      var a = vertices[i], b = vertices[(i + 1) % vertices.length];
      var c = vertices[(i + 2) % vertices.length];
      var abx = b.x - a.x, aby = b.y - a.y;
      var bcx = c.x - b.x, bcy = c.y - b.y;
      var turn = orientation * (abx * bcy - aby * bcx) /
        (Math.sqrt(abx * abx + aby * aby) *
          Math.sqrt(bcx * bcx + bcy * bcy));
      if (turn < -1e-9) return true;
    }
    return false;
  }

  /** Expand hole centres into safe rapid, feed, and retract moves. */
  public function drill(holes:Array<Point3>, tool:Tool, depth:Float,
      retractZ:Float, feed:Float):CamJob {
    if (holes == null || holes.length == 0 || tool == null ||
        !Math.isFinite(depth) || !Math.isFinite(retractZ) ||
        !Math.isFinite(feed) || feed <= 0.0)
      throw "CAM drill needs holes, tool, depth, retract and feed";
    for (hole in holes) {
      if (hole == null || !Math.isFinite(hole.x) || !Math.isFinite(hole.y) ||
          !Math.isFinite(hole.z) || depth >= hole.z ||
          retractZ < hole.z || safeZ < retractZ)
        throw "CAM drill depth and retract must be below safe Z";
    }
    var span = nextSpan();
    selectTool(tool, span);
    for (hole in holes) {
      rapidToSafeXY(hole.x, hole.y, span);
      rapid(new Point3(hole.x, hole.y, retractZ), span);
      feedTo(new Point3(hole.x, hole.y, depth), feed, span);
      rapid(new Point3(hole.x, hole.y, safeZ), span);
    }
    return this;
  }

  public function finish():CamProgram {
    if (ops.length == 0) throw "CAM job has no operations";
    var result = ops.copy();
    var endSpan = Provenance.cam(operationNumber + 1);
    result.push(ToolpathOp.Spindle(Off, 0.0, endSpan));
    result.push(ToolpathOp.End(endSpan));
    return new CamProgram(result, tools);
  }

  function cutLoop(contour:CamContour, depth:Float, feed:Float,
      plungeFeed:Float,
      span:Provenance):Void {
    var first = contour.vertices[0];
    rapidToSafeXY(first.x, first.y, span);
    feedTo(new Point3(first.x, first.y, depth), plungeFeed, span);
    for (i in 1...contour.vertices.length) {
      var next = contour.vertices[i];
      feedTo(new Point3(next.x, next.y, depth), feed, span);
    }
    feedTo(new Point3(first.x, first.y, depth), feed, span);
    rapid(new Point3(first.x, first.y, safeZ), span);
  }

  function cutOffset(contour:CamContour, radius:Float, depth:Float,
      feed:Float, plungeFeed:Float, span:Provenance, outside:Bool):Void {
    var path = offsetPath(contour, radius, depth, outside);
    var first = toolpathkit.path.GeometryTools.pointAt(path[0], 0.0);
    rapidToSafeXY(first.x, first.y, span);
    feedTo(first, plungeFeed, span);
    for (geometry in path) feedGeometry(geometry, feed, span);
    rapid(new Point3(first.x, first.y, safeZ), span);
  }

  /** Round corners that open toward the cutter and trim the other joins. */
  static function offsetPath(contour:CamContour, radius:Float,
      depth:Float, outside:Bool):Array<PathGeometry> {
    var count = contour.vertices.length;
    var orientation = contour.signedArea > 0.0 ? 1.0 : -1.0;
    var shiftedStarts:Array<Point3> = [], shiftedEnds:Array<Point3> = [];
    var convex:Array<Bool> = [];
    for (i in 0...count) {
      var a = contour.vertices[i], b = contour.vertices[(i + 1) % count];
      var dx = b.x - a.x, dy = b.y - a.y;
      var length = Math.sqrt(dx * dx + dy * dy);
      var direction = outside ? 1.0 : -1.0;
      var nx = direction * orientation * dy * radius / length;
      var ny = -direction * orientation * dx * radius / length;
      shiftedStarts.push(new Point3(a.x + nx, a.y + ny, depth));
      shiftedEnds.push(new Point3(b.x + nx, b.y + ny, depth));
    }
    for (i in 0...count) {
      var previous = (i + count - 1) % count;
      var a = contour.vertices[previous], b = contour.vertices[i];
      var c = contour.vertices[(i + 1) % count];
      var abx = b.x - a.x, aby = b.y - a.y;
      var bcx = c.x - b.x, bcy = c.y - b.y;
      var turn = orientation * (abx * bcy - aby * bcx) /
        (Math.sqrt(abx * abx + aby * aby) *
          Math.sqrt(bcx * bcx + bcy * bcy));
      var rounded = outside ? turn > 1e-9 : turn < -1e-9;
      convex.push(rounded);
      if (rounded) continue;
      if (Math.abs(turn) <= 1e-9) {
        var meeting = shiftedEnds[previous];
        shiftedStarts[i] = meeting;
        continue;
      }
      var p = shiftedStarts[previous], q = shiftedStarts[i];
      var rx = shiftedEnds[previous].x - p.x;
      var ry = shiftedEnds[previous].y - p.y;
      var sx = shiftedEnds[i].x - q.x;
      var sy = shiftedEnds[i].y - q.y;
      var denominator = rx * sy - ry * sx;
      if (Math.abs(denominator) < 1e-14)
        throw "CAM profile offset has a degenerate corner";
      var t = ((q.x - p.x) * sy - (q.y - p.y) * sx) / denominator;
      var u = ((q.x - p.x) * ry - (q.y - p.y) * rx) / denominator;
      if (t < -1e-9 || t > 1.0 + 1e-9 ||
          u < -1e-9 || u > 1.0 + 1e-9)
        throw "CAM profile offset exceeds a narrow feature";
      var meeting = new Point3(p.x + t * rx, p.y + t * ry, depth);
      shiftedEnds[previous] = meeting;
      shiftedStarts[i] = meeting;
    }
    var result:Array<PathGeometry> = [];
    for (i in 0...count) {
      var a = contour.vertices[i], b = contour.vertices[(i + 1) % count];
      var start = shiftedStarts[i], end = shiftedEnds[i];
      var forward = (end.x - start.x) * (b.x - a.x) +
        (end.y - start.y) * (b.y - a.y);
      if (forward <= 1e-12)
        throw "CAM profile offset collapses a narrow feature";
      result.push(PathGeometry.Line(start, end));
      var nextIndex = (i + 1) % count;
      if (!convex[nextIndex]) continue;
      var vertex = contour.vertices[(i + 1) % count];
      var next = shiftedStarts[nextIndex];
      var startAngle = Math.atan2(end.y - vertex.y,
        end.x - vertex.x);
      var endAngle = Math.atan2(next.y - vertex.y, next.x - vertex.x);
      var sweep = endAngle - startAngle;
      var arcDirection = outside ? orientation : -orientation;
      if (arcDirection > 0.0) while (sweep <= 0.0) sweep += 2.0 * Math.PI;
      else while (sweep >= 0.0) sweep -= 2.0 * Math.PI;
      result.push(PathGeometry.Arc(new Point3(vertex.x, vertex.y, depth),
        radius, startAngle, sweep));
    }
    validatePathAgainstContour(contour, result, radius, !outside);
    return result;
  }

  static function validatePathAgainstContour(contour:CamContour,
      path:Array<PathGeometry>, radius:Float, expectedInside:Bool):Void {
    var points = contour.vertices, count = points.length;
    for (geometry in path) {
      var length = GeometryTools.length(geometry);
      switch geometry {
        case Line(start, end):
          for (i in 0...count)
            if (segmentDistance(start, end, points[i],
                points[(i + 1) % count]) < radius - 1e-8)
              throw "CAM profile offset gouges a nonadjacent edge";
        case Arc(_, _, _, sweep):
          var samples = Std.int(Math.max(32,
            Math.ceil(Math.abs(sweep) * 128.0)));
          for (sample in 0...(samples + 1)) {
            var point = GeometryTools.pointAt(geometry,
              length * sample / samples);
            for (i in 0...count)
              if (pointSegmentDistance(point, points[i],
                  points[(i + 1) % count]) < radius - 1e-8)
                throw "CAM profile offset gouges a nonadjacent edge";
          }
        case _: throw "CAM profile offset needs planar lines and arcs";
      }
      var midpoint = GeometryTools.pointAt(geometry, length * 0.5);
      if (insidePolygon(midpoint, points) != expectedInside)
        throw "CAM profile offset crosses its source contour";
    }
  }

  static function insidePolygon(point:Point3, polygon:Array<Point3>):Bool {
    var inside = false;
    for (i in 0...polygon.length) {
      var a = polygon[i], b = polygon[(i + 1) % polygon.length];
      if ((a.y > point.y) != (b.y > point.y) &&
          point.x < a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y))
        inside = !inside;
    }
    return inside;
  }

  static function segmentDistance(a:Point3, b:Point3,
      c:Point3, d:Point3):Float {
    var ax = b.x - a.x, ay = b.y - a.y;
    var cx = d.x - c.x, cy = d.y - c.y;
    var denominator = ax * cy - ay * cx;
    if (Math.abs(denominator) > 1e-14) {
      var t = ((c.x - a.x) * cy - (c.y - a.y) * cx) / denominator;
      var u = ((c.x - a.x) * ay - (c.y - a.y) * ax) / denominator;
      if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) return 0.0;
    }
    return Math.min(Math.min(pointSegmentDistance(a, c, d),
      pointSegmentDistance(b, c, d)),
      Math.min(pointSegmentDistance(c, a, b),
        pointSegmentDistance(d, a, b)));
  }

  static function pointSegmentDistance(point:Point3,
      a:Point3, b:Point3):Float {
    var dx = b.x - a.x, dy = b.y - a.y;
    var t = Math.max(0.0, Math.min(1.0,
      ((point.x - a.x) * dx + (point.y - a.y) * dy) / (dx * dx + dy * dy)));
    var x = point.x - a.x - t * dx, y = point.y - a.y - t * dy;
    return Math.sqrt(x * x + y * y);
  }

  function rapid(target:Point3, span:Provenance):Void {
    if (current.distanceTo(target) > 1e-12) {
      var kind = target.z > current.z + 1e-12 ? MoveKind.Retract :
        Math.abs(target.z - current.z) <= 1e-12 ? MoveKind.Link :
        MoveKind.Rapid;
      ops.push(ToolpathOp.Move(kind, PathGeometry.Line(current, target),
        0.0, 0.0, span));
    }
    current = target;
  }

  function rapidToSafeXY(x:Float, y:Float, span:Provenance):Void {
    rapid(new Point3(current.x, current.y, safeZ), span);
    rapid(new Point3(x, y, safeZ), span);
  }

  function feedTo(target:Point3, speed:Float, span:Provenance):Void {
    if (current.distanceTo(target) > 1e-12) {
      var kind = MoveKind.Cut;
      if (target.z < current.z - 1e-12)
        kind = Math.abs(target.x - current.x) <= 1e-12 &&
          Math.abs(target.y - current.y) <= 1e-12 ?
          MoveKind.Plunge : MoveKind.Ramp;
      ops.push(ToolpathOp.Move(kind, PathGeometry.Line(current, target),
        speed, 0.0, span));
    }
    current = target;
  }

  function feedGeometry(geometry:PathGeometry, speed:Float,
      span:Provenance):Void {
    ops.push(ToolpathOp.Move(MoveKind.Cut, geometry, speed, 0.0, span));
    current = toolpathkit.path.GeometryTools.pointAt(geometry,
      toolpathkit.path.GeometryTools.length(geometry));
  }

  function selectTool(tool:Tool, span:Provenance):Void {
    var known = false;
    for (used in tools) if (used.number == tool.number) {
      if (used != tool) throw 'CAM job uses two different tools numbered ${tool.number}';
      known = true;
    }
    if (!known) tools.push(tool);
    if (selectedTool != tool.number) {
      rapid(new Point3(current.x, current.y, safeZ), span);
      if (selectedTool >= 0) {
        ops.push(ToolpathOp.Spindle(Off, 0.0, span));
      }
      ops.push(ToolpathOp.ToolChange(tool.number, span));
      ops.push(ToolpathOp.Spindle(Clockwise, spindleRpm, span));
      selectedTool = tool.number;
    }
  }

  static function depthLevels(surface:Float, depth:Float,
      stepDown:Float):Array<Float> {
    if (!Math.isFinite(stepDown) || stepDown <= 0.0)
      throw "CAM step-down must be positive";
    var count = Std.int(Math.ceil((surface - depth) / stepDown));
    if (count < 1 || count > 10000)
      throw "CAM cut needs 1..10000 depth levels";
    return [for (level in 1...(count + 1))
      Math.max(depth, surface - level * stepDown)];
  }

  static function faceRef(face:Face):Null<String>
    return face.index < 0 ? null : 'face:${face.index}';

  function nextSpan(?featureRef:String):Provenance
    return Provenance.cam(++operationNumber, featureRef);

  function require(contour:CamContour, tool:Tool, depth:Float,
      feed:Float):Void {
    if (contour == null || tool == null || tool.diameter <= 0.0 ||
        !Math.isFinite(depth) || depth >= contour.z ||
        !Math.isFinite(feed) || feed <= 0.0 || safeZ <= contour.z)
      throw "CAM cut needs a contour, cutter, depth, feed and safe Z";
  }
}
