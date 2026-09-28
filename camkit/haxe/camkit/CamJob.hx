package camkit;

import cnckit.CncTool;
import cnckit.CncChannels;
import cnckit.ir.CncGeometry;
import cnckit.ir.CncOp;
import cnckit.ir.CncPoint;
import cnckit.parse.CncSpan;

/** 2.5D toolpath producer. All inputs here are metres and metres/second. */
class CamJob {
  public final safeZ:Float;
  public final spindleRpm:Float;
  var current:CncPoint;
  var selectedTool:Int = -1;
  var operationNumber:Int = 0;
  var ops:Array<CncOp> = [];

  public function new(safeZ:Float, spindleRpm:Float, ?initial:CncPoint) {
    if (!Math.isFinite(safeZ) || !Math.isFinite(spindleRpm) ||
        spindleRpm <= 0.0)
      throw "CAM needs finite safe Z and positive spindle RPM";
    this.safeZ = safeZ;
    this.spindleRpm = spindleRpm;
    current = initial == null ? new CncPoint(0.0, 0.0, 0.0) : initial;
  }

  /** Offset closed profile; `outside` and `inside` use the cutter radius. */
  public function profile(contour:CamContour, tool:CncTool, depth:Float,
      feed:Float, ?side:String = "outside", ?stepDown:Float = 0.002):CamJob {
    require(contour, tool, depth, feed);
    var offset = switch side {
      case "outside": -tool.diameter * 0.5;
      case "inside": tool.diameter * 0.5;
      case "on": 0.0;
      case _: throw 'Unknown CAM profile side "$side"';
    };
    var span = nextSpan();
    selectTool(tool, span);
    var path:Null<CamContour> = null;
    if (side == "outside") contour.inset(offset); // Validates convexity.
    else path = contour.inset(offset);
    for (level in depthLevels(contour.z, depth, stepDown)) {
      if (side == "outside") cutOutside(contour, tool.diameter * 0.5,
        level, feed, span);
      else cutLoop(path, level, feed, span);
    }
    return this;
  }

  /** Repeated inward offsets, no larger than `stepOver`, for convex pockets. */
  public function pocket(contour:CamContour, tool:CncTool, depth:Float,
      feed:Float, stepOver:Float, ?stepDown:Float = 0.002):CamJob {
    require(contour, tool, depth, feed);
    if (!Math.isFinite(stepOver) || stepOver <= 0.0 ||
        stepOver > tool.diameter)
      throw "CAM pocket step-over must be positive and no larger than tool diameter";
    var span = nextSpan();
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
    for (level in depthLevels(contour.z, depth, stepDown))
      for (ring in rings) cutLoop(ring, level, feed, span);
    return this;
  }

  /** Expand hole centres into safe rapid, feed, and retract moves. */
  public function drill(holes:Array<CncPoint>, tool:CncTool, depth:Float,
      retractZ:Float, feed:Float):CamJob {
    if (holes == null || holes.length == 0 || tool == null ||
        !Math.isFinite(depth) || !Math.isFinite(retractZ) ||
        !Math.isFinite(feed) || feed <= 0.0)
      throw "CAM drill needs holes, tool, depth, retract and feed";
    var span = nextSpan();
    selectTool(tool, span);
    for (hole in holes) {
      if (hole == null || !Math.isFinite(hole.x) || !Math.isFinite(hole.y) ||
          !Math.isFinite(hole.z) || depth >= hole.z ||
          retractZ < hole.z || safeZ < retractZ)
        throw "CAM drill depth and retract must be below safe Z";
      rapid(new CncPoint(hole.x, hole.y, safeZ), span);
      rapid(new CncPoint(hole.x, hole.y, retractZ), span);
      feedTo(new CncPoint(hole.x, hole.y, depth), feed, span);
      rapid(new CncPoint(hole.x, hole.y, safeZ), span);
    }
    return this;
  }

  public function finish():CamProgram {
    if (ops.length == 0) throw "CAM job has no operations";
    var result = ops.copy();
    var endSpan = new CncSpan(operationNumber + 1, 1, 0);
    result.push(CncOp.Spindle(CncChannels.SpindleSpeed, 0.0, endSpan));
    result.push(CncOp.Spindle(CncChannels.SpindleDirection, 0.0, endSpan));
    result.push(CncOp.End(endSpan));
    return new CamProgram(result);
  }

  function cutLoop(contour:CamContour, depth:Float, feed:Float,
      span:CncSpan):Void {
    var first = contour.vertices[0];
    rapid(new CncPoint(first.x, first.y, safeZ), span);
    feedTo(new CncPoint(first.x, first.y, depth), feed, span);
    for (i in 1...contour.vertices.length) {
      var next = contour.vertices[i];
      feedTo(new CncPoint(next.x, next.y, depth), feed, span);
    }
    feedTo(new CncPoint(first.x, first.y, depth), feed, span);
    rapid(new CncPoint(first.x, first.y, safeZ), span);
  }

  function cutOutside(contour:CamContour, radius:Float, depth:Float,
      feed:Float, span:CncSpan):Void {
    // A round join keeps the cutter centre exactly one radius from every
    // authored outside vertex. Miter joins would leave material at corners.
    var count = contour.vertices.length;
    var orientation = contour.signedArea > 0.0 ? 1.0 : -1.0;
    var shiftedStarts:Array<CncPoint> = [], shiftedEnds:Array<CncPoint> = [];
    for (i in 0...count) {
      var a = contour.vertices[i], b = contour.vertices[(i + 1) % count];
      var dx = b.x - a.x, dy = b.y - a.y;
      var length = Math.sqrt(dx * dx + dy * dy);
      var nx = orientation * dy * radius / length;
      var ny = -orientation * dx * radius / length;
      shiftedStarts.push(new CncPoint(a.x + nx, a.y + ny, depth));
      shiftedEnds.push(new CncPoint(b.x + nx, b.y + ny, depth));
    }
    var first = shiftedStarts[0];
    rapid(new CncPoint(first.x, first.y, safeZ), span);
    feedTo(first, feed, span);
    for (i in 0...count) {
      feedTo(shiftedEnds[i], feed, span);
      var vertex = contour.vertices[(i + 1) % count];
      var next = shiftedStarts[(i + 1) % count];
      var startAngle = Math.atan2(current.y - vertex.y,
        current.x - vertex.x);
      var endAngle = Math.atan2(next.y - vertex.y, next.x - vertex.x);
      var sweep = endAngle - startAngle;
      if (orientation > 0.0) while (sweep <= 0.0) sweep += 2.0 * Math.PI;
      else while (sweep >= 0.0) sweep -= 2.0 * Math.PI;
      feedGeometry(CncGeometry.Arc(new CncPoint(vertex.x, vertex.y, depth),
        radius, startAngle, sweep), feed, span);
    }
    rapid(new CncPoint(first.x, first.y, safeZ), span);
  }

  function rapid(target:CncPoint, span:CncSpan):Void {
    if (current.distanceTo(target) > 1e-12)
      ops.push(CncOp.Rapid(CncGeometry.Line(current, target), span));
    current = target;
  }

  function feedTo(target:CncPoint, speed:Float, span:CncSpan):Void {
    if (current.distanceTo(target) > 1e-12)
      ops.push(CncOp.Feed(CncGeometry.Line(current, target), speed, 0.0, span));
    current = target;
  }

  function feedGeometry(geometry:CncGeometry, speed:Float,
      span:CncSpan):Void {
    ops.push(CncOp.Feed(geometry, speed, 0.0, span));
    current = cnckit.ir.CncGeometryTools.pointAt(geometry,
      cnckit.ir.CncGeometryTools.length(geometry));
  }

  function selectTool(tool:CncTool, span:CncSpan):Void {
    if (selectedTool != tool.number) {
      if (selectedTool >= 0) {
        ops.push(CncOp.Spindle(CncChannels.SpindleSpeed, 0.0, span));
        ops.push(CncOp.Spindle(CncChannels.SpindleDirection, 0.0, span));
      }
      ops.push(CncOp.ToolChange(tool.number, span));
      ops.push(CncOp.Spindle(CncChannels.SpindleDirection, 1.0, span));
      ops.push(CncOp.Spindle(CncChannels.SpindleSpeed, spindleRpm, span));
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

  function nextSpan():CncSpan return new CncSpan(++operationNumber, 1, 0);

  function require(contour:CamContour, tool:CncTool, depth:Float,
      feed:Float):Void {
    if (contour == null || tool == null || tool.diameter <= 0.0 ||
        !Math.isFinite(depth) || depth >= contour.z ||
        !Math.isFinite(feed) || feed <= 0.0 || safeZ <= contour.z)
      throw "CAM cut needs a contour, cutter, depth, feed and safe Z";
  }
}
