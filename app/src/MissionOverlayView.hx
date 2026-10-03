package app;

import Canvas;
import Color;
import PathBuilder;
import app.MissionPlayer.MissionOverlay;
import robotkit.mobile.Pose2;
import robotkit.navigation.Costmap2;
import robotkit.navigation.MotionGuardState;
import robotkit.perception.Obstacle;

/**
 * Draws a running mission on the floor of the viewport, from the data `MissionPlayer.overlay` hands over
 * each frame: the route being followed (red while the guard holds the base), the edge of the costmap's
 * blocked area, the obstacles the lidar added to it as circles, and the footprint where wheel odometry
 * puts the robot. Lines only, a few hundred a frame: the costmap's edge is rebuilt when its costs change.
 */
class MissionOverlayView {
  /** Height above the floor the lines are drawn at, in metres, to keep clear of it. */
  public static inline var LIFT:Float = 0.02;
  static inline var CIRCLE_SEGMENTS:Int = 12;

  var overlay:Null<MissionOverlay> = null;
  /** The costmap's blocked-area edge as x0, y0, x1, y1 runs, and what it was drawn from. */
  var edge:Array<Float> = [];
  var edgeMap:Null<Costmap2> = null;
  var edgeRevision:Int = -1;
  /** Segments added to the path being built; a path with none cannot be stroked. */
  var appended:Int = 0;

  public function new() {}

  /** What to draw from now on; null draws nothing. */
  public function set(next:Null<MissionOverlay>):Void {
    overlay = next;
    var map = next == null ? null : next.costmap;
    if (map == null) {
      edge = [];
      edgeMap = null;
      edgeRevision = -1;
    } else if (map != edgeMap || map.revision != edgeRevision) {
      edge = blockedEdge(map);
      edgeMap = map;
      edgeRevision = map.revision;
    }
  }

  public function paint(canvas:Canvas, camera:PerspectiveCamera, width:Float, height:Float):Void {
    var drawn = overlay;
    if (drawn == null) return;
    appended = 0;
    var blocked = switch drawn.guard { case Blocked(_): true; case _: false; };

    var wall = new PathBuilder();
    var index = 0;
    while (index < edge.length) {
      segment(wall, camera, width, height, edge[index], edge[index + 1], edge[index + 2], edge[index + 3]);
      index += 4;
    }
    stroke(canvas, wall, Color.rgba(0.95, 0.35, 0.3, 0.55), 1.0);

    var sensed = new PathBuilder();
    for (obstacle in drawn.obstacles) capsule(sensed, camera, width, height, obstacle);
    stroke(canvas, sensed, Color.rgba(1.0, 0.3, 0.85, 0.9), 2.0);

    var route = new PathBuilder();
    for (point in 1...drawn.route.length)
      segment(route, camera, width, height, drawn.route[point - 1].x, drawn.route[point - 1].y,
        drawn.route[point].x, drawn.route[point].y);
    stroke(canvas, route, blocked ? Color.rgba(1.0, 0.2, 0.2, 0.95) : Color.rgba(1.0, 0.8, 0.2, 0.95), 2.0);

    var ghost = new PathBuilder();
    var corners = drawn.outline;
    for (corner in 0...corners.length) {
      var next = corners[(corner + 1) % corners.length];
      segment(ghost, camera, width, height, corners[corner].x, corners[corner].y, next.x, next.y);
    }
    var at = drawn.odometry;
    if (at != null) segment(ghost, camera, width, height, at.x, at.y, at.x + 0.3 * Math.cos(at.yaw), at.y + 0.3 * Math.sin(at.yaw));
    stroke(canvas, ghost, Color.rgba(0.6, 0.85, 1.0, 0.9), 1.5);
  }

  /** An obstacle's outline: two sides along its segment and a half circle round each end. */
  function capsule(path:PathBuilder, camera:PerspectiveCamera, width:Float, height:Float, obstacle:Obstacle):Void {
    var ends = obstacle.ends(), r = obstacle.radiusMeters, yaw = obstacle.detection.pose.yaw;
    var nx = -Math.sin(yaw) * r, ny = Math.cos(yaw) * r;
    segment(path, camera, width, height, ends[0].x + nx, ends[0].y + ny, ends[1].x + nx, ends[1].y + ny);
    segment(path, camera, width, height, ends[0].x - nx, ends[0].y - ny, ends[1].x - nx, ends[1].y - ny);
    for (end in 0...2) {
      // The cap runs from one side, round the outer end, to the other.
      var centre = ends[end], start = yaw + (end == 1 ? -Math.PI / 2 : Math.PI / 2);
      for (step in 0...CIRCLE_SEGMENTS) {
        var from = start + Math.PI * step / CIRCLE_SEGMENTS;
        var to = start + Math.PI * (step + 1) / CIRCLE_SEGMENTS;
        segment(path, camera, width, height, centre.x + r * Math.cos(from), centre.y + r * Math.sin(from),
          centre.x + r * Math.cos(to), centre.y + r * Math.sin(to));
      }
    }
  }

  /** Strokes the path if any segment reached it, and starts counting for the next. */
  function stroke(canvas:Canvas, path:PathBuilder, color:Color, width:Float):Void {
    if (appended > 0) canvas.strokeTransient(path.build(), color, width);
    appended = 0;
  }

  function segment(path:PathBuilder, camera:PerspectiveCamera, width:Float, height:Float, x0:Float, y0:Float,
      x1:Float, y1:Float):Void {
    var from = camera.project(x0, y0, LIFT, width, height), to = camera.project(x1, y1, LIFT, width, height);
    if (from != null && to != null) {
      path.moveTo(from.x, from.y).lineTo(to.x, to.y);
      appended++;
    }
  }

  /**
   * The edge of the cells the costmap will not plan through, in map coordinates: wherever a blocked and a free
   * cell meet (the grid's border counts as free), merged into runs along each row and column.
   */
  public static function blockedEdge(map:Costmap2):Array<Float> {
    var grid = map.grid;
    var res = grid.resolutionMeters, ox = grid.origin.x, oy = grid.origin.y;
    function blocked(x:Int, y:Int):Bool return grid.contains(x, y) && !map.isTraversable(x, y);
    var result:Array<Float> = [];
    // Horizontal runs on the line between rows y - 1 and y.
    for (y in 0...grid.height + 1) {
      var start = -1;
      for (x in 0...grid.width + 1) {
        var edgeHere = x < grid.width && blocked(x, y - 1) != blocked(x, y);
        if (edgeHere && start < 0) start = x;
        if (!edgeHere && start >= 0) {
          result.push(ox + start * res); result.push(oy + y * res);
          result.push(ox + x * res); result.push(oy + y * res);
          start = -1;
        }
      }
    }
    // Vertical runs on the line between columns x - 1 and x.
    for (x in 0...grid.width + 1) {
      var start = -1;
      for (y in 0...grid.height + 1) {
        var edgeHere = y < grid.height && blocked(x - 1, y) != blocked(x, y);
        if (edgeHere && start < 0) start = y;
        if (!edgeHere && start >= 0) {
          result.push(ox + x * res); result.push(oy + start * res);
          result.push(ox + x * res); result.push(oy + y * res);
          start = -1;
        }
      }
    }
    return result;
  }
}
