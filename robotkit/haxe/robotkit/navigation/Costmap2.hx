package robotkit.navigation;

import robotkit.perception.FreeSpaceView;
import robotkit.perception.Obstacle;

/**
 * Traversability and inflation costs over an OccupancyGrid2. Occupied cells,
 * dynamic obstacle disks, and (by default) unknown cells are inflated by the
 * robot radius. A soft cost falls off beyond the inflated boundary.
 *
 * A cell is lethal when a robot centred on it would overlap an obstacle, and
 * blocked (not traversable) when it is lethal or within half a cell diagonal
 * of that: the extra margin keeps a route through cell centres clear wherever
 * in the cell the robot actually is. A robot can therefore stand in a blocked
 * cell without touching anything, and a planner may escape from one through
 * cells that are not lethal.
 */
class Costmap2 {
  public final grid:OccupancyGrid2;
  public final footprintRadiusMeters:Float;
  public final unknownIsBlocked:Bool;
  public final inflationCostDistanceMeters:Float;
  public final inflationCostWeight:Float;
  /**
   * How long (s) an obstacle sensed once stays in the dynamic layer after it was last seen, unless a later
   * observation sees through its place; zero keeps only what the latest observation shows.
   */
  public final obstacleMemorySeconds:Float;
  /** Counts the changes to the costs, so a view of them can tell when to redraw. */
  public var revision(default, null):Int = 0;

  var dynamicObstacles:Array<Obstacle> = [];
  /** What `senseObstacles` has seen and not yet forgotten, with the layer's clock when each was last seen. */
  var remembered:Array<RememberedObstacle> = [];
  var clockSeconds:Float = 0.0;
  /** The grid's own layer, kept apart so a change of dynamic obstacles does not rasterize the grid again. */
  var staticBlocked:Array<Bool>;
  var staticLethal:Array<Bool>;
  var staticCosts:Array<Float>;
  /** The costs in force: the static layer, or a copy of it with the dynamic obstacles drawn in. */
  var blockedValues:Array<Bool>;
  var lethalValues:Array<Bool>;
  var costs:Array<Float>;

  public function new(grid:OccupancyGrid2, footprintRadiusMeters:Float,
      ?unknownIsBlocked:Bool = true,
      ?inflationCostDistanceMeters:Float = 0.5,
      ?inflationCostWeight:Float = 2.0, ?obstacleMemorySeconds:Float = 0.0) {
    if (grid == null || !Math.isFinite(footprintRadiusMeters) ||
        footprintRadiusMeters < 0.0 ||
        !Math.isFinite(inflationCostDistanceMeters) ||
        inflationCostDistanceMeters < 0.0 ||
        !Math.isFinite(inflationCostWeight) || inflationCostWeight < 0.0 ||
        !Math.isFinite(obstacleMemorySeconds) || obstacleMemorySeconds < 0.0)
      throw "Costmap2 requires a grid and finite nonnegative inflation settings";
    this.grid = grid;
    this.footprintRadiusMeters = footprintRadiusMeters;
    this.unknownIsBlocked = unknownIsBlocked;
    this.inflationCostDistanceMeters = inflationCostDistanceMeters;
    this.inflationCostWeight = inflationCostWeight;
    this.obstacleMemorySeconds = obstacleMemorySeconds;
    refresh();
  }

  /** Rebuilds cached costs after the underlying occupancy grid changes. */
  public function refresh():Void {
    for (obstacle in dynamicObstacles) {
      if (obstacle == null) throw "Costmap2 obstacles cannot contain null";
      if (obstacle.detection.frameId != grid.frameId)
        throw 'Dynamic obstacle ${obstacle.detection.id} is in frame ${obstacle.detection.frameId}; expected ${grid.frameId}';
    }
    var count = grid.width * grid.height;
    blockedValues = [for (_ in 0...count) false];
    lethalValues = [for (_ in 0...count) false];
    costs = [for (_ in 0...count) 0.0];
    var halfCellDiagonal = grid.resolutionMeters * Math.sqrt(2.0) * 0.5;
    for (y in 0...grid.height) for (x in 0...grid.width) {
      var value = grid.cell(x, y);
      if (value == OccupancyCell.Occupied ||
          (unknownIsBlocked && value == OccupancyCell.Unknown)) {
        rasterizeObstacle((x + 0.5) * grid.resolutionMeters,
          (y + 0.5) * grid.resolutionMeters, halfCellDiagonal);
      }
    }
    staticBlocked = blockedValues;
    staticLethal = lethalValues;
    staticCosts = costs;
    drawDynamicObstacles();
  }

  /** The costs in force: the grid's layer, plus a disk per dynamic obstacle. */
  function drawDynamicObstacles():Void {
    revision++;
    if (dynamicObstacles.length == 0) {
      blockedValues = staticBlocked;
      lethalValues = staticLethal;
      costs = staticCosts;
      return;
    }
    blockedValues = staticBlocked.copy();
    lethalValues = staticLethal.copy();
    costs = staticCosts.copy();
    for (obstacle in dynamicObstacles) {
      var local = obstacle.detection.pose.relativeTo(grid.origin);
      rasterizeObstacle(local.x, local.y, obstacle.radiusMeters);
    }
  }

  /**
   * Replaces the dynamic obstacle layer and refreshes costs. An unchanged set (a sensor that has not
   * scanned since) costs nothing; a changed one redraws only the disks, not the grid.
   */
  public function setDynamicObstacles(obstacles:Array<Obstacle>):Void {
    if (obstacles == null) throw "Costmap2 obstacles cannot be null";
    for (obstacle in obstacles) {
      if (obstacle == null) throw "Costmap2 obstacles cannot contain null";
      if (obstacle.detection.frameId != grid.frameId)
        throw 'Dynamic obstacle ${obstacle.detection.id} is in frame ${obstacle.detection.frameId}; expected ${grid.frameId}';
    }
    if (sameObstacles(obstacles)) return;
    dynamicObstacles = obstacles.copy();
    drawDynamicObstacles();
  }

  /**
   * Takes one observation's obstacles into the dynamic layer, `elapsedSeconds` after the last. With a memory,
   * an obstacle that is no longer seen stays until the memory runs out, unless `view` shows free space where
   * it stood; one seen again (overlapping its disk) is replaced by the new sighting. Time is the sum of
   * the elapsed durations, so it follows the caller's (simulation) clock.
   */
  public function senseObstacles(seen:Array<Obstacle>, ?view:FreeSpaceView, ?elapsedSeconds:Float = 0.0):Void {
    if (!Math.isFinite(elapsedSeconds) || elapsedSeconds < 0.0) throw "Costmap2 elapsed time must be finite and nonnegative";
    clockSeconds += elapsedSeconds;
    if (obstacleMemorySeconds <= 0.0) {
      setDynamicObstacles(seen);
      return;
    }
    var next = [for (obstacle in seen) new RememberedObstacle(obstacle, clockSeconds)];
    for (old in remembered) {
      if (clockSeconds - old.seenAt > obstacleMemorySeconds) continue;
      var pose = old.obstacle.detection.pose;
      var resighted = false;
      for (obstacle in seen) {
        var dx = obstacle.detection.pose.x - pose.x, dy = obstacle.detection.pose.y - pose.y;
        var reach = obstacle.radiusMeters + old.obstacle.radiusMeters;
        if (dx * dx + dy * dy < reach * reach) { resighted = true; break; }
      }
      if (resighted) continue;
      if (view != null && view.freeAt(pose.x, pose.y, old.obstacle.radiusMeters)) continue;
      next.push(old);
    }
    remembered = next;
    setDynamicObstacles([for (entry in next) entry.obstacle]);
  }

  public function clearDynamicObstacles():Void {
    remembered = [];
    dynamicObstacles = [];
    drawDynamicObstacles();
  }

  /** The dynamic obstacles now in the layer. */
  public function dynamicLayer():Array<Obstacle> return dynamicObstacles.copy();

  function sameObstacles(next:Array<Obstacle>):Bool {
    if (next.length != dynamicObstacles.length) return false;
    for (index in 0...next.length) {
      var a = next[index], b = dynamicObstacles[index];
      if (a.radiusMeters != b.radiusMeters || a.detection.pose.x != b.detection.pose.x ||
          a.detection.pose.y != b.detection.pose.y)
        return false;
    }
    return true;
  }

  public function contains(x:Int, y:Int):Bool return grid.contains(x, y);

  public function isTraversable(x:Int, y:Int):Bool {
    if (!grid.contains(x, y)) return false;
    var index = y * grid.width + x;
    return !blockedValues[index];
  }

  /** True when a robot centred on the cell would overlap an obstacle, or off the grid. */
  public function isLethal(x:Int, y:Int):Bool {
    if (!grid.contains(x, y)) return true;
    return lethalValues[y * grid.width + x];
  }

  /** Returns nonnegative traversal cost, or a large sentinel if blocked. */
  public function cellCost(x:Int, y:Int):Float {
    if (!grid.contains(x, y)) return 1.0e300;
    var index = y * grid.width + x;
    return blockedValues[index] ? 1.0e300 : costs[index];
  }

  function rasterizeObstacle(localX:Float, localY:Float,
      obstacleRadiusMeters:Float):Void {
    var resolution = grid.resolutionMeters;
    var cellRadius = resolution * Math.sqrt(2.0) * 0.5;
    var lethalRadius = obstacleRadiusMeters + footprintRadiusMeters;
    var hardRadius = lethalRadius + cellRadius;
    var extent = hardRadius + inflationCostDistanceMeters;
    var minX = Std.int(Math.floor((localX - extent) / resolution));
    var maxX = Std.int(Math.floor((localX + extent) / resolution));
    var minY = Std.int(Math.floor((localY - extent) / resolution));
    var maxY = Std.int(Math.floor((localY + extent) / resolution));
    if (minX < 0) minX = 0;
    if (minY < 0) minY = 0;
    if (maxX >= grid.width) maxX = grid.width - 1;
    if (maxY >= grid.height) maxY = grid.height - 1;
    for (y in minY...maxY + 1) for (x in minX...maxX + 1) {
      var centerX = (x + 0.5) * resolution;
      var centerY = (y + 0.5) * resolution;
      var dx = centerX - localX;
      var dy = centerY - localY;
      var distance = Math.sqrt(dx * dx + dy * dy);
      var index = y * grid.width + x;
      if (distance <= hardRadius) {
        blockedValues[index] = true;
        if (distance <= lethalRadius) lethalValues[index] = true;
      } else if (inflationCostDistanceMeters > 0.0) {
        var clearance = distance - hardRadius;
        if (clearance < inflationCostDistanceMeters) {
          var cost = inflationCostWeight *
            (1.0 - clearance / inflationCostDistanceMeters);
          if (cost > costs[index]) costs[index] = cost;
        }
      }
    }
  }
}

private class RememberedObstacle {
  public final obstacle:Obstacle;
  public final seenAt:Float;

  public function new(obstacle:Obstacle, seenAt:Float) {
    this.obstacle = obstacle;
    this.seenAt = seenAt;
  }
}
