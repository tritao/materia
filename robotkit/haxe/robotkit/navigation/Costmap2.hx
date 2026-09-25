package robotkit.navigation;

import robotkit.perception.Obstacle;

/**
 * Traversability and inflation costs over an OccupancyGrid2. Occupied cells,
 * dynamic obstacle disks, and (by default) unknown cells are inflated by the
 * robot radius. A soft cost falls off beyond the inflated boundary.
 */
class Costmap2 {
  public final grid:OccupancyGrid2;
  public final footprintRadiusMeters:Float;
  public final unknownIsBlocked:Bool;
  public final inflationCostDistanceMeters:Float;
  public final inflationCostWeight:Float;

  var dynamicObstacles:Array<Obstacle> = [];
  var blockedValues:Array<Bool>;
  var costs:Array<Float>;

  public function new(grid:OccupancyGrid2, footprintRadiusMeters:Float,
      ?unknownIsBlocked:Bool = true,
      ?inflationCostDistanceMeters:Float = 0.5,
      ?inflationCostWeight:Float = 2.0) {
    if (grid == null || !Math.isFinite(footprintRadiusMeters) ||
        footprintRadiusMeters < 0.0 ||
        !Math.isFinite(inflationCostDistanceMeters) ||
        inflationCostDistanceMeters < 0.0 ||
        !Math.isFinite(inflationCostWeight) || inflationCostWeight < 0.0)
      throw "Costmap2 requires a grid and finite nonnegative inflation settings";
    this.grid = grid;
    this.footprintRadiusMeters = footprintRadiusMeters;
    this.unknownIsBlocked = unknownIsBlocked;
    this.inflationCostDistanceMeters = inflationCostDistanceMeters;
    this.inflationCostWeight = inflationCostWeight;
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
    for (obstacle in dynamicObstacles) {
      var local = obstacle.detection.pose.relativeTo(grid.origin);
      rasterizeObstacle(local.x, local.y, obstacle.radiusMeters);
    }
  }

  /** Replaces the dynamic obstacle layer and refreshes costs. */
  public function setDynamicObstacles(obstacles:Array<Obstacle>):Void {
    if (obstacles == null) throw "Costmap2 obstacles cannot be null";
    for (obstacle in obstacles) {
      if (obstacle == null) throw "Costmap2 obstacles cannot contain null";
      if (obstacle.detection.frameId != grid.frameId)
        throw 'Dynamic obstacle ${obstacle.detection.id} is in frame ${obstacle.detection.frameId}; expected ${grid.frameId}';
    }
    dynamicObstacles = obstacles.copy();
    refresh();
  }

  public function clearDynamicObstacles():Void {
    dynamicObstacles = [];
    refresh();
  }

  public function contains(x:Int, y:Int):Bool return grid.contains(x, y);

  public function isTraversable(x:Int, y:Int):Bool {
    if (!grid.contains(x, y)) return false;
    var index = y * grid.width + x;
    return !blockedValues[index];
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
    var hardRadius = obstacleRadiusMeters + footprintRadiusMeters + cellRadius;
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
