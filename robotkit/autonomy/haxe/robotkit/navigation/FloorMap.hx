package robotkit.navigation;

import robotkit.mobile.Pose2;

/** A posed obstacle box, with dimensions in metres and yaw in radians. */
typedef FloorBox = {
  var id:String;
  var x:Float;
  var y:Float;
  var z:Float;
  var halfX:Float;
  var halfY:Float;
  var halfZ:Float;
  var yaw:Float;
}

/** Rasterizes posed obstacle boxes for both navigation execution and upstream parking planners. */
class FloorMap {
  public static inline var DEFAULT_RESOLUTION:Float = 0.05;
  public static inline var DEFAULT_MARGIN:Float = 1.0;
  public static inline var DEFAULT_CLEARANCE:Float = 1.0;
  /** Leave space to detour around inflated obstacles even when only the selected goals bound the map. */
  public static function navigationMargin(footprintRadius:Float, inflationDistance:Float,
      resolution:Float = DEFAULT_RESOLUTION):Float {
    if (!Math.isFinite(footprintRadius) || footprintRadius < 0 || !Math.isFinite(inflationDistance) ||
        inflationDistance < 0 || !Math.isFinite(resolution) || resolution <= 0)
      throw "Navigation map margin needs finite nonnegative footprint and inflation and a positive resolution";
    return Math.max(DEFAULT_MARGIN, footprintRadius + inflationDistance + 2 * resolution);
  }
  public static function standing(boxes:Array<FloorBox>, clearance:Float):Array<FloorBox>
    return [for (box in boxes) if (box.z - box.halfZ < clearance && box.z + box.halfZ > 0.01) box];

  public static function rasterize(obstacles:Array<FloorBox>, poses:Array<Pose2>, resolution:Float,
      margin:Float, clearance:Float, frame:String):OccupancyGrid2 {
    var minX = Math.POSITIVE_INFINITY, minY = Math.POSITIVE_INFINITY;
    var maxX = Math.NEGATIVE_INFINITY, maxY = Math.NEGATIVE_INFINITY;
    function extend(x:Float, y:Float):Void {
      minX = Math.min(minX, x); maxX = Math.max(maxX, x);
      minY = Math.min(minY, y); maxY = Math.max(maxY, y);
    }
    var blocking = standing(obstacles, clearance);
    for (box in blocking) {
      var reach = Math.sqrt(box.halfX * box.halfX + box.halfY * box.halfY);
      extend(box.x - reach, box.y - reach);
      extend(box.x + reach, box.y + reach);
    }
    for (pose in poses) extend(pose.x, pose.y);
    if (!Math.isFinite(minX)) throw "A mobile mission needs somewhere to drive";
    // Keep world cell boundaries stable when planners consider different sets of goals.
    var originX = Math.floor((minX - margin) / resolution) * resolution;
    var originY = Math.floor((minY - margin) / resolution) * resolution;
    var width = Math.ceil((maxX + margin - originX) / resolution);
    var height = Math.ceil((maxY + margin - originY) / resolution);
    var grid = new OccupancyGrid2(resolution, new Pose2(originX, originY), width, height, frame,
      OccupancyCell.Free);
    // A cell square overlaps a box when its centre lies within the box grown by half a cell's diagonal
    // projected on each box axis; growing by the full half-diagonal is the safe side of that.
    var grow = resolution * Math.sqrt(0.5);
    for (box in blocking) {
      var c = Math.cos(box.yaw), s = Math.sin(box.yaw);
      var reach = Math.sqrt(box.halfX * box.halfX + box.halfY * box.halfY) + grow;
      var low = grid.worldToCell(new Pose2(box.x - reach, box.y - reach));
      var high = grid.worldToCell(new Pose2(box.x + reach, box.y + reach));
      if (low == null || high == null) throw 'Obstacle "${box.id}" is off the map';
      for (cx in low.x...high.x + 1) for (cy in low.y...high.y + 1) {
        var centre = grid.cellCenter(cx, cy);
        var dx = centre.x - box.x, dy = centre.y - box.y;
        var along = dx * c + dy * s, across = -dx * s + dy * c;
        if (Math.abs(along) <= box.halfX + grow && Math.abs(across) <= box.halfY + grow)
          grid.setCell(cx, cy, OccupancyCell.Occupied);
      }
    }
    return grid;
  }
}
