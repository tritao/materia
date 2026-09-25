package robotkit.navigation;

import robotkit.mobile.Pose2;

/**
 * Mutable planar occupancy grid. `origin` is the pose of the lower-left grid
 * corner in `frameId`; cell coordinates address areas of `resolution` metres.
 */
class OccupancyGrid2 {
  public final resolutionMeters:Float;
  public final origin:Pose2;
  public final width:Int;
  public final height:Int;
  public final frameId:String;

  final values:Array<OccupancyCell>;

  public function new(resolutionMeters:Float, origin:Pose2, width:Int,
      height:Int, ?frameId:String = "map",
      ?initialValue:OccupancyCell = OccupancyCell.Unknown) {
    if (!Math.isFinite(resolutionMeters) || resolutionMeters <= 0.0 ||
        origin == null || width <= 0 || height <= 0 ||
        width > Std.int(0x3fffffff / height) ||
        frameId == null || frameId.length == 0)
      throw "OccupancyGrid2 requires a finite resolution, origin, positive dimensions, and frame";
    if (initialValue != OccupancyCell.Unknown && initialValue != OccupancyCell.Free &&
        initialValue != OccupancyCell.Occupied)
      throw "OccupancyGrid2 initial value is invalid";
    this.resolutionMeters = resolutionMeters;
    this.origin = new Pose2(origin.x, origin.y, origin.yaw);
    this.width = width;
    this.height = height;
    this.frameId = frameId;
    values = [];
    for (_ in 0...(width * height)) values.push(initialValue);
  }

  public function contains(x:Int, y:Int):Bool
    return x >= 0 && x < width && y >= 0 && y < height;

  public function cell(x:Int, y:Int):OccupancyCell {
    requireCell(x, y);
    return values[y * width + x];
  }

  public function setCell(x:Int, y:Int, value:OccupancyCell):Void {
    requireCell(x, y);
    if (value != OccupancyCell.Unknown && value != OccupancyCell.Free &&
        value != OccupancyCell.Occupied)
      throw "OccupancyGrid2 cell value is invalid";
    values[y * width + x] = value;
  }

  /** Converts a frame pose to a cell, returning null when outside the grid. */
  public function worldToCell(pose:Pose2):Null<GridCell2> {
    if (pose == null) throw "OccupancyGrid2 pose cannot be null";
    var local = pose.relativeTo(origin);
    var x = Std.int(Math.floor(local.x / resolutionMeters));
    var y = Std.int(Math.floor(local.y / resolutionMeters));
    return contains(x, y) ? new GridCell2(x, y) : null;
  }

  /** Returns the center pose of a grid cell in `frameId`. */
  public function cellCenter(x:Int, y:Int):Pose2 {
    requireCell(x, y);
    return origin.compose(new Pose2((x + 0.5) * resolutionMeters,
      (y + 0.5) * resolutionMeters));
  }

  function requireCell(x:Int, y:Int):Void {
    if (!contains(x, y)) throw "OccupancyGrid2 cell is out of range";
  }
}
