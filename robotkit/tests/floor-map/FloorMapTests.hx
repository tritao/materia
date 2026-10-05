import robotkit.mobile.Pose2;
import robotkit.navigation.FloorMap;
import robotkit.navigation.FloorMap.FloorBox;
import robotkit.navigation.Costmap2;
import robotkit.navigation.AStarPlanner;
import robotkit.navigation.OccupancyCell;

class FloorMapTests {
  static var assertions:Int = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
  static function box(id:String, z:Float):FloorBox
    return {id: id, x: 0, y: 0, z: z, halfX: 0.1, halfY: 0.8, halfZ: 0.1, yaw: Math.PI / 4};
  public static function main():Void {
    var start = new Pose2(-1.5, 0), goal = new Pose2(1.5, 0);
    var obstacles = [box("barrier", 0.2), box("floor", -0.1), box("overhead", 2.0)];
    var standing = FloorMap.standing(obstacles, 1.0);
    check(standing.length == 1 && standing[0].id == "barrier", "Floor and overhead geometry do not block a floor route");
    var grid = FloorMap.rasterize(obstacles, [start, goal], 0.05, 1, 1, "map");
    var center:robotkit.navigation.GridCell2 = cast grid.worldToCell(new Pose2());
    check(grid.cell(center.x, center.y) == OccupancyCell.Occupied, "The posed barrier occupies its centre");
    var costs = new Costmap2(grid, 0.2);
    var planner = new AStarPlanner(costs);
    var route = planner.plan(start, goal);
    check(route.length > 3.0, "Navigation finds a detour around a rotated CAD obstacle");
    for (pose in route.poses()) {
      var cell:robotkit.navigation.GridCell2 = cast grid.worldToCell(pose);
      check(costs.isTraversable(cell.x, cell.y), "Every planned waypoint clears the inflated obstacle");
    }
    var rejected = false;
    try planner.plan(start, new Pose2()) catch (_:Dynamic) rejected = true;
    check(rejected, "An occupied parking pose is rejected");
    check(obstacles.length == 3, "Mapping does not change its source obstacle list");
    Sys.println('RobotKit floor map tests passed ($assertions assertions)');
  }
}
