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
    var expanded = FloorMap.rasterize(obstacles, [start, goal, new Pose2(-3.127, 2.013)], 0.05, 1, 1, "map");
    for (x in 0...20) for (y in 0...20) {
      var pose = new Pose2(-0.975 + x * 0.1, -0.975 + y * 0.1);
      var old:robotkit.navigation.GridCell2 = cast grid.worldToCell(pose);
      var added:robotkit.navigation.GridCell2 = cast expanded.worldToCell(pose);
      check(grid.cell(old.x, old.y) == expanded.cell(added.x, added.y), "Extra candidate goals do not move rasterized world obstacles");
    }
    var table:FloorBox = {id: "table", x: 2, y: 0, z: 0.2, halfX: 0.15, halfY: 0.45, halfZ: 0.2, yaw: 0};
    var west = new Pose2(0.35, 0), east = new Pose2(3.65, 0);
    var goals = [new Pose2(-1.5, -0.8), west, east];
    var radius = Math.sqrt(2.45 * 2.45 + 0.82 * 0.82) / 2;
    var clipped = new AStarPlanner(new Costmap2(FloorMap.rasterize([table], goals, 0.05, 1, 1, "map"), radius, true, 0.3, 1.5));
    var clippedRoute = false;
    try clipped.plan(west, east) catch (_:Dynamic) clippedRoute = true;
    check(clippedRoute, "A fixed one-metre map margin clips the enlarged carrier's detour");
    var roomy = new AStarPlanner(new Costmap2(FloorMap.rasterize([table], goals, 0.05,
      FloorMap.navigationMargin(radius, 0.3), 1, "map"), radius, true, 0.3, 1.5));
    var detour = roomy.plan(west, east);
    check(detour.length > east.x - west.x, "Footprint-derived bounds preserve the route between selected stations");
    check(FloorMap.navigationMargin(0.3, 0.3) == 1, "Light carrier mapping keeps its existing margin");
    Sys.println('RobotKit floor map tests passed ($assertions assertions)');
  }
}
