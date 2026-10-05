import processkit.WeldStationPlanner;
import processkit.WeldStationCandidates;
import processkit.WeldStationPlanner.WeldStationCandidate;
import robotkit.mobile.Pose2;
import robotkit.spatial.Vec3;

class WeldStationTests {
  static var checks:Int = 0;
  static function check(ok:Bool, message:String):Void {
    checks++;
    if (!ok) throw message;
  }
  static function rejects(action:() -> Void, expected:String):Void {
    var reason = "";
    try action() catch (error:Dynamic) reason = Std.string(error);
    check(reason.indexOf(expected) >= 0, 'Expected $expected, got $reason');
  }
  static function distance(from:Pose2, to:Pose2):Null<Float>
    return Math.sqrt(Math.pow(from.x - to.x, 2) + Math.pow(from.y - to.y, 2));
  public static function main():Void {
    candidateGeometry();
    var seams = ["a", "b", "c", "d", "e", "f"];
    var candidates = [new WeldStationCandidate("greedy", new Pose2(10, 0)),
      new WeldStationCandidate("left", new Pose2(2, 0)), new WeldStationCandidate("right", new Pose2(1, 0))];
    function access(station:WeldStationCandidate, seam:String):Null<String> {
      var names = station.id == "greedy" ? ["a", "b", "c", "d"] :
        station.id == "left" ? ["a", "b", "e"] : ["c", "d", "f"];
      return names.indexOf(seam) >= 0 ? null : "torch is out of reach";
    }
    var plan = WeldStationPlanner.plan(seams, candidates, new Pose2(), access, distance);
    check(plan.stations.length == 2, "Exact cover beats greedy's three stations");
    check(plan.stations[0].id == "right" && plan.stations[1].id == "left", "Route uses the cheaper direction");
    check(plan.travelCost == 2, "Route includes travel from the initial pose");
    var assigned:Array<String> = [];
    for (group in plan.seams) for (name in group) assigned.push(name);
    check(assigned.length == seams.length, "Every seam assigned once");
    for (name in seams) check(assigned.indexOf(name) >= 0, 'Required seam $name survives planning');
    var overlapping = WeldStationPlanner.plan(["a", "b", "c"], candidates, new Pose2(), access, distance);
    check(overlapping.stations.length == 1 && overlapping.stations[0].id == "greedy",
      "Station count takes priority over travel cost");
    function blocked(from:Pose2, to:Pose2):Null<Float> return to.x == 10 ? null : distance(from, to);
    var navigable = WeldStationPlanner.plan(["a", "b", "c"], candidates, new Pose2(), access, blocked);
    check(navigable.stations.length == 2, "An inaccessible one-station cover does not hide a navigable two-station cover");
    function directed(from:Pose2, to:Pose2):Null<Float>
      return from.x == 1 && to.x == 2 ? null : distance(from, to);
    var oneWay = WeldStationPlanner.plan(seams, candidates, new Pose2(), access, directed);
    check(oneWay.stations[0].id == "left" && oneWay.travelCost == 3,
      "A blocked directed transition forces the feasible reverse order");
    function shared(station:WeldStationCandidate, seam:String):Null<String> {
      if (station.id == "greedy") return "arm clearance violation";
      return (station.id == "left" ? ["a", "b", "c"] : ["b", "c", "d"]).indexOf(seam) >= 0 ? null : "out of reach";
    }
    var overlap = WeldStationPlanner.plan(["a", "b", "c", "d"], candidates, new Pose2(), shared, distance);
    var deposited:Array<String> = [];
    for (group in overlap.seams) for (name in group) deposited.push(name);
    check(deposited.length == 4 && overlap.stations.length == 2, "Overlapping coverage does not duplicate welds");
    var calls = new Map<String, Int>();
    function counted(from:Pose2, to:Pose2):Null<Float> {
      var key = '${from.x}>${to.x}';
      var previous = calls.get(key);
      calls.set(key, previous == null ? 1 : previous + 1);
      return distance(from, to);
    }
    WeldStationPlanner.plan(seams, candidates, new Pose2(), access, counted);
    for (count in calls) check(count == 1, "Each needed navigation edge is computed only once");
    check(!calls.exists("0>10"), "Navigation does not plan routes to candidates outside a minimum cover");
    var fullCalls = new Map<String, Int>();
    function prove(station:WeldStationCandidate, seam:String):Null<String> {
      var key = station.id + ":" + seam;
      var before = fullCalls.get(key);
      fullCalls.set(key, before == null ? 1 : before + 1);
      return station.id == "greedy" ? "swept entry collision" : access(station, seam);
    }
    var proven = WeldStationPlanner.verified(["a", "b", "c"], candidates, new Pose2(), access, prove, distance);
    check(proven.stations.length == 2, "Provisional one-station coverage is rejected after its swept entry fails");
    for (count in fullCalls) check(count == 1, "Full coverage checks are cached across selection retries");
    for (index in 0...proven.stations.length) for (name in proven.seams[index])
      check(fullCalls.exists(proven.stations[index].id + ":" + name), "Every assigned seam has a full-motion proof");
    rejects(() -> {
      WeldStationPlanner.verified(["a"], candidates, new Pose2(), (_, _) -> null,
        (_, _) -> "all entries collide", distance);
    }, "all entries collide");
    rejects(() -> { WeldStationPlanner.plan(["missing"], candidates, new Pose2(), access, distance); }, "missing");
    rejects(() -> { WeldStationPlanner.plan(seams, candidates, new Pose2(), access, (_, _) -> null); }, "no navigation route");
    rejects(() -> { WeldStationPlanner.plan(["a", "a"], candidates, new Pose2(), access, distance); }, "unique");
    rejects(() -> { WeldStationPlanner.plan(seams, [candidates[0], candidates[0]], new Pose2(), access, distance); }, "unique");
    rejects(() -> { WeldStationPlanner.plan(seams, candidates, new Pose2(), access, (_, _) -> -1.0); }, "nonnegative");
    check(WeldStationPlanner.plan([], [], new Pose2(), access, distance).stations.length == 0, "Empty work needs no stations");
    check(seams.length == 6 && candidates.length == 3, "Planning leaves its inputs unchanged");
    Sys.println('ProcessKit welding station tests passed ($checks checks)');
  }

  static function candidateGeometry():Void {
    var endpoints = [new Vec3(-0.2, -0.1, 0.9), new Vec3(0.2, 0.1, 1.1)];
    var mount = new Vec3(0.925, 0.04, 0.328);
    var work = new Pose2(2.0, -0.15, Math.PI / 2);
    var grid = WeldStationCandidates.around(endpoints, work, mount, 0.45, 0.65, 3, 12);
    check(grid.length == 36, "Documented heading/standoff grid produces every candidate");
    for (index in 0...grid.length) {
      var candidate = grid[index];
      var arm = candidate.pose.compose(new Pose2(mount.x, mount.y));
      var dx = work.x - arm.x, dy = work.y - arm.y;
      var standoff = 0.45 + 0.1 * (index % 3);
      check(Math.abs(Math.sqrt(dx * dx + dy * dy) - standoff) < 1e-9,
        "Standoff measures the mounted arm's position, not the long chassis centre");
      check(Math.abs(Pose2.wrapAngle(Math.atan2(dy, dx) - arm.yaw)) < 1e-9, "Mounted arm faces the work bounds");
    }
    var shift = new Pose2(-1.2, 0.8, -0.3);
    var moved = WeldStationCandidates.around(endpoints, shift.compose(work), mount, 0.45, 0.65, 3, 12);
    for (index in 0...grid.length) {
      var expected = shift.compose(grid[index].pose);
      check(Math.abs(expected.x - moved[index].pose.x) < 1e-9 && Math.abs(expected.y - moved[index].pose.y) < 1e-9 &&
        Math.abs(Pose2.wrapAngle(expected.yaw - moved[index].pose.yaw)) < 1e-9,
        "A displaced CAD work frame displaces its stations without authored world coordinates");
    }
    rejects(() -> { WeldStationCandidates.around([], work, mount, 0.4, 0.6); }, "seam endpoints");
    rejects(() -> { WeldStationCandidates.around(endpoints, work, mount, 0.6, 0.4); }, "standoffs");
  }
}
