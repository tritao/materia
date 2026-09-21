package robotkit.world;

/** Registry of logical robots owned by one world host. */
class RobotRegistry {
  final values:Map<RobotId, RobotInstance>;

  public function new() {
    values = new Map<RobotId, RobotInstance>();
  }

  public function add(robot:RobotInstance):Void {
    if (robot == null) throw "RobotRegistry cannot add a null robot";
    if (values.exists(robot.id())) throw 'RobotRegistry already contains "${robot.id()}"';
    values.set(robot.id(), robot);
  }

  public function get(id:RobotId):Null < RobotInstance > return values.get(id);

  public function remove(id:RobotId):Null < RobotInstance > {
    var robot = values.get(id);
    if (robot != null) values.remove(id);
    return robot;
  }

  public function ids():Array < RobotId > {
    var result:Array<RobotId> = [];
    for (id in values.keys()) result.push(id);
    result.sort(function(left, right) return Reflect.compare(left, right));
    return result;
  }

  public function all():Array < RobotInstance > {
    var result:Array<RobotInstance> = [];
    for (id in ids()) result.push(values.get(id));
    return result;
  }
}
