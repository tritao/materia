package motionkit.robot;

/** Machine-authored dependency graph. The shared endpoint executes one ready coordinate at a time. */
class HomingSequence {
  public static function order(axes:Array<HomingAxis>):Array<HomingAxis> {
    var coordinates = new Map<String, HomingAxis>();
    for (axis in axes) {
      if (axis == null) throw "Homing cannot contain a null axis";
      var joint = axis.switches[0].joint;
      if (coordinates.exists(joint)) throw "A homing coordinate belongs to multiple axes";
      coordinates.set(joint, axis);
      var expected = axis.switches[0].homeAfter.copy(); expected.sort(Reflect.compare);
      for (contact in axis.switches) {
        var dependencies = contact.homeAfter.copy(); dependencies.sort(Reflect.compare);
        if (dependencies.join("\n") != expected.join("\n"))
          throw "Home switches on one coordinate must agree on dependencies";
      }
    }
    var visiting = new Map<String, Bool>(), visited = new Map<String, Bool>();
    var result:Array<HomingAxis> = [];
    function visit(id:String):Void {
      if (!coordinates.exists(id)) throw 'Home dependency "$id" has no homing axis';
      if (visiting.exists(id)) throw "Homing dependencies contain a cycle";
      if (visited.exists(id)) return;
      visiting.set(id, true);
      var axis = coordinates.get(id), dependencies = axis.switches[0].homeAfter.copy();
      dependencies.sort(Reflect.compare);
      for (dependency in dependencies) visit(dependency);
      visiting.remove(id); visited.set(id, true); result.push(axis);
    }
    var names = [for (id in coordinates.keys()) id]; names.sort(Reflect.compare);
    for (id in names) visit(id);
    return result;
  }
}
