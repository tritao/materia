package motionkit.robot;

import motionkit.event.EventValue;
import robotkit.model.RobotModel;
import robotkit.execution.GearTerm;

/** Planning mirror of the runtime event protocol. Process speed channels provide master velocity.
    A process spindle's acceleration envelope must be declared explicitly for torque checks.
*/
class GearCheckState {
  public var active:Map<Int, Array<GearTerm>> = new Map();
  final staged:Map<Int, Array<GearTerm>> = new Map();
  final values:Map<String, Float> = new Map();
  final model:RobotModel;
  public function new(model:RobotModel) this.model = model;
  public function observe(channel:String, value:EventValue):Void {
    if (!StringTools.startsWith(channel, "motion.gear/")) {
      switch value { case Analog(v): values.set(channel, v); case _: }
      return;
    }
    var follower = Std.parseInt(channel.substr(12));
    if (follower == null || follower < 0 || follower >= model.joints.length) throw 'Invalid gear channel $channel';
    switch value {
      case Process(command, argument):
        switch command {
          case "gear.release": active.remove(follower); staged.remove(follower);
          case "gear.begin": staged.set(follower, []);
          case "gear.engage":
            var terms = staged.get(follower);
            if (terms == null || terms.length == 0) throw "Gear engagement has no staged masters";
            active.set(follower, terms.copy()); staged.remove(follower);
            var visiting:Array<Int> = [], visited:Array<Int> = [];
            for (joint in active.keys()) visit(joint, visiting, visited);
          case "gear.loop":
            if (!staged.exists(follower) || !Math.isFinite(argument) || argument <= 0 || argument > 1000000) throw "Invalid gear loop rate";
          case "gear.quantum":
            var terms = staged.get(follower);
            if (terms == null || terms.length == 0) throw "Gear quantum has no term";
            var t = terms.pop();
            if (t == null) throw "Gear quantum has no term";
            terms.push(new GearTerm(t.master, t.ratio, t.latencyNs, argument, t.extrapolate));
          case _:
            var fields = command.split("/");
            var terms = staged.get(follower);
            if (fields.length != 4 || fields[0] != "gear.term" || terms == null || terms.length >= 8) throw "Invalid gear term event";
            var master = Std.parseInt(fields[1]), latency = Std.parseInt(fields[2]), predict = Std.parseInt(fields[3]);
            if (master == null || latency == null || predict == null || master >= model.joints.length || master == follower || (predict != 0 && predict != 1)) throw "Invalid gear term inputs";
            for (term in terms) if (term.master == master) throw "Duplicate gear master";
            terms.push(new GearTerm(master, argument, latency, 0, predict == 1));
        }
      case _: throw "Electronic gearbox requires process events";
    }
  }
  function visit(joint:Int, visiting:Array<Int>, visited:Array<Int>):Void {
    if (visiting.indexOf(joint) >= 0) throw 'Electronic gearbox cycle at ${model.joints[joint].id}';
    if (visited.indexOf(joint) >= 0) return;
    visiting.push(joint);
    var terms = active.get(joint);
    if (terms != null) for (term in terms) visit(term.master, visiting, visited);
    visiting.pop(); visited.push(joint);
  }
  /** Null for an ordinary trajectory master; process speed is in joint SI units. */
  public function processVelocity(joint:Int):Null<Float> {
    for (actuator in model.actuators) {
      var process = actuator.processVelocity;
      var installed = switch actuator.transmission { case SimpleTransmission(id, _, _): id; };
      if (process == null || installed != model.joints[joint].id) continue;
      var speed = values.get(process.speedChannel), direction = values.get(process.directionChannel);
      if (speed == null || direction == null) throw 'Electronic gear master ${model.joints[joint].id} needs programmed process speed and direction';
      return speed * direction * process.radiansPerSpeedUnit;
    }
    return null;
  }
}
