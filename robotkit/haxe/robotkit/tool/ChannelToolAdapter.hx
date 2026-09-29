package robotkit.tool;

import robotkit.world.FiredProcessEvent;
import robotkit.world.ProcessEventValue;

/** Applies runtime output records to simulation tools at scheduled path time. */
class ChannelToolAdapter {
  final handlers:Map<String, FiredProcessEvent -> Void> = new Map();

  public function new() {}

  /** A true output closes the gripper; false opens it. */
  public function bindGripper(channel:String, gripper:Gripper):Void {
    if (gripper == null) throw "Gripper channel binding requires a gripper";
    bind(channel, function(event) {
      if (digital(event.value, "Gripper")) gripper.close(event.scheduledTimeNs);
      else gripper.open(event.scheduledTimeNs);
    });
  }

  /** A true output enables vacuum; false releases it. */
  public function bindVacuum(channel:String, vacuum:Vacuum):Void {
    if (vacuum == null) throw "Vacuum channel binding requires a vacuum";
    bind(channel, function(event) {
      if (digital(event.value, "Vacuum")) vacuum.enable(event.scheduledTimeNs);
      else vacuum.disable(event.scheduledTimeNs);
    });
  }

  /** A true output holds the changer locked; false requests release. */
  public function bindChangerLock(channel:String, changerLock:ChangerLock):Void {
    if (changerLock == null) throw "Changer lock channel binding requires a lock";
    bind(channel, function(event) {
      if (digital(event.value, "Changer lock")) changerLock.lock(event.scheduledTimeNs);
      else changerLock.unlock(event.scheduledTimeNs);
    });
  }

  public function bindSprayerFlow(channel:String, sprayer:Sprayer, onFlow:Float):Void {
    if (sprayer == null || !Math.isFinite(onFlow) || onFlow < 0.0)
      throw "Invalid sprayer channel binding";
    bind(channel, function(event) sprayer.setFlow(number(event.value, onFlow),
      event.scheduledTimeNs));
  }

  public function bindSprayerPressure(channel:String, sprayer:Sprayer, onPressure:Float):Void {
    if (sprayer == null || !Math.isFinite(onPressure) || onPressure < 0.0)
      throw "Invalid sprayer pressure binding";
    bind(channel, function(event) sprayer.setPressure(number(event.value, onPressure),
      event.scheduledTimeNs));
  }

  public function bindSurfaceTool(channel:String, tool:SurfaceTool):Void {
    if (tool == null) throw "Surface tool binding requires a tool";
    bind(channel, function(event) {
      var enabled = switch event.value {
        case Digital(value): value;
        case Analog(value): value > 0.0;
        case Process(_, _): throw "Surface tool needs a digital or analog event";
      };
      if (enabled) tool.enable(event.scheduledTimeNs);
      else tool.disable(event.scheduledTimeNs);
    });
  }

  public function bindSanderSpeed(channel:String, sander:Sander, onRpm:Float):Void {
    if (sander == null || !Math.isFinite(onRpm) || onRpm < 0.0)
      throw "Invalid sander channel binding";
    bind(channel, function(event) sander.setSpeed(number(event.value, onRpm),
      event.scheduledTimeNs));
  }

  public function bind(channel:String, handler:FiredProcessEvent -> Void):Void {
    if (channel == null || channel.length == 0 || handler == null)
      throw "Invalid process channel binding";
    if (handlers.exists(channel)) throw "Duplicate process channel binding";
    handlers.set(channel, handler);
  }

  public function apply(record:FiredProcessEvent):Void {
    if (record == null) throw "Process event record is required";
    var handler = handlers.get(record.channel);
    if (handler == null) throw 'No tool bound to channel ${record.channel}';
    handler(record);
  }

  public function applyAll(records:Array<FiredProcessEvent>):Void {
    if (records == null) throw "Process records are required";
    for (record in records) apply(record);
  }

  static function number(value:ProcessEventValue, onValue:Float):Float {
    return switch value {
      case Digital(enabled): enabled ? onValue : 0.0;
      case Analog(number): number;
      case Process(_, _): throw "Numeric tool channel needs a digital or analog event";
    };
  }

  static function digital(value:ProcessEventValue, capability:String):Bool {
    return switch value {
      case Digital(enabled): enabled;
      case Analog(_), Process(_, _): throw '$capability channel needs a digital event';
    };
  }
}
