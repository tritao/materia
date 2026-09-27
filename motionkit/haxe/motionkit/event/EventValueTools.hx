package motionkit.event;

/** Shared validation for event values at immutable container boundaries. */
class EventValueTools {
  public static function validate(value:EventValue, label:String):Void {
    if (value == null) throw '$label is required';
    switch value {
      case Digital(_):
      case Analog(number):
        if (!Math.isFinite(number)) throw '$label analog value must be finite';
      case Process(command, argument):
        if (command == null || StringTools.trim(command).length == 0)
          throw '$label process command must be non-empty';
        if (!Math.isFinite(argument)) throw '$label process argument must be finite';
    }
  }

  public static function matchesKind(value:EventValue, kind:ChannelKind):Bool {
    if (value == null || kind == null) return false;
    return switch value {
      case Digital(_): switch kind { case Digital: true; case _: false; };
      case Analog(_): switch kind { case Analog: true; case _: false; };
      case Process(_, _): switch kind { case Process: true; case _: false; };
    };
  }
}
