package robotkit.execution;

import RobotKitRuntime;




/** ASCII ABI encoding shared by plan submission, blueprint and event polling. */
class ProcessEventCodec {
  public static function validateId(value:String):Void {
    if (value == null || value.length == 0 || value.length >=
        RobotKitRuntimeConstants.RK_PROCESS_CHANNEL_ID_BYTES)
      throw "Process channel ID must fit the runtime ABI";
    for (i in 0...value.length) {
      var code = value.charCodeAt(i);
      if (code <= ' '.code || code > '~'.code) throw "Process channel ID must be printable ASCII";
    }
  }

  public static function validateValue(value:ProcessEventValue):Void {
    if (value == null) throw "Process event needs a value";
    switch value {
      case Digital(_):
      case Analog(number):
        if (!Math.isFinite(number)) throw "Process analog value must be finite";
      case Process(command, argument):
        if (command == null || command.length == 0 || command.length >=
            RobotKitRuntimeConstants.RK_PROCESS_COMMAND_BYTES || !Math.isFinite(argument))
          throw "Invalid process command";
        for (i in 0...command.length) {
          var code = command.charCodeAt(i);
          if (code <= ' '.code || code > '~'.code) throw "Process command must be printable ASCII";
        }
    }
  }

  public static function encode(value:ProcessEventValue):rk_event_value {
    validateValue(value);
    var result = new rk_event_value();
    switch value {
      case Digital(enabled):
        result.set_kind(RobotKitRuntimeConstants.RK_EVENT_DIGITAL);
        result.set_digital(enabled ? 1 : 0);
      case Analog(number):
        result.set_kind(RobotKitRuntimeConstants.RK_EVENT_ANALOG);
        result.set_analog(number);
      case Process(command, argument):
        result.set_kind(RobotKitRuntimeConstants.RK_EVENT_PROCESS);
        for (i in 0...command.length) result.set_command(i, command.charCodeAt(i));
        result.set_argument(argument);
    }
    return result;
  }

  public static function decode(value:rk_event_value):ProcessEventValue {
    return switch value.get_kind() {
      case RobotKitRuntimeConstants.RK_EVENT_DIGITAL:
        ProcessEventValue.Digital(value.get_digital() != 0);
      case RobotKitRuntimeConstants.RK_EVENT_ANALOG:
        ProcessEventValue.Analog(value.get_analog());
      case RobotKitRuntimeConstants.RK_EVENT_PROCESS:
        ProcessEventValue.Process(readCommand(value), value.get_argument());
      default: throw "Unknown runtime process value";
    };
  }

  public static function readChannel(value:rk_event_record):String {
    var result = new StringBuf();
    for (i in 0...RobotKitRuntimeConstants.RK_PROCESS_CHANNEL_ID_BYTES) {
      var code = value.get_channel(i);
      if (code == 0) break;
      result.addChar(code);
    }
    return result.toString();
  }

  static function readCommand(value:rk_event_value):String {
    var result = new StringBuf();
    for (i in 0...RobotKitRuntimeConstants.RK_PROCESS_COMMAND_BYTES) {
      var code = value.get_command(i);
      if (code == 0) break;
      result.addChar(code);
    }
    return result.toString();
  }
}
