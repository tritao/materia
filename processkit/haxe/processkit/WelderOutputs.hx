package processkit;

/** What a welder is told: the three outputs of its arc channels. A real supply implements it over its I/O or Modbus registers. */
interface WelderOutputs {
  function setArc(on:Bool):Void;
  /** Wire feed speed, in metres per minute. */
  function setWireSpeed(metresPerMinute:Float):Void;
  /** Voltage setpoint, in volts. */
  function setVoltage(volts:Float):Void;
}
