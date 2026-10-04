package machinekit.motion;

/** Mechanical switch characteristics, in assembly millimetres. */
interface SwitchPart {
  function tripConnector():String;
  function switchHysteresis():Float;
  function switchRepeatability():Float;
}
