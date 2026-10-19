package machinekit.motion;
/** RA-D8: interchangeable bought and built rotary axes, without exposing their internal parts. */
interface PrecisionRotaryAxis {
  function assembly():machinekit.assembly.MachineAssembly;
  function specification():RotaryAxisSpec;
}
