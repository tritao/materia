package machinekit.motion;

import materia.assembly.AssemblyDefinition.AssemblyActuator;

/**
 * A motor part that can drive an assembly joint. The part states its drive kind and what it can
 * deliver; `MachineAssembly.addMotor` puts the result in the assembly, and rebuilding the assembly
 * asks the part again, so editing a motor's ratings changes its actuator. A servo motor part
 * implements this the same way a stepper does.
 */
interface MotorDrive {
  /**
   * The actuator `id` on joint `joint`: its usable effort and rate, its rotor inertia, and its
   * drive kind with torque-speed curve. `volts` is the supply (it moves a stepper's pull-out
   * curve), `margin` the share of a stepper's torque to rely on; a servo ignores both.
   */
  function actuator(id:String, joint:String, volts:Float, margin:Float):AssemblyActuator;
}
