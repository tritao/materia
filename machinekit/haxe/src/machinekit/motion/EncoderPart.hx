package machinekit.motion;

import materia.assembly.AssemblyDefinition.AssemblyEncoder;

/**
 * A part that reads a joint's position in counts. `MachineAssembly.addEncoder` puts the result in the
 * assembly, and rebuilding the assembly asks the part again, so editing an encoder's resolution changes
 * what the assembly records. Where the encoder sits (on a motor's back shaft, along a rail) comes from
 * the joint the assembly puts it on.
 */
interface EncoderPart {
  /** The encoder `id` on joint `joint`: its kind, counts and index. */
  function encoder(id:String, joint:String):AssemblyEncoder;
}
