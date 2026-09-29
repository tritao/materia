package machinekit.assembly;

import materia.assembly.AssemblyDefinition;
import machinekit.assembly.MachineAssemblyDescription.FrozenAssemblyDefinition;
import haxeon.wire.JsonWire;

/** Detach mutable builder definitions at the description boundary. */
class FrozenAssemblyDefinitions {
	public static function freeze(value:AssemblyDefinition):FrozenAssemblyDefinition
		return JsonWire.decode(JsonWire.encode(value));

	public static function thaw(value:FrozenAssemblyDefinition):AssemblyDefinition
		return JsonWire.decode(JsonWire.encode(value));
}
