package machinekit.picking;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import machinekit.component.MachineComponent;

/** One named component occurrence in a virtual picking station. */
typedef StationInstance = {
	var id:String;
	var label:String;
	var component:MachineComponent;
	var pose:AssemblyFrame;
}
