package machinekit.component;

import materia.project.AssemblyRecord.AssemblyFrame;

/** A named frame in the component's CAD coordinates. Joints act along its +Y axis. */
typedef Connector = {
	var name:String;
	var role:ConnectorRole;
	var frame:AssemblyFrame;
}
