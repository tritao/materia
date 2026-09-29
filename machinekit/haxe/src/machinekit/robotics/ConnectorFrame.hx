package machinekit.robotics;

import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Millimetre frame whose connector approach axis is +Y. */
abstract ConnectorFrame(AssemblyFrame) {
	public inline function new(frame:AssemblyFrame) this = frame;
	public inline function raw():AssemblyFrame return (this : AssemblyFrame);
}
