package machinekit.pressbrake.planning;

import machinekit.assembly.MachineAssembly;
import machinekit.pressbrake.PressBrakeProfile;

/** An assembly declares where its physical press datums live; planning reads their evaluated poses. */
interface PressGeometrySource {
	public function pressAssembly():MachineAssembly;
	public function pressGeometryDatums():PressGeometryDatums;
	public function pressProfile():PressBrakeProfile;
	public function pressCapacityN():Float;
}
