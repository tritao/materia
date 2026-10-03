package machinekit.robotics;

import machinekit.assembly.MachineAssembly;

/** What the arm carries on its tool flange: the end effector built for that flange, and the
 * connectors and service inlets of it that the arm publishes as its own. The arm includes the
 * effector as `tool`, so `expose` names its members `tool/<member>`.
 */
interface ArmTool {
	function build(flange:RobotFlange):EndEffector;
	/** Publishes the tool's connectors and the service inlets that must be supplied from outside. */
	function expose(arm:MachineAssembly):Void;
}

