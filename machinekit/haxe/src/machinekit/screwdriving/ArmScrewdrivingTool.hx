package machinekit.screwdriving;

import machinekit.robotics.ArmTool;
import machinekit.robotics.RobotFlange;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorPlate;
import machinekit.assembly.MachineAssembly;

/** Straight spindle with compliant feed, mounted on the arm's flange adapter. */
class ArmScrewdrivingTool implements ArmTool {
	public final spindle:ScrewdriverSpindle;

	public function new(bit:ScrewdriverBit) spindle = new ScrewdriverSpindle(bit);

	public function build(flange:RobotFlange):EndEffector {
		var result = new EndEffector();
		result.addComponent("plate", new EndEffectorPlate(flange)); result.include("driver", spindle);
		result.mount("plate", "robot");
		result.addMate("driver-mount", "fixed", "plate", "tool", "driver/body", "mount");
		result.exposePort("power", "driver/body", "power");
		result.exposePort("control", "driver/body", "control"); result.exposePort("vacuum", "driver/body", "vacuum");
		result.workingFrame("tcp", "driver/carrier", "tcp", true);
		return result;
	}
	public function expose(arm:MachineAssembly):Void {
		arm.exposeConnector("toolTcp", "tool/driver/carrier", "tcp");
		arm.exposePort("driverPower", "tool/driver/body", "power");
		arm.exposePort("driverControl", "tool/driver/body", "control");
		arm.exposePort("driverVacuum", "tool/driver/body", "vacuum");
	}
}
