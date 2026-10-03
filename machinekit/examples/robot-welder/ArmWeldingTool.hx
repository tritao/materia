import machinekit.robotics.ArmTool;
import machinekit.assembly.MachineAssembly;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorPlate;
import machinekit.robotics.RobotFlange;
import machinekit.welding.WeldingTorch;

/** Welding tool for the arm's ISO 9409-1 style tool flange: an adapter plate and a MIG torch
 * with a breakaway mount. Its `tcp` working frame is the wire tip at nominal stickout, +Z along
 * the wire. The torch takes weld current, gas, wire and its trigger from a feeder, so the arm
 * publishes the `toolTcp` connector and the torch's four inlets as `torchPower`, `torchGas`,
 * `torchWire` and `torchControl`.
 */
class ArmWeldingTool implements ArmTool {
	/** The torch's inlets, as the tool and the arm name them. */
	static final INLETS:Array<{tool:String, arm:String}> = [
		{tool: "power", arm: "torchPower"}, {tool: "gas", arm: "torchGas"},
		{tool: "wire", arm: "torchWire"}, {tool: "control", arm: "torchControl"}];

	public final bendDegrees:Float;

	public function new(bendDegrees:Float = 45) {
		this.bendDegrees = bendDegrees;
	}

	public function build(flange:RobotFlange):EndEffector {
		var result = new EndEffector();
		result.addComponent("plate", new EndEffectorPlate(flange));
		result.addComponent("torch", new WeldingTorch(bendDegrees));
		result.mount("plate", "robot");
		result.addMate("torch-mate", "fixed", "plate", "tool", "torch", "robot");
		for (inlet in INLETS) result.exposePort(inlet.tool, "torch", inlet.tool);
		result.workingFrame("tcp", "torch", "tcp", true);
		return result;
	}

	public function expose(arm:MachineAssembly):Void {
		arm.exposeConnector("toolTcp", "tool/torch", "tcp");
		for (inlet in INLETS) arm.exposePort(inlet.arm, "tool/torch", inlet.tool);
	}
}
