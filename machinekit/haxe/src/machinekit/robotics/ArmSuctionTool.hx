package machinekit.robotics;

import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.component.Solids;
import machinekit.pneumatic.VacuumPressureSensor;
import machinekit.pneumatic.schmalz.SchmalzPushInFitting;
import machinekit.pneumatic.schmalz.SchmalzSuctionCup;
import machinekit.pneumatic.schmalz.SchmalzVacuumGenerator;
import machinekit.pneumatic.schmalz.SchmalzVacuumHose;

/** Suction tool for the arm's ISO 9409-1 style tool flange: an adapter plate, a frame bar, and a
 * catalog ejector, cup, fitting and hose, arranged like the fixed EOAT in `examples/eoat`, with an
 * inline vacuum sensor between the ejector and the hose that tells a sealed cup from an open one.
 * Its `contact` working frame is the cup's contact face. The arm publishes `toolContact` and the
 * ejector's `compressedAir` inlet.
 */
class ArmSuctionTool implements ArmTool {
	public function new() {}

	public function expose(arm:MachineAssembly):Void {
		arm.exposeConnector("toolContact", "tool/cup", "contact");
		// The ejector's compressed-air inlet is the arm's own service input.
		arm.exposePort("compressedAir", "tool/ejector", "air");
	}

	public function build(flange:RobotFlange):EndEffector {
		var result = new EndEffector();
		result.addComponent("plate", new EndEffectorPlate(flange));
		result.addComponent("bar", new FrameBar(30, 20, 90));
		result.addComponent("ejector", new SchmalzVacuumGenerator("10.02.01.00563"));
		result.addComponent("cup", new SchmalzSuctionCup("10.01.01.11401"));
		result.addComponent("fitting", new SchmalzPushInFitting("10.08.02.00203"));
		result.addComponent("sensor", new VacuumPressureSensor(4));
		result.addComponent("hose", new SchmalzVacuumHose("10.07.09.00001", [
			new Vector(20, 0, 40), new Vector(35, 0, 60),
			new Vector(35, 0, 100), new Vector(-30, 0, 100), new Vector(0, 0, 90)]));
		result.mount("plate", "robot");
		result.addMate("bar-mate", "fixed", "plate", "tool", "bar", "base");
		result.addMemberConnector("bar", "ejector-seat", Solids.axial(20, 0, 20));
		result.addMate("ejector-mate", "fixed", "bar", "ejector-seat", "ejector", "mount");
		result.addMemberConnector("bar", "sensor-seat", Solids.axial(-25, 0, 20));
		result.addMate("sensor-mate", "fixed", "bar", "sensor-seat", "sensor", "mount");
		result.addMate("cup-mate", "fixed", "bar", "end", "cup", "mount");
		result.addMate("fitting-mate", "fixed", "cup", "mount", "fitting", "mount");
		result.addMate("hose-mate", "fixed", "bar", "base", "hose", "mount");
		result.connectPorts("ejector-sensor", "ejector", "vacuum", "sensor", "vacuumIn");
		result.connectPorts("sensor-hose", "sensor", "vacuumOut", "hose", "input");
		result.connectPorts("hose-fitting", "hose", "output", "fitting", "hose");
		result.connectPorts("fitting-cup", "fitting", "thread", "cup", "vacuum");
		result.exposePort("compressedAir", "ejector", "air");
		result.workingFrame("contact", "cup", "contact", true);
		return result;
	}
}

