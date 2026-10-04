package machinekit.robotics;

import cadkit.modeling.Vector;
import machinekit.component.Solids;
import machinekit.pneumatic.VacuumPressureSensor;
import machinekit.pneumatic.schmalz.SchmalzPushInFitting;
import machinekit.pneumatic.schmalz.SchmalzSuctionCup;
import machinekit.pneumatic.schmalz.SchmalzVacuumGenerator;
import machinekit.pneumatic.schmalz.SchmalzVacuumHose;

/** Catalog suction tool with a physical flange adapter and inline vacuum sensor. */
class SuctionTool extends EndEffector {
	public function new(flange:RobotFlange) {
		super();
		addComponent("plate", new EndEffectorPlate(flange));
		addComponent("bar", new FrameBar(30, 20, 90));
		addComponent("ejector", new SchmalzVacuumGenerator("10.02.01.00563"));
		addComponent("cup", new SchmalzSuctionCup("10.01.01.11401"));
		addComponent("fitting", new SchmalzPushInFitting("10.08.02.00203"));
		addComponent("sensor", new VacuumPressureSensor(4));
		addComponent("hose", new SchmalzVacuumHose("10.07.09.00001", [
			new Vector(20, 0, 40), new Vector(35, 0, 60),
			new Vector(35, 0, 100), new Vector(-30, 0, 100), new Vector(0, 0, 90)]));
		mount("plate", "robot");
		addMate("bar-mate", "fixed", "plate", "tool", "bar", "base");
		addMemberConnector("bar", "ejector-seat", Solids.axial(20, 0, 20));
		addMate("ejector-mate", "fixed", "bar", "ejector-seat", "ejector", "mount");
		addMemberConnector("bar", "sensor-seat", Solids.axial(-25, 0, 20));
		addMate("sensor-mate", "fixed", "bar", "sensor-seat", "sensor", "mount");
		addMate("cup-mate", "fixed", "bar", "end", "cup", "mount");
		addMate("fitting-mate", "fixed", "cup", "mount", "fitting", "mount");
		addMate("hose-mate", "fixed", "bar", "base", "hose", "mount");
		connectPorts("ejector-sensor", "ejector", "vacuum", "sensor", "vacuumIn");
		connectPorts("sensor-hose", "sensor", "vacuumOut", "hose", "input");
		connectPorts("hose-fitting", "hose", "output", "fitting", "hose");
		connectPorts("fitting-cup", "fitting", "thread", "cup", "vacuum");
		exposePort("compressedAir", "ejector", "air");
		workingFrame("contact", "cup", "contact", true);
	}
}
