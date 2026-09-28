package eoat;

import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly.AssemblyBomMass;
import machinekit.component.Solids;
import machinekit.pneumatic.schmalz.SchmalzPushInFitting;
import machinekit.pneumatic.schmalz.SchmalzSuctionCup;
import machinekit.pneumatic.schmalz.SchmalzVacuumGenerator;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorPlate;
import machinekit.robotics.FrameBar;
import machinekit.robotics.RobotFlange;

/** One fixed EOAT using a catalog cup, ejector and matching threaded fitting. */
class SchmalzEndEffectorExample {
	public static function build():EndEffector {
		var result = new EndEffector();
		result.addComponent("plate", new EndEffectorPlate(new RobotFlange(50)));
		result.addComponent("bar", new FrameBar(30, 20, 90));
		result.addComponent("ejector", new SchmalzVacuumGenerator("10.02.01.00563"));
		result.addComponent("cup", new SchmalzSuctionCup("10.01.01.11401"));
		result.addComponent("fitting", new SchmalzPushInFitting("10.08.02.00203"));
		result.mount("plate", "robot");
		result.addMate("bar-mate", "fixed", "plate", "tool", "bar", "base");
		result.addMemberConnector("bar", "ejector-seat", Solids.axial(20, 0, 20));
		result.addMate("ejector-mate", "fixed", "bar", "ejector-seat", "ejector", "mount");
		result.addMate("cup-mate", "fixed", "bar", "end", "cup", "mount");
		result.addMate("fitting-mate", "fixed", "cup", "mount", "fitting", "mount");
		result.connectPorts("vacuum-hose", "ejector", "vacuum", "fitting", "hose",
			{partNumber: "10.07.09.00001", description: "Schmalz VSL 4-2 PU vacuum hose, 0.2 m cut",
				quantity: 1, material: "polyurethane"},
			Attached(0.0022, "bar", new Vector(10, 0, 60)));
		result.connectPorts("fitting-cup", "fitting", "thread", "cup", "vacuum");
		result.exposePort("compressedAir", "ejector", "air");
		result.workingFrame("contact", "cup", "contact", true);
		return result;
	}
}
