package eoat;

import cadkit.modeling.Vector;
import machinekit.component.Solids;
import machinekit.pneumatic.schmalz.SchmalzPushInFitting;
import machinekit.pneumatic.schmalz.SchmalzSuctionCup;
import machinekit.pneumatic.schmalz.SchmalzVacuumGenerator;
import machinekit.pneumatic.schmalz.SchmalzVacuumHose;
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
		result.addComponent("hose", new SchmalzVacuumHose("10.07.09.00001", [
			new Vector(20, 0, 40), new Vector(35, 0, 60),
			new Vector(35, 0, 100), new Vector(-30, 0, 100), new Vector(0, 0, 90)]));
		result.mount("plate", "robot");
		result.addMate("bar-mate", "fixed", "plate", "tool", "bar", "base");
		result.addMemberConnector("bar", "ejector-seat", Solids.axial(20, 0, 20));
		result.addMate("ejector-mate", "fixed", "bar", "ejector-seat", "ejector", "mount");
		result.addMate("cup-mate", "fixed", "bar", "end", "cup", "mount");
		result.addMate("fitting-mate", "fixed", "cup", "mount", "fitting", "mount");
		result.addMate("hose-mate", "fixed", "bar", "base", "hose", "mount");
		result.connectPorts("ejector-hose", "ejector", "vacuum", "hose", "input");
		result.connectPorts("hose-fitting", "hose", "output", "fitting", "hose");
		result.connectPorts("fitting-cup", "fitting", "thread", "cup", "vacuum");
		result.exposePort("compressedAir", "ejector", "air");
		result.workingFrame("contact", "cup", "contact", true);
		return result;
	}
}
