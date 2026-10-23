package eoat;

import cadkit.modeling.Vector;
import machinekit.component.Solids;
import machinekit.pneumatic.PushInFitting;
import machinekit.pneumatic.SuctionCup;
import machinekit.pneumatic.VacuumGenerator;
import machinekit.pneumatic.RoutedHose;
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
		result.addComponent("ejector", VacuumGenerator.model("10.02.01.00563"));
		result.addComponent("cup", SuctionCup.model("10.01.01.11401"));
		result.addComponent("fitting", new PushInFitting("10.08.02.00203"));
		result.addComponent("hose", RoutedHose.stock("10.07.09.00001", [
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
