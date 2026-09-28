package eoat;

import machinekit.component.Solids;
import machinekit.pneumatic.PneumaticManifold;
import machinekit.pneumatic.SuctionCup;
import machinekit.pneumatic.VacuumGenerator;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorPlate;
import machinekit.robotics.EndEffectorSet;
import machinekit.robotics.FrameBar;
import machinekit.robotics.RobotFlange;
import machinekit.robotics.ToolChangerMaster;
import machinekit.robotics.ToolChangerTool;

/** Two interchangeable generic EOATs sharing one robot-side changer. */
class EndEffectorExample {
	public static function build():EndEffectorSet {
		var set = new EndEffectorSet();
		set.addComponent("adapter", new EndEffectorPlate(new RobotFlange(50)));
		set.addComponent("master", new ToolChangerMaster(1));
		set.mount("adapter", "robot");
		set.addMate("master-mate", "fixed", "adapter", "tool", "master", "robot");
		set.exposePort("robotAir", "master", "airIn1");
		set.exposePort("robotSignal", "master", "signalIn");
		set.exposePort("lock", "master", "lock");
		set.exposePort("coupledAir", "master", "airOut1");
		set.exposePort("coupledSignal", "master", "signalOut");
		set.changer("coupling", "master", "tool", [
			{robot: "coupledAir", tool: "air"},
			{robot: "coupledSignal", tool: "signal"}
		]);
		set.addTool("short", tool(90, 25));
		set.addTool("long", tool(140, 35));
		return set;
	}

	static function tool(reach:Float, cupDiameter:Float):EndEffector {
		var result = new EndEffector();
		result.addComponent("changer", new ToolChangerTool(1));
		result.addComponent("bar", new FrameBar(30, 20, reach));
		result.addComponent("manifold", new PneumaticManifold(1));
		result.addComponent("generator", new VacuumGenerator());
		result.addComponent("cup", new SuctionCup(cupDiameter, 18));
		result.mount("changer", "master");
		result.addMate("bar-mate", "fixed", "changer", "payload", "bar", "base");
		result.addMemberConnector("bar", "manifold-seat", Solids.axial(-20, 0, 25));
		result.addMemberConnector("bar", "generator-seat", Solids.axial(20, 0, 40));
		result.addMate("manifold-mate", "fixed", "bar", "manifold-seat", "manifold", "mount");
		result.addMate("generator-mate", "fixed", "bar", "generator-seat", "generator", "mount");
		result.addMate("cup-mate", "fixed", "bar", "end", "cup", "mount");
		result.connectPorts("air-manifold", "changer", "airOut1", "manifold", "input");
		result.connectPorts("air-generator", "manifold", "out1", "generator", "air");
		result.connectPorts("vacuum-cup", "generator", "vacuum", "cup", "vacuum");
		result.exposePort("air", "changer", "airIn1");
		result.exposePort("signal", "changer", "signalIn");
		result.workingFrame("contact", "cup", "contact", true);
		return result;
	}
}
