import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.MassProperties.MassSource;
import machinekit.pneumatic.PneumaticManifold;
import machinekit.pneumatic.SuctionCup;
import machinekit.pneumatic.VacuumGenerator;
import machinekit.robotics.FrameBar;
import machinekit.robotics.ParallelGripper;
import machinekit.robotics.ToolChangerMaster;
import machinekit.robotics.ToolChangerTool;

class EndEffectorComponentTests {
	public static function run():Void {
		var master = new ToolChangerMaster(2);
		var tool = new ToolChangerTool(2);
		if (master.port("airOut2").name != "airOut2" || tool.port("airIn2").name != "airIn2" ||
			master.port("lock").required != true || tool.port("signalIn").required != true)
			throw "Changer channels or required ports are missing";
		var gripper = new ParallelGripper(50, 30, 60, 12);
		if (gripper.stroke != 12 || gripper.connector("tcp").name != "tcp" ||
			!gripper.port("open").required || !gripper.port("close").required)
			throw "Gripper interface is incomplete";
		var components:Array<MachineComponent> = [master, tool, gripper, new FrameBar(30, 20, 90),
			new PneumaticManifold(2), new VacuumGenerator(), new SuctionCup(25, 18)];
		var ratedCup = new SuctionCup(40, 18, 1000, 5);
		var ratedGenerator = new VacuumGenerator(60);
		if ((cast ratedCup.effectiveAreaMm2:Float) != 1000 ||
			(cast ratedCup.ratedMomentNm:Float) != 5 ||
			(cast ratedGenerator.ratedVacuumKpa:Float) != 60 ||
			ratedCup.designation == new SuctionCup(40, 18).designation ||
			ratedGenerator.designation == new VacuumGenerator().designation)
			throw "Rated suction components must retain distinct design data";
		var rejected = false;
		try new SuctionCup(40, 18, 2000) catch (_:Dynamic) rejected = true;
		if (!rejected) throw "Suction cup accepted an effective area larger than its face";
		for (component in components) {
			var properties = component.massProperties();
			if (properties.mass <= 0 || properties.inertia == null)
				throw 'Missing geometry-derived mass for ${component.designation}';
			switch properties.source {
				case Computed(Preview):
				case _: throw 'Mass is not computed from geometry for ${component.designation}';
			}
		}
	}
}
