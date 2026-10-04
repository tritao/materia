import machinekit.component.MachineComponent;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import machinekit.component.MassProperties.MassSource;
import machinekit.pneumatic.PneumaticManifold;
import machinekit.pneumatic.SuctionCup;
import machinekit.pneumatic.VacuumGenerator;
import machinekit.pneumatic.schmalz.SchmalzPushInFitting;
import machinekit.pneumatic.schmalz.SchmalzSuctionCup;
import machinekit.pneumatic.schmalz.SchmalzVacuumGenerator;
import machinekit.pneumatic.schmalz.SchmalzVacuumHose;
import machinekit.robotics.schmalz.SchmalzSxtMaster;
import machinekit.robotics.schmalz.SchmalzSxtTool;
import cadkit.modeling.Vector;
import eoat.SchmalzEndEffectorExample;
import machinekit.component.PortInterfaces;
import machinekit.component.PortInterface;
import machinekit.robotics.FrameBar;
import machinekit.robotics.ParallelGripper;
import machinekit.robotics.ToolChangerMaster;
import machinekit.robotics.ToolChangerTool;

class EndEffectorComponentTests {
	public static function run():Void {
		var vendorCup = new SchmalzSuctionCup("10.01.01.11401");
		var vendorEjector = new SchmalzVacuumGenerator("10.02.01.00563");
		var vendorFitting = new SchmalzPushInFitting("10.08.02.00203");
		var cup30 = new SchmalzSuctionCup("10.01.01.11400");
		var cup50 = new SchmalzSuctionCup("10.01.01.11441");
		if (cup30.massProperties().mass >= vendorCup.massProperties().mass ||
			cup50.massProperties().mass <= vendorCup.massProperties().mass ||
			(cast cup30.effectiveAreaMm2:Float) >= (cast vendorCup.effectiveAreaMm2:Float) ||
			(cast cup50.effectiveAreaMm2:Float) <= (cast vendorCup.effectiveAreaMm2:Float))
			throw "Catalog cup sizes must have distinct mass and rated area";
		var sxtMaster = new SchmalzSxtMaster("10.07.13.00013");
		var sxtTool = new SchmalzSxtTool("10.07.13.00018");
		var masterHalf = sxtMaster.coupling(), toolHalf = sxtTool.coupling();
		if (masterHalf == null || toolHalf == null || masterHalf.key != toolHalf.key ||
			!PortInterfaces.compatible(sxtMaster.port("airOut2").iface,
				sxtTool.port("airIn2").iface) ||
			PortInterfaces.compatible(sxtMaster.port("airOut2").iface,
				sxtTool.port("airIn3").iface) ||
			Math.abs(sxtMaster.massProperties().mass - 1.7) > 1e-9 ||
			Math.abs(sxtTool.massProperties().mass - 0.55) > 1e-9)
			throw "Schmalz SXT pair has incorrect fit, channels or mass";
		var hose = new SchmalzVacuumHose("10.07.09.00001", [new Vector(0, 0, 0),
			new Vector(0, 0, 100), new Vector(100, 0, 100)]);
		if (Math.abs(hose.lengthMm - 200) > 1e-9 ||
			Math.abs(hose.massProperties().mass - 0.0022) > 1e-9 ||
			hose.bom.partNumber.indexOf("10.07.09.00001-L200") != 0 ||
			!PortInterfaces.compatible(hose.port("input").iface,
				vendorFitting.port("hose").iface))
			throw "Cut hose must retain routed length, mass and tube fit";
		var mismatch = new MachineAssembly();
		mismatch.addComponent("fitting", vendorFitting);
		mismatch.addComponent("hose", hose);
		mismatch.exposePort("inlet", "fitting", "hose");
		mismatch.connectPorts("bad-fit", "fitting", "thread", "hose", "input");
		mismatch.validateStructure();
		var rejectedFit = false;
		try mismatch.validate() catch (error:Dynamic)
			rejectedFit = Std.string(error).indexOf("mismatched interfaces") >= 0;
		if (!rejectedFit) throw "Mismatched physical service interfaces must fail validation";
		if (!PortInterfaces.compatible(vendorCup.port("vacuum").iface,
				vendorFitting.port("thread").iface) ||
			PortInterfaces.compatible(Thread("G1/4-F"), Thread("G1/4-F")) ||
			PortInterfaces.compatible(Thread("G1/4-F"), Thread("G1/8-M")))
			throw "Threaded vacuum connections need matching size and opposite sex";
		if (vendorCup.codeOnly || vendorEjector.codeOnly || vendorFitting.codeOnly ||
			vendorCup.bom.description.indexOf("SAF 40") < 0 ||
			vendorEjector.bom.description.indexOf("SBP 05") < 0 ||
			vendorCup.bom.material != "nitrile rubber NBR" ||
			vendorEjector.bom.material != "plastic" ||
			vendorFitting.bom.material != "brass" ||
			vendorCup.bom.partNumber != "10.01.01.11401" ||
			vendorEjector.bom.partNumber != "10.02.01.00563" ||
			vendorFitting.bom.partNumber != "10.08.02.00203")
			throw "Schmalz parts must retain catalog identity in the BOM";
		if (Math.abs((cast vendorCup.effectiveAreaMm2:Float) - 1150) > 1e-9 ||
			Math.abs(vendorCup.massProperties().mass - 0.0136) > 1e-9 ||
			Math.abs(vendorEjector.massProperties().mass - 0.0075) > 1e-9 ||
			Math.abs(vendorFitting.massProperties().mass - 0.015) > 1e-9)
			throw "Schmalz force-derived area or declared masses are incorrect";
		var example = SchmalzEndEffectorExample.build();
		if (example.validate().length != 0 ||
			example.upstreamChain("cup", "vacuum").join(" ← ").indexOf("fitting/thread") < 0 ||
			example.billOfMaterials().lines().length != 6 ||
			example.massPropertiesAtMount().unaccounted.length != 0)
			throw "Schmalz EOAT must validate with a complete service chain, BOM and mass";
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
