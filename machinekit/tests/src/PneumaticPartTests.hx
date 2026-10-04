import machinekit.assembly.MachineAssembly;
import machinekit.milling.MillPanel;
import cadkit.modeling.AssemblyModel;
import materia.assembly.AssemblyFrames;
import machinekit.pneumatic.PneumaticCylinder;
import machinekit.pneumatic.AirSupply;
import machinekit.pneumatic.SolenoidValve;

/** Cylinder areas, valve state and air-service reconstruction before runtime integration. */
class PneumaticPartTests {
	public static function run():Void {
		var door = new PneumaticCylinder("ISO6432", 25, 460, 300);
		var vise = new PneumaticCylinder("ISO15552", 32, 6, 30);
		close(door.extendForce(600000), 294.524311274, "25 mm advance force");
		close(door.retractForce(600000), 247.40042147, "25 mm retract force");
		close(vise.extendForce(600000), 482.54863159, "32 mm clamp force");
		close(vise.retractForce(600000), 414.69023027, "32 mm retract force");
		close(door.extendForce(0), 0, "no pressure gives no force");
		close(door.extendForce(300000), door.extendForce(600000) / 2, "force follows gauge pressure");
		// Independent rounded theoretical-force tables, Festo DSNU / DSBC at 6 bar.
		for (family in ["ISO6432", "ISO15552"]) {
			var bores = family == "ISO6432" ? [8, 10, 12, 16, 20, 25] : [32, 40, 50, 63, 80, 100, 125];
			var advance = family == "ISO6432" ? [30, 47, 68, 121, 189, 295] : [483, 754, 1178, 1870, 3016, 4712, 7363];
			var retract = family == "ISO6432" ? [23, 40, 51, 104, 158, 247] : [415, 633, 990, 1682, 2721, 4418, 6881];
			for (i in 0...bores.length) {
				var cylinder = new PneumaticCylinder(family, bores[i], 100, 300);
				if (Math.abs(cylinder.extendForce(600000) - advance[i]) > 0.6 ||
					Math.abs(cylinder.retractForce(600000) - retract[i]) > 0.6)
					throw '$family bore ${bores[i]} disagrees with reference force table';
				var recipe = cylinder.componentType();
				if (recipe == null) throw "Cylinder recipe is missing";
				var rebuilt = recipe.create(cylinder.values()).pneumaticCylinderSpec();
				if (rebuilt == null) throw "Cylinder recipe did not rebuild its typed pneumatic specification";
				close(rebuilt.retractForce(600000), cylinder.retractForce(600000), "reconstructed annular force");
			}
		}
		var valve = new SolenoidValve(false, false);
		if (valve.routesToA(false, false, true) || !valve.routesToA(true, false, false)) throw "Spring-return valve routing";
		var latched = new SolenoidValve(true, true);
		if (!latched.routesToA(false, false, true) || latched.routesToA(false, false, false) ||
			!latched.routesToA(true, false, false) || latched.routesToA(false, true, true) ||
			!latched.routesToA(true, true, true)) throw "Double-solenoid valve latching";
		var assembly = new MachineAssembly();
		assembly.addComponent("supply", new AirSupply());
		assembly.addComponent("valve", valve);
		assembly.addComponent("manifold", new machinekit.pneumatic.PneumaticManifold(2));
		assembly.addComponent("cylinder", door);
		assembly.addComponent("rod", door.movingRod());
		var hose:machinekit.component.BomItem = {partNumber: "REFERENCE-TUBE-6", description: "Assumed 6 mm pneumatic hose", quantity: 1, material: null};
		assembly.connectPorts("supplyHose", "supply", "air", "manifold", "input", hose);
		assembly.connectPorts("valveHose", "manifold", "out1", "valve", "P", hose);
		assembly.connectPorts("advanceHose", "valve", "A", "cylinder", "A", hose);
		assembly.connectPorts("retractHose", "valve", "B", "cylinder", "B", hose);
		var upstream = assembly.upstream("cylinder", "A");
		if (upstream.port.instanceId != "supply" || upstream.port.portName != "air") throw "Cylinder pressure does not trace to supply";
		close(assembly.airPressure("cylinder", "A"), 600000, "pressure traced through valve");
		close(assembly.airPressure("cylinder", "B"), 600000, "pressure traced through second hose");
		var lines = assembly.billOfMaterials().lines();
		var quantity = 0.0;
		for (line in lines) if (line.partNumber == hose.partNumber) quantity += line.quantity;
		close(quantity, 4, "connected hoses enter BOM");
		MachineAssemblyDescriptionTests.roundTrip(assembly, "pneumatic service", false);

		// A cylinder binds to a real guide coordinate. Its tip at home lies on the carriage's
		// guide connector and its axis follows the joint, so no actuator-only travel is invented.
		var bound = new MachineAssembly();
		bound.addComponent("base", new MillPanel(100, 100, 20));
		bound.addComponent("carriage", new MillPanel(20, 20, 20));
		bound.addMemberConnector("base", "guide", AssemblyFrames.identity());
		bound.addMemberConnector("carriage", "guide", AssemblyFrames.identity());
		bound.addMateOnAxis("door-guide", "prismatic", "base", "guide", "carriage", "guide",
			{x: 0, y: 0, z: 1}, 0, {lower: 0, upper: 460, velocity: null, effort: null, overtravel: 0});
		bound.addComponent("cylinder", door, AssemblyFrames.translation(0, 0, -door.bodyLength - door.stemLength));
		bound.addComponent("supply", new AirSupply());
		bound.addComponent("manifold", new machinekit.pneumatic.PneumaticManifold(1));
		bound.addComponent("valve", valve);
		bound.connectPorts("source-line", "supply", "air", "manifold", "input");
		bound.connectPorts("valve-line", "manifold", "out1", "valve", "P");
		bound.connectPorts("A-line", "valve", "A", "cylinder", "A");
		bound.connectPorts("B-line", "valve", "B", "cylinder", "B");
		bound.addCylinder("door-drive", "door-guide", "cylinder", "valve");
		var model = new AssemblyModel("mm");
		bound.addTo(model, "");
		var definition = model.definition("pneumatic-fixture");
		if (definition.schemaVersion != materia.assembly.AssemblyDefinitionCodec.PROCESS_VERSION ||
			definition.actuators == null || definition.actuators.length != 1)
			throw "Process actuator did not produce the process-format drive";
		var drive = definition.actuators[0];
		if (drive.drive != "pneumatic" || drive.pneumatic == null || drive.pneumatic.channelA != "valve/coilA" ||
			drive.pneumatic.channelB != null || drive.pneumatic.normallyToA || drive.pneumatic.extendSign != 1)
			throw "Derived cylinder process binding has wrong valve channels or direction";
		close(drive.maxEffort, door.extendForce(600000), "bound cylinder effort");
		close(drive.maxRate, 300, "bound cylinder speed");
		MachineAssemblyDescriptionTests.roundTrip(bound, "bound cylinder", false);

		var failed = false;
		try new PneumaticCylinder("ISO6432", 32, 100, 300) catch (_:Dynamic) failed = true;
		if (!failed) throw "Cylinder accepted bore outside its family";
		failed = false;
		try door.extendForce(-1) catch (_:Dynamic) failed = true;
		if (!failed) throw "Cylinder accepted negative pressure";
		trace('Pneumatic parts passed: door ${door.extendForce(600000)}/${door.retractForce(600000)} N, vise ${vise.extendForce(600000)} N');
	}
	static function close(actual:Float, expected:Float, label:String):Void {
		if (Math.abs(actual - expected) > 1e-6) throw '$label: $actual != $expected';
	}
}
