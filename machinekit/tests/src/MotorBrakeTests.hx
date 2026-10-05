import machinekit.assembly.MachineAssembly;
import machinekit.motion.ServoMotor;
import machinekit.motion.MotorDriver;
import machinekit.motion.PowerSupply;
import machinekit.motion.ShaftBrakePart;
import machinekit.robotics.RobotFlange;
import machinekit.document.MachineAssemblyDocuments;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.modeling.AssemblyModel;

/** Brake recipes and bindings survive saved and nested mechanical assemblies. */
class MotorBrakeTests {
	static function fixture():MachineAssembly {
		var assembly = new MachineAssembly();
		assembly.addComponent("base", new RobotFlange(50));
		assembly.addComponent("servo", ServoMotor.model("GENERIC-SERVO-750W"));
		assembly.addComponent("driver", new MotorDriver("GENERIC-AC-SERVO-AMP", new machinekit.power.PhaseRmsCurrent(20)));
		assembly.addComponent("supply", new PowerSupply(new machinekit.power.DcVoltage(310), new machinekit.power.DcCurrent(20), 1, 120, 60, 30, 4000));
		assembly.addComponent("brake", new ShaftBrakePart(19, "shell"));
		assembly.connectPorts("power", "supply", "power1", "driver", "power");
		assembly.addMateOnAxis("turn", "continuous", "base", "face", "servo", "shaftAxis", {x: 0, y: 1, z: 0});
		assembly.addMotor("motor", "turn", "servo", "driver");
		assembly.bindMotorBrake("motor", "brake", "brake.release", "brake.released", 250, 30);
		return assembly;
	}

	static function checkBinding(assembly:MachineAssembly, prefix:String):Void {
		var model = new AssemblyModel("mm");
		assembly.addTo(model, "");
		var definition = model.definition("brake-test");
		var found = false, contact = false;
		if (definition.actuators != null) for (actuator in definition.actuators) {
			var brake = actuator.brake;
			if (actuator.id == prefix + "motor" && brake != null && brake.id == prefix + "brake" &&
				brake.releaseChannel == prefix + "brake.release" && brake.releasedSensor == prefix + "brake.released" &&
				brake.holdingEffort == 4 && brake.releaseDelay == 0.05 && brake.applicationDelay == 0.03 &&
				brake.releaseTimeoutMs == 250 && brake.feedbackMaxAgeMs == 30) found = true;
		}
		if (definition.sensors != null) for (sensor in definition.sensors)
			if (sensor.id == prefix + "brake.released" && sensor.kind == "brake_released" &&
				sensor.joint == prefix + "turn") contact = true;
		if (!found || !contact) throw "Saved brake lost its physical rating, controller policy, scoped names or contact";
	}

	public static function run():Void {
		var assembly = fixture();
		checkBinding(assembly, "");
		MachineAssemblyDescriptionTests.roundTrip(assembly, "braked servo");
		var outer = new MachineAssembly();
		outer.include("mill", assembly);
		var rebuilt = MachineAssembly.decode(outer.encode());
		checkBinding(rebuilt, "mill/");
		checkBinding(rebuilt.subassemblies()[0].assembly, "");
		var document = new Document();
		var root = MachineAssemblyDocuments.defineAssembly(document, outer);
		var reopened = DocumentCodec.decode(DocumentCodec.encode(document), false, false);
		checkBinding(MachineAssemblyDocuments.rebuildAssembly(reopened.element(root.id)), "mill/");
		reopened.close(); document.close();
		var pair = new MachineAssembly();
		pair.include("first", assembly); pair.include("second", assembly);
		checkBinding(pair, "first/"); checkBinding(pair, "second/");
		for (role in ["hub", "disc", "front", "shell", "back"]) {
			var pieces = ShaftBrakePart.sectorCount(19, role);
			for (sector in 0...pieces) {
				var part = new ShaftBrakePart(19, role, sector);
				var recipe = part.componentType();
				if (recipe == null) throw "Brake part has no rebuild recipe";
				var restored = recipe.create(part.values());
				var originalSolid = part.geometry(), restoredSolid = restored.geometry();
				var volume = originalSolid.volume();
				if (!(volume > 0) || Math.abs(restoredSolid.volume() - volume) > 1e-7)
					throw "Brake sector or rotor solid changed after recipe rebuild";
				originalSolid.close(); restoredSolid.close();
			}
		}
		Sys.println("Motor brake recipe, saved binding and nested scope checks passed");
	}
}
