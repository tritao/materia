import machinekit.assembly.MachineAssembly;
import machinekit.fluid.CylinderPhaseSpeeds;
import machinekit.pressbrake.HydraulicCylinder;
import machinekit.pressbrake.HydraulicSupply;
import machinekit.pressbrake.HydraulicValve;
import machinekit.milling.MillPanel;
import cadkit.modeling.AssemblyModel;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinition.AssemblyHydraulicDrive;

/** Hydraulic service, guide alignment and actual hose routing rebuild the saved drive. */
class HydraulicPartTests {
	public static function main():Void run();
	public static function run():Void {
		for (distributed in [false, true]) for (reversed in [false, true]) {
			var assembly = fixture(reversed, 10000000, distributed);
			var drive = compiled(assembly);
			close(drive.pressurePa, 10000000, "regulated hydraulic service");
			close(drive.bore, 160, "installed cylinder bore");
			close(drive.rod, 100, "installed rod diameter");
			close(drive.stroke, 100, "installed guide travel");
			close(drive.approachSpeed, 80, "approach phase");
			close(drive.workingSpeed, 8, "working phase");
			close(drive.returnSpeed, 70, "return phase");
			close(drive.changePoint, 68.5, "explicit speed-change travel");
			if (drive.channelA != (reversed ? "valve/coilB" : "valve/coilA") ||
				drive.channelB != (reversed ? "valve/coilA" : "valve/coilB") || drive.extendSign != 1)
				throw "Hydraulic hose routing did not map full-bore and annular coils";
			MachineAssemblyDescriptionTests.roundTrip(assembly, reversed ? "reversed hydraulic hoses" : "hydraulic service", false);
			var rebuilt = MachineAssembly.decode(assembly.encode());
			close(compiled(rebuilt).changePoint, 68.5, "phase profile reconstructed from current schema");
		}
		close(compiled(fixture(false, 0)).pressurePa, 0, "unpressurized service remains explicit");
		var original = fixture(false, 10000000, true);
		var saved = original.describe();
		var records = saved.machine.cylinders;
		if (records == null || records.length != 1) throw "Saved hydraulic fixture lost its cylinder record";
		var phases = records[0].phases;
		if (phases == null) throw "Saved hydraulic fixture lost its authored flow setup";
		phases.working = 4; phases.changePoint = 60;
		var edited = compiled(MachineAssembly.fromDescription(saved));
		close(edited.workingSpeed, 4, "edited flow setup rebuilt");
		close(edited.changePoint, 60, "edited change point rebuilt");
		close(compiled(original).workingSpeed, 8, "description owns its copied flow setup");
		var outer = new MachineAssembly();
		outer.include("press", original);
		var nested = compiled(outer);
		if (nested.channelA != "press/valve/coilA" || nested.channelB != "press/valve/coilB")
			throw "Nested hydraulic bindings did not scope valve channels";
		close(nested.changePoint, 68.5, "nested flow setup retained");
		MachineAssemblyDescriptionTests.roundTrip(outer, "nested hydraulic service", false);
		var missingPhases = unbound(false, 10000000);
		refuses(() -> missingPhases.addCylinder("ram-drive", "ram", "cylinder", "valve"), "Hydraulic binding needs explicit phase speeds");
		var wrongStroke = unbound(false, 10000000);
		refuses(() -> wrongStroke.addCylinder("ram-drive", "ram", "cylinder", "valve", new CylinderPhaseSpeeds(120, 80, 8, 70, 68.5)),
			"Hydraulic binding needs explicit phase speeds");
		var wrongValve = unbound(false, 10000000);
		wrongValve.addComponent("pneumatic-valve", new machinekit.pneumatic.SolenoidValve());
		refuses(() -> wrongValve.addCylinder("ram-drive", "ram", "cylinder", "pneumatic-valve", new CylinderPhaseSpeeds(100, 80, 8, 70, 68.5)),
			"Hydraulic cylinder needs a closed-centre hydraulic valve");
		refuses(() -> { var invalid = new HydraulicSupply(-1); }, "Hydraulic supply pressure must be finite and nonnegative");
		var cylinder = new HydraulicCylinder(160, 100, 100);
		close(cylinder.extendForce(10000000), 201061.92982974676, "full-bore pressure force");
		close(cylinder.retractForce(10000000), 122522.11349000193, "annular pressure force");
		var valve = new HydraulicValve();
		checkValve(valve.valveSpec());
		trace("Hydraulic parts and bindings passed");
	}

	static function checkValve(capability:Null<machinekit.fluid.ValveSpec>):Void {
		if (capability == null) throw "Hydraulic valve lost its typed capability";
		switch capability.controlSpec() {
			case ClosedCentre:
			case PressureEffort(_, _): throw "Hydraulic valve declares pneumatic latching";
		}
	}

	static function fixture(reversed:Bool, pressurePa:Float, distributed:Bool = false):MachineAssembly {
		var result = unbound(reversed, pressurePa, distributed);
		result.addCylinder("ram-drive", "ram", "cylinder", "valve", new CylinderPhaseSpeeds(100, 80, 8, 70, 68.5));
		return result;
	}
	static function unbound(reversed:Bool, pressurePa:Float, distributed:Bool = false):MachineAssembly {
		var result = new MachineAssembly();
		result.addComponent("base", new MillPanel(100, 100, 20));
		result.addComponent("carriage", new MillPanel(20, 20, 20));
		result.addMemberConnector("base", "guide", AssemblyFrames.identity());
		result.addMemberConnector("carriage", "guide", AssemblyFrames.identity());
		result.addMateOnAxis("ram", "prismatic", "base", "guide", "carriage", "guide",
			{x: 0, y: 0, z: 1}, 0, {lower: 0, upper: 100, velocity: null, effort: null, overtravel: 0});
		var cylinder = new HydraulicCylinder(160, 100, 100);
		result.addComponent("cylinder", cylinder, AssemblyFrames.translation(0, 0, -cylinder.bodyLength - cylinder.stemLength));
		result.addComponent("supply", new HydraulicSupply(pressurePa));
		result.addComponent("valve", new HydraulicValve());
		result.connectPorts("pressure", "supply", "P", "valve", "P");
		result.connectPorts("return", "valve", "T", "supply", "T");
		if (distributed) {
			for (chamber in ["A", "B"]) {
				var id = 'distribution$chamber';
				result.addComponent(id, new machinekit.pressbrake.HydraulicManifold(2));
				var outlet = reversed ? (chamber == "A" ? "B" : "A") : chamber;
				result.connectPorts('$id-feed', "valve", outlet, id, "input");
				result.connectPorts('$id-cylinder', id, "out1", "cylinder", chamber);
			}
		} else {
			result.connectPorts("full-bore", "valve", reversed ? "B" : "A", "cylinder", "A");
			result.connectPorts("annular", "valve", reversed ? "A" : "B", "cylinder", "B");
		}
		return result;
	}
	static function compiled(assembly:MachineAssembly):AssemblyHydraulicDrive {
		var model = new AssemblyModel("mm");
		assembly.addTo(model, "");
		var definition = model.definition("hydraulic-fixture");
		if (definition.actuators == null || definition.actuators.length != 1) throw "Hydraulic fixture lost its cylinder binding";
		var actuator = definition.actuators[0];
		var drive = actuator.hydraulic;
		if (actuator.drive != "hydraulic" || drive == null || actuator.pneumatic != null)
			throw "Cylinder binding did not compile a typed hydraulic drive";
		close(actuator.maxEffort, drive.pressurePa * 0.020106192982974676, "derived installed full-bore effort");
		close(actuator.maxRate, 80, "maximum phase rate");
		return drive;
	}
	static function refuses(action:Void->Void, reason:String):Void {
		var failure:Null<String> = null;
		try action() catch (error:Dynamic) failure = Std.string(error);
		if (failure == null || failure.indexOf(reason) < 0) throw 'Expected refusal "$reason", got "$failure"';
	}
	static function close(actual:Float, expected:Float, message:String):Void {
		if (!Math.isFinite(actual) || Math.abs(actual - expected) > 1e-6)
			throw '$message: expected $expected, got $actual';
	}
}
