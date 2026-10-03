import cadkit.modeling.AssemblyModel;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.MachineAssemblyDescription;
import machinekit.assembly.MachineAssemblyDescription.SavedValue;
import machinekit.motion.Gearbox;
import machinekit.motion.MotorDriver;
import machinekit.motion.ServoMotor;
import machinekit.robotics.ArmJoint;
import machinekit.robotics.RobotFlange;

/** Motor bindings retain a source member, never a saved copy of its reduction. */
class GearboxTests {
	static function check(value:Bool, message:String):Void { if (!value) throw message; }
	static function definition(machine:MachineAssembly):materia.assembly.AssemblyDefinition {
		var model = new AssemblyModel("mm");
		machine.addTo(model, "");
		return model.definition("gearbox");
	}
	static function fixture(gearbox:String):MachineAssembly {
		var machine = new MachineAssembly();
		machine.addComponent("base", new RobotFlange(50));
		machine.addComponent("motor", new ArmJoint(80, 80, null, ServoMotor.model("GENERIC-SERVO-50W")));
		machine.addComponent("driver", new MotorDriver("GENERIC-SERVO-AMP", 5, 1, 24));
		machine.addComponent("gearbox", new Gearbox(10, 0.9, 60, 40, 8, true));
		machine.addMateOnAxis("turn", "continuous", "base", "face", "motor", "stator", {x: 0, y: 0, z: 1});
		machine.addMate("driver-mount", "fixed", "base", "face", "driver", "mount");
		machine.addMate("gearbox-mount", "fixed", "base", "face", "gearbox", "input");
		machine.addMotor("drive", "turn", "motor", "driver", 0.5, gearbox);
		return machine;
	}
	public static function run():Void {
		var machine = fixture("gearbox");
		var before = definition(machine);
		check(before.actuators != null && before.encoders != null, "A geared servo compiles its actuator and intrinsic sensor");
		check(before.actuators[0].gearRatio == 10 && before.actuators[0].gearEfficiency == 0.9,
			"The actuator derives reduction and efficiency from its member");
		check(before.actuators[0].assumed != null && before.actuators[0].assumed.indexOf("gearbox efficiency") >= 0,
			"Assumed gearbox ratings reach the actuator");
		var edited:MachineAssemblyDescription = haxeon.wire.JsonWire.decode(haxeon.wire.JsonWire.encode(machine.describe()));
		edited.machine.members = [for (member in edited.machine.members) member.occurrence != "gearbox" ? member :
			{occurrence: member.occurrence, material: member.material, source: switch member.source {
				case Typed(id, values): Typed(id, [for (entry in values) switch entry.name {
					case "ratio": {name: entry.name, value: SavedValue.Number(20)};
					case "efficiency": {name: entry.name, value: SavedValue.Number(0.8)};
					case "basis": {name: entry.name, value: SavedValue.Token("stated")};
					case _: entry;
				}]);
				case other: other;
			}}];
		var changed = MachineAssembly.fromDescription(edited), after = definition(changed);
		check(after.actuators != null && after.encoders != null, "A rebuilt geared servo retains its sensor");
		check(after.actuators[0].gearRatio == 20 && after.actuators[0].gearEfficiency == 0.8,
			"Editing the source part recompiles both gearbox ratings");
		check(after.encoders[0].counts == before.encoders[0].counts * 2,
			"Motor-side feedback counts follow the edited reduction");
		check(after.actuators[0].assumed != null && after.actuators[0].assumed.indexOf("gearbox efficiency") < 0,
			"Stated gearbox ratings remove the corresponding assumption");
		var included = new MachineAssembly();
		included.include("unit", changed);
		var nested = MachineAssembly.decode(included.encode()), compiled = definition(nested);
		check(compiled.actuators != null && compiled.actuators[0].gearRatio == 20,
			"Included and reconstructed bindings resolve prefixed gearbox members");
		var rejected = false;
		try { var invalid = fixture("driver"); } catch (_:Dynamic) rejected = true;
		check(rejected, "A driver cannot stand in for a gearbox member");
	}
}
