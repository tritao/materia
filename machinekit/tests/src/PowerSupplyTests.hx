import cadkit.modeling.AssemblyModel;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.MachineAssemblyDescription;
import machinekit.assembly.MachineAssemblyDescription.MemberSource;
import machinekit.assembly.MachineAssemblyDescription.SavedValue;
import machinekit.motion.MotorDriver;
import machinekit.motion.NemaStepper;
import machinekit.motion.PowerSupply;
import machinekit.robotics.RobotFlange;

/** Voltage follows the current service graph, including late wiring and included assemblies. */
class PowerSupplyTests {
	static function check(value:Bool, message:String):Void {
		if (!value) throw message;
	}
	static function near(actual:Float, expected:Float, message:String):Void {
		check(Math.abs(actual - expected) <= 1e-9 * Math.max(1, Math.abs(expected)), '$message: $actual, expected $expected');
	}
	static function fixture(stated:Null<Float>):MachineAssembly {
		var machine = new MachineAssembly();
		machine.addComponent("base", new RobotFlange(50));
		machine.addComponent("motor", NemaStepper.frame(23));
		machine.addComponent("driver", new MotorDriver("GENERIC-DM542", 2.8, 16, stated));
		machine.addMateOnAxis("turn", "continuous", "base", "face", "motor", "mountFace", {x: 0, y: 0, z: 1});
		machine.addMate("driver-mount", "fixed", "base", "face", "driver", "mount");
		machine.addMotor("drive", "turn", "motor", "driver");
		return machine;
	}
	static function supply(machine:MachineAssembly, voltage:Float):Void {
		machine.addComponent("supply", new PowerSupply(voltage, 10));
		machine.addMate("supply-mount", "fixed", "base", "face", "supply", "mount");
		machine.connectPorts("driver-power", "supply", "power1", "driver", "power");
	}
	static function definition(machine:MachineAssembly):materia.assembly.AssemblyDefinition {
		var model = new AssemblyModel("mm");
		machine.addTo(model, "");
		return model.definition("power");
	}
	static function rate(machine:MachineAssembly):Float {
		var actuators = definition(machine).actuators;
		if (actuators == null || actuators.length != 1) throw "A powered fixture needs one actuator";
		return actuators[0].maxRate;
	}
	static function rejects(action:Void->Void, message:String):Void {
		var failed = false;
		try action() catch (_:Dynamic) failed = true;
		check(failed, message);
	}
	public static function run():Void {
		var nominal = NemaStepper.frame(23).usableSpeed(24);
		var fallback = fixture(24);
		near(rate(fallback), nominal, "An unmodelled supply uses the driver's stated voltage");
		supply(fallback, 48);
		near(rate(fallback), 2 * nominal, "Late wiring replaces the fallback on the next compilation");
		var powered = fixture(null);
		rejects(() -> { var result = rate(powered); }, "A driver without a fallback must have a powered service graph");
		supply(powered, 24);
		near(rate(powered), nominal, "A wired supply gives the nominal motor curve");
		var source = powered.upstream("driver", "power");
		check(!source.external && source.port.instanceId == "supply", "Power traces to the modelled supply member");
		var rebuilt = MachineAssembly.decode(powered.encode());
		near(rate(rebuilt), nominal, "A saved powered machine resolves its service graph again");
		var edited:MachineAssemblyDescription = haxeon.wire.JsonWire.decode(haxeon.wire.JsonWire.encode(powered.describe()));
		edited.machine.members = [for (member in edited.machine.members) member.occurrence != "supply" ? member :
			{occurrence: member.occurrence, material: member.material, source: switch member.source {
				case Typed(id, values): Typed(id, [for (entry in values) entry.name != "voltage" ? entry :
					{name: "voltage", value: SavedValue.Number(48)}]);
				case other: other;
			}}];
		near(rate(MachineAssembly.fromDescription(edited)), 2 * nominal, "Editing the supply rebuilds the torque-speed curve");
		var included = new MachineAssembly();
		included.include("unit", powered);
		near(rate(included), nominal, "Included power connections retain their member prefixes");
		near(rate(MachineAssembly.decode(included.encode())), nominal, "Nested reconstruction keeps its supplied voltage");
		var outside = fixture(24);
		outside.exposePort("power", "driver", "power");
		near(rate(outside), nominal, "An exposed boundary still needs the driver's stated voltage");
		var excessive = fixture(null);
		supply(excessive, 60);
		rejects(() -> { var result = rate(excessive); }, "A supply outside the driver voltage range must be rejected");
		var unknown = fixture(24);
		unknown.addComponent("upstream-driver", new MotorDriver("GENERIC-DM542", 2.8, 16, 24));
		unknown.addMate("upstream-mount", "fixed", "base", "face", "upstream-driver", "mount");
		unknown.connectPorts("unknown-voltage", "upstream-driver", "motor", "driver", "power");
		rejects(() -> { var result = rate(unknown); }, "A modelled source without an output voltage cannot silently use the fallback");
		var servo = new MachineAssembly();
		servo.addComponent("base", new RobotFlange(50));
		servo.addComponent("motor", new machinekit.robotics.ArmJoint(80, 80, null,
			machinekit.motion.ServoMotor.model("GENERIC-SERVO-50W")));
		servo.addComponent("driver", new MotorDriver("GENERIC-SERVO-AMP", 5, 1, 24));
		servo.addMateOnAxis("turn", "continuous", "base", "face", "motor", "stator", {x: 0, y: 0, z: 1});
		servo.addMate("driver-mount", "fixed", "base", "face", "driver", "mount");
		servo.addMotor("drive", "turn", "motor", "driver");
		servo.addComponent("first", new machinekit.motion.ShaftEncoder(4096));
		servo.addComponent("second", new machinekit.motion.ShaftEncoder(8192));
		servo.addMate("first-mount", "fixed", "base", "face", "first", "mount");
		servo.addMate("second-mount", "fixed", "base", "face", "second", "mount");
		servo.addEncoder("z-feedback", "turn", "first", "drive");
		servo.addEncoder("a-feedback", "turn", "second", "drive");
		var restored = definition(MachineAssembly.decode(servo.encode()));
		check(restored.encoders != null && restored.encoders.length == 2 &&
			restored.actuators != null && restored.actuators[0].encoder == "a-feedback",
			"Feedback rewiring survives canonical sensor sorting without losing either explicit sensor");
	}
}
