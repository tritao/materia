import machinekit.gantry.RotaryHead;
import machinekit.gantry.RotaryHead.CHead;
import machinekit.gantry.RotaryHead.CaHead;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.motion.PowerSupply;
import machinekit.motion.MotorDriver;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.AssemblyModel;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinitionCodec;

class RotaryHeadCheck {
	static function near(value:Float, expected:Float, message:String):Void
		if (Math.abs(value - expected) > 1e-7) throw '$message: $value, expected $expected';

	static function powered(head:RotaryHead):MachineAssembly {
		var machine = new MachineAssembly();
		machine.addComponent("supply", new PowerSupply(24, 20, head.rotaryJoints.length), AssemblyFrames.translation(-300, 0, 0));
		machine.include("head", head);
		var drivers:Array<String> = [];
		for (index in 0...head.rotaryJoints.length) {
			var id = 'driver$index';
			machine.addComponent(id, new MotorDriver("GENERIC-SERVO-AMP", 5), AssemblyFrames.translation(-250, index * 80, 0));
			machine.connectPorts('power$index', "supply", 'power${index + 1}', id, "power");
			drivers.push(id);
		}
		head.bindDrives(machine, "head", drivers);
		var model = new AssemblyModel();
		machine.addTo(model, "");
		var actuators:Array<materia.assembly.AssemblyDefinition.AssemblyActuator> = model.definition().actuators;
		if (actuators == null) throw "Head drives are missing from the assembly export";
		if (actuators.length != head.rotaryJoints.length) throw "Each head joint must have its own derived drive";
		for (actuator in actuators) {
			if (actuator.drive != "servo" || !(actuator.maxEffort > 0) || !(actuator.maxRate > 0))
				throw "Head limits must come from servo and gearbox ratings";
		}
		return machine;
	}

	static function run():Void {
		var c = new CHead();
		powered(c);
		var state = new AssemblyState(c.definition());
		var zeroPin = AssemblyFrames.transformVector(state.worldConnector("flange", "face"), 1, 0, 0);
		state.setJoint("c", Math.PI / 2);
		var face = state.worldConnector("flange", "face");
		near(face.z, c.flangeZero.z, "C rotation keeps flange height");
		var pin = AssemblyFrames.transformVector(face, 1, 0, 0);
		near(pin.x, -zeroPin.y, "C rotates the locating pin X");
		near(pin.y, zeroPin.x, "C rotates the locating pin Y");
		var ca = new CaHead();
		powered(ca);
		state = new AssemblyState(ca.definition());
		state.setJoint("a", Math.PI / 4);
		state.setJoint("c", Math.PI / 2);
		face = state.worldConnector("flange", "face");
		near(face.x, RotaryHead.A_REACH / Math.sqrt(2), "C carries the tilted flange X");
		near(face.y, 0, "C carries the tilted flange Y");
		near(face.z, RotaryHead.A_AXIS + RotaryHead.A_REACH / Math.sqrt(2), "A pivots around its shaft");
		var normal = AssemblyFrames.transformVector(face, 0, 1, 0);
		near(normal.x, 1 / Math.sqrt(2), "CA flange normal X");
		near(normal.y, 0, "CA flange normal Y");
		near(normal.z, 1 / Math.sqrt(2), "CA flange normal Z");
		stopGeometry(ca);
		Sys.println("Rotary heads: C/CA FK, gearbox drives and physical tilt stops passed");
	}

	static function stopGeometry(head:CaHead):Void {
		var a = head.rotaryJoints[1];
		if (a.hardStop == null || a.hardStop <= a.upper) throw "Cable-limited A travel needs clearance before its stop";
		var hardStop:Float = cast a.hardStop;
		var definition = AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(head.definition()));
		for (joint in definition.joints) if (joint.id == "a") {
			joint.limits.lower = -hardStop - 0.05;
			joint.limits.upper = hardStop + 0.05;
		}
		var state = new AssemblyState(definition), parts = new PosedParts();
		try {
			for (side in [-1, 1]) for (outside in [false, true]) {
				state.setJoint("a", side * (outside ? hardStop + 0.02 : a.upper));
				var flange = parts.posed(head.flange, state.worldPose("flange"));
				var id = side > 0 ? "aStopNegative" : "aStopPositive";
				var stop = parts.posed(head.component(id), state.worldPose(id));
				var common = flange.intersect(stop);
				var overlap = common.volume();
				common.close(); flange.close(); stop.close();
				if (outside ? overlap <= 1e-3 : overlap > 1e-3)
					throw 'Tilt stop geometry disagrees with travel: side=$side outside=$outside overlap=$overlap';
			}
		} catch (error:Dynamic) { parts.close(); throw error; }
		parts.close();
	}

	static function main():Void run();
}
