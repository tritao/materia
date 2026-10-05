import machinekit.gantry.RotaryHead;
import machinekit.gantry.Gantry;
import machinekit.gantry.GantrySpec;
import machinekit.gantry.GantrySpec.GantryHead;
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
		var zeroPin = AssemblyFrames.transformVector(state.worldConnector("outputFlange", "face"), 1, 0, 0);
		state.setJoint("c", Math.PI / 2);
		var face = state.worldConnector("outputFlange", "face");
		near(face.z, c.flangeZero.z, "C rotation keeps flange height");
		var pin = AssemblyFrames.transformVector(face, 1, 0, 0);
		near(pin.x, -zeroPin.y, "C rotates the locating pin X");
		near(pin.y, zeroPin.x, "C rotates the locating pin Y");
		var ca = new CaHead();
		powered(ca);
		state = new AssemblyState(ca.definition());
		state.setJoint("a", Math.PI / 4);
		state.setJoint("c", Math.PI / 2);
		face = state.worldConnector("outputFlange", "face");
		near(face.x, RotaryHead.A_REACH / Math.sqrt(2), "C carries the tilted flange X");
		near(face.y, 0, "C carries the tilted flange Y");
		near(face.z, RotaryHead.A_AXIS + RotaryHead.A_REACH / Math.sqrt(2), "A pivots around its shaft");
		var normal = AssemblyFrames.transformVector(face, 0, 1, 0);
		near(normal.x, 1 / Math.sqrt(2), "CA flange normal X");
		near(normal.y, 0, "CA flange normal Y");
		near(normal.z, 1 / Math.sqrt(2), "CA flange normal Z");
		stopGeometry(ca);
		integratedHeads();
		Sys.println("Rotary heads: C/CA FK, gearbox drives and physical tilt stops passed");
	}

	static function integratedHeads():Void {
		var plain = new Gantry(new GantrySpec(300, 300, 150));
		for (selection in [GantryHead.C, GantryHead.CA]) {
			var gantry = new Gantry(new GantrySpec(300, 300, 150, null, null, null, true,
				"MGN12C", 23, "HFS5-4040", "HFS5-4040", false, selection));
			var head = gantry.head;
			if (head == null || head.hasTilt != (selection == GantryHead.CA)) throw "GantrySpec selects its rotary head";
			var flange = gantry.connector("toolFlange");
			if (flange.instanceId != "head/outputFlange" || flange.connectorName != "face") throw "The tool must use the head's output flange";
			var model = new AssemblyModel(); gantry.addTo(model, "");
			var definition = model.definition();
			if (definition.actuators == null || definition.actuators.length != 4 + head.rotaryJoints.length)
				throw "The rotary head and every translational motor need a cabinet drive";
			for (actuator in definition.actuators) if (StringTools.startsWith(actuator.joint, "head/") &&
				(!(actuator.maxRate > 0) || !(actuator.maxEffort > 0))) throw "Integrated head limits come from its drive ratings";
			var state = new AssemblyState(definition);
			var zero = state.worldConnector("head/outputFlange", "face");
			near(zero.z, plain.flangeZero.z, "Frame elevation preserves the tool's nominal Z envelope");
			for (x in [0.0, 300.0]) for (y in [0.0, 300.0]) for (z in [0.0, 150.0]) {
				state.setJoint("x", x); state.setJoint("y", y); state.setJoint("z", z);
				state.setJoint("head/c", Math.PI / 2);
				if (head.hasTilt) state.setJoint("head/a", Math.PI / 4);
				var face = state.worldConnector("head/outputFlange", "face");
				near(face.z, zero.z - z + (head.hasTilt ? RotaryHead.A_REACH * (1 - 1 / Math.sqrt(2)) : 0), "XYZ carries the rotating tool in Z");
				var origin = state.worldPose("head/mount");
				near(origin.x, x, "X carries the complete head");
				near(origin.y, gantry.flangeZero.y + y, "Y carries the complete head");
			}
			headClearance(gantry, state);
			var report = gantry.check();
			report.throwIfErrors();
		}
	}

	/** Solid checks at XYZ corners and both rotary cable limits. */
	static function headClearance(gantry:Gantry, state:AssemblyState):Void {
		var head:RotaryHead = cast gantry.head;
		var parts = new PosedParts();
		var carriers = ["frameFront", "frameBack", "frameLeft", "frameRight", "beam", "railX",
			"zColumn", "railZ", "xCarriage", "zCarriage"];
		var moving = [for (member in gantry.components()) if (StringTools.startsWith(member.id, "head/")) member];
		try {
			for (x in [0.0, 300.0]) for (y in [0.0, 300.0]) for (z in [0.0, 150.0]) {
				state.setJoint("x", x); state.setJoint("y", y); state.setJoint("z", z);
				for (c in [-Math.PI, 0.0, Math.PI]) for (a in (head.hasTilt ? [-Math.PI / 2, 0.0, Math.PI / 2] : [0.0])) {
					state.setJoint("head/c", c);
					if (head.hasTilt) state.setJoint("head/a", a);
					for (member in moving) {
						var shape = parts.posed(member.component, state.worldPose(member.id));
						var box = PosedParts.boxOf(shape);
						for (id in carriers) {
							var fixed = parts.posed(gantry.component(id), state.worldPose(id));
							var overlap = PosedParts.commonVolume(shape, box, fixed, PosedParts.boxOf(fixed));
							fixed.close();
							if (overlap > 1e-3) { shape.close(); throw 'Head collision ${member.id}/$id at $x,$y,$z C=$c A=$a: $overlap mm³'; }
						}
						shape.close();
					}
				}
			}
		} catch (error:Dynamic) { parts.close(); throw error; }
		parts.close();
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
				var flange = parts.posed(head.flange, state.worldPose("outputFlange"));
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
