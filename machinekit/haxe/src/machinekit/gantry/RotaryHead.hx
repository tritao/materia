package machinekit.gantry;

import machinekit.assembly.AxisBuilder;
import machinekit.assembly.MachineAssembly;
import machinekit.gantry.GantryParts.GantryPlate;
import machinekit.gantry.GantryParts.GantryBoredBracket;
import machinekit.gantry.GantryParts.GantryIdlerPin;
import machinekit.motion.ServoMotor;
import machinekit.motion.Gearbox;
import machinekit.robotics.RobotFlange;
import machinekit.standard.DeepGrooveBearing;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** A head's travel is the intersection of its cable allowance and mechanical stop. */
typedef RotaryHeadJoint = {
	var id:String;
	var motor:String;
	var gearbox:String;
	var lower:Float;
	var upper:Float;
	var cableWrap:Float;
	var hardStop:Null<Float>;
}

/** Servo/gearhead mechanics in a local frame with +Z toward the tool.
 * Drivers and their power wiring belong to the carrying machine's fixed cabinet.
 * Dimensions, cable allowances and generic drive ratings are stated design assumptions.
 */
class RotaryHead extends AxisBuilder {
	public final rotaryJoints:Array<RotaryHeadJoint> = [];
	public final flange:RobotFlange;
	public final flangeZero:AssemblyFrame;
	public final hasTilt:Bool;
	/** Conservative radial clearance for every permitted C/A pose, in millimetres. */
	public final radialEnvelopeMm:Float;
	/** Furthest extent toward the tool over permitted poses, in millimetres. */
	public final axialEnvelopeMm:Float;
	public static inline var C_OUTPUT:Float = 133;
	public static inline var A_AXIS:Float = 255;
	public static inline var A_REACH:Float = 70;
	public static inline var A_STOP_TOP:Float = 210;

	public function new(hasTilt:Bool = false, cCableWrap:Float = Math.PI, aCableWrap:Float = Math.PI / 2) {
		super();
		for (value in [cCableWrap, aCableWrap])
			if (!Math.isFinite(value) || value <= 0 || value > Math.PI)
				throw "A rotary head cable allowance must be positive and at most one full turn";
		this.hasTilt = hasTilt;
		flange = new RobotFlange(50);
		var servo = ServoMotor.model("GENERIC-SERVO-50W");
		radialEnvelopeMm = hasTilt ? Math.sqrt(Math.pow(126 + servo.rating.bodyLength, 2) +
			Math.pow(servo.rating.bodyDiameter / 2, 2)) : Math.sqrt(50 * 50 + 40 * 40);
		axialEnvelopeMm = hasTilt ? A_AXIS + Math.max(Math.sqrt(A_REACH * A_REACH +
			Math.pow(flange.flangeDiameter / 2, 2)), Math.sqrt(Math.pow(A_REACH + flange.pilotHeight, 2) +
			Math.pow(flange.pilotDiameter / 2, 2))) : C_OUTPUT + flange.thickness + flange.pilotHeight;
		place("mount", new GantryBoredBracket("rotary head mount", 100, 80, 8,
			AssemblyFrames.identity(), flange.pilotDiameter + 0.5), AssemblyFrames.identity());
		addMemberConnector("mount", "mount", machinekit.component.Solids.axial(0, 0, 0));
		exposeConnector("mount", "mount", "mount");
		for (side in [-1, 1]) attach(side < 0 ? "supportLeft" : "supportRight",
			new GantryPlate("head motor support", 20, 80, 77), AssemblyFrames.translation(side * 40, 0, 8), "mount");
		attach("motorPlate", new GantryBoredBracket("head motor plate", 100, 80, 8,
			AssemblyFrames.identity(), 10.5), AssemblyFrames.translation(0, 0, 85), "supportLeft");
		attach("cMotor", ServoMotor.model("GENERIC-SERVO-50W"), AssemblyFrames.translation(0, 0, 85), "motorPlate");
		attach("cGearbox", new Gearbox(130, 0.85, 60, 40, 10, true),
			AssemblyFrames.translation(0, 0, 93), "motorPlate");
		turn("c", "cGearbox", "cShaft", new GantryIdlerPin(10, 18),
			AssemblyFrames.translation(0, 0, C_OUTPUT), {x: 0, y: 0, z: 1}, cCableWrap, null, "cMotor", "cGearbox");
		if (hasTilt) {
			buildTilt(aCableWrap);
			flangeZero = AssemblyFrames.translation(0, 0, A_AXIS + A_REACH);
			attach("outputFlange", flange, flangeZero, "aStem");
		} else {
			flangeZero = AssemblyFrames.translation(0, 0, C_OUTPUT + flange.thickness);
			attach("outputFlange", flange, flangeZero, "cShaft");
		}
		exposeConnector("toolFlange", "outputFlange", "face");
	}

	function buildTilt(cableWrap:Float):Void {
		attach("forkBase", new GantryPlate("rotary fork base", 252, 180, 8),
			AssemblyFrames.translation(0, 0, C_OUTPUT), "cShaft");
		var axisFrame = orient(0, 0, A_AXIS - C_OUTPUT - 8, [0.0, 0, 1], [1.0, 0, 0]);
		var bearing = DeepGrooveBearing.metric("6000");
		for (side in [-1, 1]) attach(side < 0 ? "forkLeft" : "forkRight",
			new GantryBoredBracket("rotary fork arm", 12, 180, 149, axisFrame,
				side < 0 ? 10.5 : bearing.outside),
			AssemblyFrames.translation(side * 120, 0, C_OUTPUT + 8), "forkBase");
		attach("aMotor", ServoMotor.model("GENERIC-SERVO-50W"),
			orient(-126, 0, A_AXIS, [0.0, 0, 1], [1.0, 0, 0]), "forkLeft");
		attach("aGearbox", new Gearbox(130, 0.85, 60, 40, 10, true),
			orient(-114, 0, A_AXIS, [0.0, 0, 1], [1.0, 0, 0]), "forkLeft");
		attach("aBearing", bearing, orient(114, 0, A_AXIS, [0.0, 0, 1], [1.0, 0, 0]), "forkRight");
		// The outer flange rim reaches a stop bar before the fork's lower bridge.
		var radius = flange.flangeDiameter / 2;
		var hardStop = Math.acos((A_STOP_TOP - A_AXIS) / Math.sqrt(A_REACH * A_REACH + radius * radius)) -
			Math.atan2(radius, A_REACH);
		for (side in [-1, 1]) attach(side < 0 ? "aStopNegative" : "aStopPositive",
			new GantryPlate("A hard stop", 252, 40, 10),
			AssemblyFrames.translation(0, side * 70, A_STOP_TOP - 10), "forkBase");
		turn("a", "aGearbox", "aShaft", new GantryIdlerPin(10, 208),
			orient(-74, 0, A_AXIS, [0.0, 0, 1], [1.0, 0, 0]), {x: 1, y: 0, z: 0},
			cableWrap, hardStop, "aMotor", "aGearbox");
		attach("aStem", new GantryBoredBracket("A flange stem", 40, 20, A_REACH - flange.thickness + 6,
			orient(0, 0, 6, [0.0, 0, 1], [1.0, 0, 0]), 10.2),
			AssemblyFrames.translation(0, 0, A_AXIS - 6), "aShaft");
	}

	function turn(id:String, parent:String, shaft:String, part:machinekit.component.MachineComponent,
			pose:AssemblyFrame, axis:{x:Float, y:Float, z:Float}, cableWrap:Float, hardStop:Null<Float>,
			motor:String, gearbox:String):Void {
		var bound = hardStop == null ? cableWrap : Math.min(cableWrap, hardStop);
		hang(shaft, part, pose, parent);
		addMateOnAxis(id, "revolute", parent, 'to-$shaft', shaft, 'attach-$shaft', axis, 0,
			{lower: -bound, upper: bound, velocity: null, effort: null,
				assumptions: [{quantity: "position limit", label: "stated rotary head cable allowance"}]});
		rotaryJoints.push({id: id, motor: motor, gearbox: gearbox, lower: -bound, upper: bound,
			cableWrap: cableWrap, hardStop: hardStop});
	}

	/** Attach the head's motors to cabinet drivers already wired in the carrying assembly. */
	public function bindDrives(machine:MachineAssembly, prefix:String, drivers:Array<String>):Void {
		if (drivers.length != rotaryJoints.length) throw "Each rotary head motor needs a cabinet driver";
		function path(id:String):String return prefix.length == 0 ? id : prefix + "/" + id;
		for (index in 0...rotaryJoints.length) {
			var joint = rotaryJoints[index];
			machine.addMotor(path(joint.id + "Drive"), path(joint.id), path(joint.motor), drivers[index], 0.5, path(joint.gearbox));
		}
	}
}

class CHead extends RotaryHead {
	public function new(cCableWrap:Float = Math.PI) super(false, cCableWrap);
}

class CaHead extends RotaryHead {
	public function new(cCableWrap:Float = Math.PI, aCableWrap:Float = Math.PI / 2) super(true, cCableWrap, aCableWrap);
}
