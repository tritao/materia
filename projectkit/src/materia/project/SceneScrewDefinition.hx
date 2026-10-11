package materia.project;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinition;
import materia.project.SceneArtifact.SceneArtifactData;

/** Explicit joint qualification: rad/s, radians from snug, Nm and mm. */
@:wire typedef TighteningSettings = {
	@:id(1) var runDownSpeed:Float;
	@:id(2) var finalSpeed:Float;
	@:id(3) var targetTorque:Float;
	@:id(4) var minimumAngle:Float;
	@:id(5) var maximumAngle:Float;
	@:id(6) var runDownTorqueLimit:Float;
	@:id(7) var finalTorqueLimit:Float;
	@:id(8) var maximumRunDownAngle:Float;
	@:id(9) var minimumStroke:Float;
	@:id(10) var maximumStroke:Float;
}

/** Qualified simulation response, independent of the published torque table. */
@:wire typedef ScrewSimulationSettings = {
	@:id(1) var leadInMm:Float;
	@:id(2) var maximumAxisAngle:Float;
	@:id(3) var prevailingTorque:Float;
	@:id(4) var stiffnessNmPerRad:Float;
	@:id(5) var stripTorque:Float;
}

/** A physically distinct supply for one replacement attempt; lengths use scene metres. */
@:wire typedef ScrewSupply = {
	@:id(1) var screw:String;
	@:id(2) var presenter:String;
	@:id(3) var presentSensor:String;
	@:id(4) var pickup:AssemblyFrame;
}

/** Reject pose follows its declared fixture, and a replacement is never another guessed nearby body. */
@:wire typedef ScrewRecovery = {
	@:id(1) var replacement:ScrewSupply;
	@:id(2) var rejectMember:String;
	@:id(3) var reject:AssemblyFrame;
}

/** A freshly derived CAD fastening. All frames use metres; named occurrences own their live poses. */
@:wire typedef SceneScrew = {
	@:id(1) var fastening:String;
	@:id(2) var screw:String;
	@:id(3) var frame:String;
	@:id(4) var threadFace:String;
	@:id(5) var presenter:String;
	@:id(6) var presentSensor:String;
	/** Socket bottom in the screw's CAD frame, with +Z into its shank. */
	@:id(7) var socket:AssemblyFrame;
	/** Pickup socket in the presenter's frame. */
	@:id(8) var pickup:AssemblyFrame;
	/** Starting socket, final socket and thread lead-in in the threaded member's frame. */
	@:id(9) var entry:AssemblyFrame;
	@:id(10) var seat:AssemblyFrame;
	@:id(11) var hole:AssemblyFrame;
	@:id(12) var leadMm:Float;
	/** +1 for right hand, -1 for left hand. */
	@:id(13) var hand:Int;
	@:id(14) var slot:Int;
	@:id(15) var program:TighteningSettings;
	@:id(16) var simulation:ScrewSimulationSettings;
	@:id(17) var approach:Float;
	@:id(18) @:optional var recovery:ScrewRecovery;
}

/** Strict saved-data boundary, shared with the live process program. No CAD or process dependency. */
class SceneScrewDefinition {
	public static function validateProgram(settings:TighteningSettings):Void {
		if (settings == null) throw "Tightening program needs qualified settings";
		for (value in [settings.runDownSpeed, settings.finalSpeed, settings.targetTorque,
			settings.maximumAngle, settings.runDownTorqueLimit, settings.finalTorqueLimit,
			settings.maximumRunDownAngle, settings.maximumStroke])
			if (!Math.isFinite(value) || value <= 0) throw "Tightening program needs finite positive settings";
		if (!Math.isFinite(settings.minimumAngle) || settings.minimumAngle < 0 ||
			!Math.isFinite(settings.minimumStroke) || settings.minimumStroke < 0)
			throw "Tightening minima must be finite and nonnegative";
		if (settings.minimumAngle >= settings.maximumAngle || settings.minimumStroke > settings.maximumStroke ||
			settings.runDownTorqueLimit >= settings.targetTorque || settings.finalTorqueLimit < settings.targetTorque)
			throw "Tightening program has inconsistent windows or torque limits";
	}
	public static function copyProgram(settings:TighteningSettings):TighteningSettings {
		validateProgram(settings);
		return {runDownSpeed:settings.runDownSpeed, finalSpeed:settings.finalSpeed, targetTorque:settings.targetTorque,
			minimumAngle:settings.minimumAngle, maximumAngle:settings.maximumAngle, runDownTorqueLimit:settings.runDownTorqueLimit,
			finalTorqueLimit:settings.finalTorqueLimit, maximumRunDownAngle:settings.maximumRunDownAngle,
			minimumStroke:settings.minimumStroke, maximumStroke:settings.maximumStroke};
	}
	static function named(value:String):Bool return value != null && value.length > 0 && value.length <= 4096;
	static function validFrame(frame:AssemblyFrame):Bool {
		if (frame == null) return false;
		for (value in [frame.x,frame.y,frame.z,frame.qx,frame.qy,frame.qz,frame.qw]) if (!Math.isFinite(value)) return false;
		return Math.abs(frame.qx*frame.qx+frame.qy*frame.qy+frame.qz*frame.qz+frame.qw*frame.qw-1) <= 1e-8;
	}
	public static function runDownMm(screw:SceneScrew):Float {
		var direction = AssemblyFrames.transformVector(screw.hole,0,0,1);
		return ((screw.seat.x-screw.entry.x)*direction.x+(screw.seat.y-screw.entry.y)*direction.y+
			(screw.seat.z-screw.entry.z)*direction.z)*1000;
	}
	public static function validate(screw:SceneScrew, occurrences:Array<String>):Void {
		function fail(detail:String):Void throw 'Scene screw "${screw == null ? "missing" : screw.fastening}": $detail';
		if (screw == null) { fail("definition is missing"); return; }
		if (!named(screw.fastening) || !named(screw.threadFace)) fail("fastening and topological face names are required");
		for (member in [screw.screw,screw.frame,screw.presenter])
			if (!named(member) || occurrences.indexOf(member) < 0) fail('occurrence "$member" does not exist');
		if (screw.screw == screw.frame || screw.screw == screw.presenter) fail("the screw must have its own occurrence");
		if (!named(screw.presentSensor)) fail("presenter sensor is missing");
		for (frame in [screw.socket,screw.pickup,screw.entry,screw.seat,screw.hole])
			if (!validFrame(frame)) fail("frame must have finite coordinates and a unit rotation");
		if (screw.slot <= 0 || (screw.hand != 1 && screw.hand != -1)) fail("program slot or thread hand is invalid");
		if (!Math.isFinite(screw.leadMm) || screw.leadMm <= 0 || !Math.isFinite(screw.approach) || screw.approach <= 0 || screw.approach > 1)
			fail("lead and approach must be positive and finite");
		validateProgram(screw.program);
		if (screw.program.minimumStroke >= screw.program.maximumStroke) fail("feed window has no room to follow");
		var response = screw.simulation;
		if (response == null) { fail("qualified simulation response is missing"); return; }
		for (value in [response.leadInMm,response.maximumAxisAngle,response.stiffnessNmPerRad,response.stripTorque])
			if (!Math.isFinite(value) || value <= 0) fail("simulation response must be positive and finite");
		if (!Math.isFinite(response.prevailingTorque) || response.prevailingTorque < 0) fail("prevailing torque is invalid");
		var axis = AssemblyFrames.transformVector(screw.hole,0,0,1);
		for (frame in [screw.entry,screw.seat]) {
			var direction = AssemblyFrames.transformVector(frame,0,0,1);
			if (axis.x*direction.x+axis.y*direction.y+axis.z*direction.z < 1-1e-8) fail("socket axis does not point into the named thread");
			var dx = frame.x-screw.hole.x, dy = frame.y-screw.hole.y, dz = frame.z-screw.hole.z;
			var along = dx*axis.x+dy*axis.y+dz*axis.z;
			if (dx*dx+dy*dy+dz*dz-along*along > 1e-16) fail("socket is not on the named thread axis");
		}
		if (Math.abs(screw.entry.qx*screw.seat.qx+screw.entry.qy*screw.seat.qy+screw.entry.qz*screw.seat.qz+screw.entry.qw*screw.seat.qw) < 1-1e-8)
			fail("entry and seat have different socket rotations");
		if (runDownMm(screw) < -1e-8) fail("seat lies behind the thread entry");
		var recovery = screw.recovery;
		if (recovery != null) {
			var supply = recovery.replacement;
			if (supply == null) {fail("replacement supply is missing"); return;}
			for (member in [supply.screw,supply.presenter,recovery.rejectMember])
				if (!named(member) || occurrences.indexOf(member) < 0) fail('recovery occurrence "$member" does not exist');
			if (supply.screw == screw.screw || supply.screw == screw.frame || supply.screw == supply.presenter || supply.screw == recovery.rejectMember)
				fail("replacement must be a distinct screw occurrence");
			if (!named(supply.presentSensor) || !validFrame(supply.pickup) || !validFrame(recovery.reject))
				fail("replacement sensor and recovery frames must be valid");
		}

	}
	/** A replacement changes only supply identity and pickup; the qualified connection has one owner. */
	public static function replacement(job:SceneScrew):SceneScrew {
		var recovery = job.recovery;
		if (recovery == null) throw 'Scene screw "${job.fastening}" has no declared replacement';
		var supply = recovery.replacement;
		return {fastening:job.fastening,screw:supply.screw,frame:job.frame,threadFace:job.threadFace,
			presenter:supply.presenter,presentSensor:supply.presentSensor,pickup:supply.pickup,socket:job.socket,
			entry:job.entry,seat:job.seat,hole:job.hole,leadMm:job.leadMm,hand:job.hand,slot:job.slot,
			program:job.program,simulation:job.simulation,approach:job.approach};
	}

	/** Validate identity against the actual named thread packet, never another equal bore. */
	public static function validateScene(screw:SceneScrew, flat:AssemblyDefinition, data:SceneArtifactData):Void {
		validate(screw,[for (item in flat.occurrences) item.id]);
		var members = [for (item in flat.occurrences) if (item.id == screw.frame) item];
		var found = false;
		for (part in data.parts) if (part.id == members[0].definition) {
			var threads = part.threads;
			if (threads != null) for (thread in threads) if (thread.faceName == screw.threadFace) {
				if (!thread.spec.internal || Math.abs(thread.spec.pitch*thread.spec.starts-screw.leadMm) > 1e-8 ||
					(switch thread.spec.hand { case RightHand: 1; case LeftHand: -1; }) != screw.hand)
					throw 'Scene screw "${screw.fastening}": named thread disagrees with lead or hand';
				var z = AssemblyFrames.transformVector(screw.hole,0,0,1);
				var x = AssemblyFrames.transformVector(screw.hole,1,0,0);
				var origin = [screw.hole.x,screw.hole.y,screw.hole.z];
				var direction = [z.x,z.y,z.z], across = [x.x,x.y,x.z];
				for (i in 0...3) if (Math.abs(origin[i]-thread.origin[i]*data.metresPerUnit) > 1e-8 ||
					Math.abs(direction[i]-thread.direction[i]) > 1e-8 || Math.abs(across[i]-thread.xDirection[i]) > 1e-8)
					throw 'Scene screw "${screw.fastening}": hole frame disagrees with named face "${screw.threadFace}"';
				if (found) throw 'Scene screw "${screw.fastening}": named thread is ambiguous';
				found = true;
			}
		}
		if (!found) throw 'Scene screw "${screw.fastening}": face "${screw.frame}/${screw.threadFace}" has no internal thread';
		var sensors = flat.sensors;
		var present = sensors == null ? [] : [for (sensor in sensors) if (sensor.id == screw.presentSensor &&
			sensor.kind == "presence" && sensor.occurrence == screw.presenter) sensor];
		if (present.length != 1) throw 'Scene screw "${screw.fastening}": presenter sensor "${screw.presentSensor}" is missing or ambiguous';
		var recovery = screw.recovery;
		if (recovery != null) {
			var supply = recovery.replacement;
			var original = [for (item in flat.occurrences) if (item.id == screw.screw) item];
			var spare = [for (item in flat.occurrences) if (item.id == supply.screw) item];
			if (original.length != 1 || spare.length != 1 || original[0].definition != spare[0].definition)
				throw 'Scene screw "${screw.fastening}": replacement does not share the exact screw component definition';
			var supplied = sensors == null ? [] : [for (sensor in sensors) if (sensor.id == supply.presentSensor && sensor.kind == "presence" && sensor.occurrence == supply.presenter) sensor];
			if (supplied.length != 1) throw 'Scene screw "${screw.fastening}": replacement presenter sensor is missing or ambiguous';
		}

	}
	static function number(raw:Dynamic, name:String):Float {
		var value:Dynamic = Reflect.field(raw,name);
		if (!Std.isOfType(value,Float) && !Std.isOfType(value,Int)) throw 'Scene screw field "$name" must be numeric';
		return cast value;
	}
	static function integer(raw:Dynamic, name:String):Int {
		var value = number(raw,name);
		if (!Math.isFinite(value) || value < -2147483647 || value > 2147483647 || Math.floor(value) != value)
			throw 'Scene screw field "$name" must be an integer';
		return Std.int(value);
	}
	static function text(raw:Dynamic, name:String):String {
		var value:Dynamic = Reflect.field(raw,name);
		if (!Std.isOfType(value,String)) throw 'Scene screw field "$name" must be text';
		return cast value;
	}
	static function frame(raw:Dynamic, name:String):AssemblyFrame {
		var value:Dynamic = Reflect.field(raw,name);
		return {x:number(value,"x"),y:number(value,"y"),z:number(value,"z"),qx:number(value,"qx"),
			qy:number(value,"qy"),qz:number(value,"qz"),qw:number(value,"qw")};
	}
	public static function decode(raw:Dynamic):SceneScrew {
		var program:Dynamic = Reflect.field(raw,"program"), response:Dynamic = Reflect.field(raw,"simulation");
		var recovering:Dynamic = Reflect.field(raw,"recovery");
		var recovery:Null<ScrewRecovery> = null;
		if (recovering != null) {
			var supply:Dynamic = Reflect.field(recovering,"replacement");
			recovery = {replacement:{screw:text(supply,"screw"),presenter:text(supply,"presenter"),presentSensor:text(supply,"presentSensor"),pickup:frame(supply,"pickup")},
				rejectMember:text(recovering,"rejectMember"),reject:frame(recovering,"reject")};
		}
		return {fastening:text(raw,"fastening"),screw:text(raw,"screw"),frame:text(raw,"frame"),threadFace:text(raw,"threadFace"),
			presenter:text(raw,"presenter"),presentSensor:text(raw,"presentSensor"),socket:frame(raw,"socket"),pickup:frame(raw,"pickup"),
			entry:frame(raw,"entry"),seat:frame(raw,"seat"),hole:frame(raw,"hole"),leadMm:number(raw,"leadMm"),
			hand:integer(raw,"hand"),slot:integer(raw,"slot"),approach:number(raw,"approach"),recovery:recovery,
			program:{runDownSpeed:number(program,"runDownSpeed"),finalSpeed:number(program,"finalSpeed"),targetTorque:number(program,"targetTorque"),
				minimumAngle:number(program,"minimumAngle"),maximumAngle:number(program,"maximumAngle"),runDownTorqueLimit:number(program,"runDownTorqueLimit"),
				finalTorqueLimit:number(program,"finalTorqueLimit"),maximumRunDownAngle:number(program,"maximumRunDownAngle"),
				minimumStroke:number(program,"minimumStroke"),maximumStroke:number(program,"maximumStroke")},
			simulation:{leadInMm:number(response,"leadInMm"),maximumAxisAngle:number(response,"maximumAxisAngle"),prevailingTorque:number(response,"prevailingTorque"),
				stiffnessNmPerRad:number(response,"stiffnessNmPerRad"),stripTorque:number(response,"stripTorque")}};
	}
}
