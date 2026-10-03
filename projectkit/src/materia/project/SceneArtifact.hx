package materia.project;

import haxe.io.Bytes;
import materia.assembly.AssemblyCodec;
import materia.assembly.AssemblyRecord;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.project.Appearance;
import materia.project.Appearance.Appearances;
import materia.project.MaterialLibrary;
import materia.project.MeshMassProperties;
import materia.units.LengthUnit;

typedef SceneArtifactFaceRange = {
	var faceIndex:Int;
	var firstIndex:Int;
	var indexCount:Int;
}

typedef SceneArtifactPart = {
	var id:String;
	var name:String;
	var red:Float;
	var green:Float;
	var blue:Float;
	@:optional var appearance:Appearance;
	@:optional var materialId:String;
	@:optional var materialDensity:Float;
	@:optional var materialSpec:String;
	@:optional var volume:Float;
	@:optional var centerOfMass:Array<Float>;
	@:optional var inertia:Array<Float>;
	var vertexCount:Int;
	var indexCount:Int;
	var vertices:Bytes;
	var normals:Bytes;
	var indices:Bytes;
	@:optional var edgeSegments:Bytes;
	@:optional var edgeIds:Bytes;
	var faceRanges:Array<SceneArtifactFaceRange>;
	/**
		What each B-rep face offers an assembly mate, from the producer's geometry kernel (CadKit's
		`GeometricConnectors.describeFaces`): opaque here, indexed like `faceRanges.faceIndex`. Lets an editor
		without the B-rep author mates on faces (since version 11).
	*/
	@:optional var faceDescriptors:String;
}

typedef SceneArtifactData = {
	@:optional var lengthUnit:String;
	/** Metres represented by one coordinate in every mesh stream. */
	var metresPerUnit:Float;
	var parts:Array<SceneArtifactPart>;
	@:optional var assembly:AssemblyRecord;
	/** Geometry definitions are parts; occurrences live in this separate kinematic definition. */
	@:optional var assemblyDefinition:AssemblyDefinition;
	@:optional var assemblyState:AssemblyStateRecord;
	/** Optional editable source document for generated parts. */
	@:optional var recipeDocument:String;
	@:optional var recipeDiagnostics:Array<String>;
	/** A machining job the generator made for its own machine; see SceneArtifactMachining. */
	@:optional var machining:SceneArtifactMachining;
	/** The assembly is a wheeled robot that drives on the floor; see SceneArtifactMobileBase. */
	@:optional var mobileBase:SceneArtifactMobileBase;
	/** Work the assembly's robot does on its own; see SceneArtifactMission. */
	@:optional var mission:SceneArtifactMission;
	/** The tools the assembly's robot works with; see SceneArtifactRobotTool. */
	@:optional var robotTools:Array<SceneArtifactRobotTool>;
	/** The sensors on the assembly's robot; see SceneArtifactRobotSensor. */
	@:optional var robotSensors:Array<SceneArtifactRobotSensor>;
}

/**
 * A sensor on the assembly's robot (since version 14), as its part declares it. A `lidar` scans the
 * horizontal plane of the connector `mount.connector` of the occurrence `mount.occurrence`, `rayCount`
 * rays round the full circle from the connector's +X axis, seeing out to `maxRange` metres and scanning
 * `updateRate` times a second. `id` names it in the robot's observations.
 */
typedef SceneArtifactRobotSensor = {
	var kind:String;
	var id:String;
	var mount:SceneArtifactPlace;
	var rayCount:Int;
	var maxRange:Float;
	var updateRate:Float;
}

/**
 * A tool on the assembly's robot and how it is worked (since version 13), as its parts' declared
 * capabilities say: a `suction` tool seals on what touches the occurrence `contact.occurrence` at its
 * connector `contact.connector` while the digital process channel `channel` is on, and reports the
 * vacuum it pulls on the sensor `sensor` when it has one.
 *
 * A `torch` tool is a MIG/MAG welding torch: `contact` is the connector at its wire tip (the tool point),
 * `channel` is the digital channel that lights its arc, `sensor` is its `tool_weld` sensor, and `torch`
 * describes the welder behind it. The kind is part of the same JSON section, so it needs no format version.
 */
typedef SceneArtifactRobotTool = {
	var kind:String;
	var contact:SceneArtifactPlace;
	var channel:String;
	@:optional var sensor:String;
	@:optional var torch:SceneArtifactTorch;
}

/**
 * The welder a `torch` robot tool runs on. `wireSpeedChannel` (metres per minute) and `voltageChannel` (volts)
 * are its analogue setpoints. The weld circuit closes through the occurrences in `groundedWork` (the work
 * clamp's member and what is welded to it); the arc strikes only against them. `maxCurrentA` is the supply's
 * rating, `efficiency` the fraction of the mains power it delivers to the arc, `wireDiameterMm` the wire's
 * diameter and `stickoutMm` the wire's extension past the contact tip. `maxWireSpeedMPerMin` is the feeder's top
 * wire speed and `depositionEfficiency` the fraction of the melted wire that reaches the weld; the weld steps' recipes
 * were made for this wire, and the simulated weld deposits with the same figure.
 */
typedef SceneArtifactTorch = {
	var wireSpeedChannel:String;
	var voltageChannel:String;
	var groundedWork:Array<String>;
	var maxCurrentA:Float;
	var efficiency:Float;
	var wireDiameterMm:Float;
	var stickoutMm:Float;
	var maxWireSpeedMPerMin:Float;
	var depositionEfficiency:Float;
}

/**
 * The assembly is a differential-drive robot that rolls on the floor (since version 13). Its root body
 * is the chassis, driven over the floor plane in the assembly frame: +X forward, +Z up, the origin on
 * the floor between the wheels' contacts. `leftWheel` and `rightWheel` are its wheel joints (+Y side
 * first), each turning about its own axle; the simulation reads each one's rolling direction from
 * its axis. Lengths are metres, speeds m/s and rad/s, accelerations m/s² and rad/s², whatever the
 * scene's unit. `footprintLength`/`footprintWidth` are the chassis outline, centred on the origin.
 *
 * A robot that works in a scene of its own names its occurrence subtree as `robot` (an include id:
 * its occurrences are `robot/...`) and stands at `origin` (x, y and heading on the floor, in the
 * assembly frame); everything outside the subtree is its world, held where it stands. Without
 * `robot` the whole assembly is the robot, standing at the assembly origin.
 */
typedef SceneArtifactMobileBase = {
	var leftWheel:String;
	var rightWheel:String;
	var wheelRadius:Float;
	var trackWidth:Float;
	var maxLinearSpeed:Float;
	var maxAngularSpeed:Float;
	var maxLinearAcceleration:Float;
	var maxAngularAcceleration:Float;
	@:optional var footprintLength:Float;
	@:optional var footprintWidth:Float;
	@:optional var robot:String;
	@:optional var origin:SceneArtifactFloorPose;
}

/**
 * Work the assembly's robot does on its own when the simulation runs (since version 13): its
 * steps in order, starting over after the last when `loop`. Steps name what they act on in the
 * assembly, so where they lead follows the model. Picking and placing work the robot's one suction
 * tool (see SceneArtifactRobotTool).
 */
typedef SceneArtifactMission = {
	var steps:Array<SceneArtifactMissionStep>;
	@:optional var loop:Bool;
}

/**
 * One step of a mission.
 * - `goTo`: drive the mobile base to `pose`, in the assembly frame on the floor.
 * - `pick`: take hold of the occurrence `at.occurrence` with the tool, meeting it at its connector
 *   `at.connector`, wherever the part is when the step starts.
 * - `place`: set what the tool holds down with its base on the connector `at.connector` of
 *   `at.occurrence`, and let go.
 * - `weld`: weld the seam `weld` describes with the robot's torch.
 */
typedef SceneArtifactMissionStep = {
	var kind:String;
	@:optional var pose:SceneArtifactFloorPose;
	@:optional var at:SceneArtifactPlace;
	@:optional var weld:SceneArtifactWeld;
}

/**
 * A weld to run and how. What is welded is a result of the CAD, never authored: `path` is the seams as the CAD
 * found them, run one after the other as one weld (a straight seam is a path of one segment, the four sides of a
 * tube a path of four), and the rest is what the generator derived from them, so a player needs no CAD to weld
 * them. Lengths are metres, whatever the scene's unit.
 * - `path` is the segments in order, each starting where the one before ends. The torch strikes the arc at the
 *   start of the first, keeps it up along the whole path and ends it at the end of the last. Where one segment's
 *   torch angles differ from the next one's, as at the corner of a tube, the torch turns from one to the other
 *   as it passes the joint. The joints of a path may differ, but the process is one, so they ask for one leg.
 * - `frame` is the occurrence the path is relative to, the workpiece's reference member: every pose and normal in the
 *   path is in that occurrence's own frame (metres, with its normals turned with it). A player finds the occurrence where it
 *   stands when the step starts and places the path by it, so the weld follows a workpiece that is not where it was
 *   designed. Without a `frame` the path is in the assembly as designed (the form welds took before it existed).
 * - `metal` is the occurrence that carries the weld metal: the part the bead is shown on as the welder lays it.
 * - `legSize` is the leg the weldment asks for.
 * - `process` is how the weld is run (see SceneArtifactWeldProcess); the generator derived it from the leg,
 *   so the leg the weld reaches is a result that the declared one is checked against.
 *
 * A step written before paths existed has `seam`, `joint`, `start`, `stop` and `normals` on the weld itself, for
 * one straight seam; reading it makes that the single segment of a path.
 */
typedef SceneArtifactWeld = {
	@:optional var frame:String;
	var metal:String;
	var path:Array<SceneArtifactWeldSegment>;
	var legSize:Float;
	var process:SceneArtifactWeldProcess;
}

/**
 * One primitive of a weld path, the piece of a seam the CAD found. `kind` says what shape it is: only a `line` so far,
 * straight from `start` to `stop` with the torch holding its orientation; an `arc` for a curved seam would add its
 * centre and sweep, and be another kind.
 * - `seam` is its name as the CAD found it (`member:face|member:face`, the two faces the weld metal fills against,
 *   each in the geometry of the occurrence `member`).
 * - `joint` is how the members meet; only a `fillet` is deposited so far.
 * - `start` and `stop` are the poses of the wire tip at the two ends: +Z along the wire out of the torch, +X along
 *   the direction of travel as far as it is square to the wire. They carry the work and travel angles.
 * - `normals` are the two faces' outward unit normals, which say where the weld metal sits (the fillet
 *   fills the corner they leave open).
 */
typedef SceneArtifactWeldSegment = {
	var kind:String;
	var seam:String;
	var joint:String;
	var start:SceneArtifactTorchPose;
	var stop:SceneArtifactTorchPose;
	var normals:Array<Array<Float>>;
}

/** A pose in the assembly frame: position in metres and an xyzw rotation. */
typedef SceneArtifactTorchPose = {
	var position:Array<Float>;
	var rotation:Array<Float>;
}

/**
 * How a seam is welded: the wire speed (metres per minute) and voltage (volts) of the arc, the travel
 * speed along the seam (metres per second), and the sequence around it. The torch approaches from `approach`
 * metres out along the wire, strikes the arc at the start and holds until it is established, dwells
 * `startDwell` seconds, travels the seam, dwells `craterDwell` seconds at its end to fill the crater, stops the
 * wire, lets the arc burn back for `burnback` seconds, and retracts.
 */
typedef SceneArtifactWeldProcess = {
	var wireSpeed:Float;
	var voltage:Float;
	var travelSpeed:Float;
	var approach:Float;
	var startDwell:Float;
	var craterDwell:Float;
	var burnback:Float;
}

/** A connector of an occurrence in the assembly. */
typedef SceneArtifactPlace = {
	var occurrence:String;
	var connector:String;
}

/** A pose on the floor: x and y in metres and the heading `yaw` in radians about +Z. */
typedef SceneArtifactFloorPose = {
	var x:Float;
	var y:Float;
	var yaw:Float;
}

/**
 * A machining job the generator made for its own machine, with everything needed to run it (since
 * version 12). Lengths are metres, whatever the scene's unit.
 *
 * The machine's `axes` are three prismatic joints whose positions are machine X, Y and Z, and machine
 * coordinates name where the spindle's gauge line is: the origin of the `spindle` occurrence, whose
 * +Z runs up the spindle axis. `workOffset` is G54 in machine coordinates. Each tool in `tools` hangs
 * `length` below the gauge line and has the shape `profile` (a toolpath cutter profile's `encode`),
 * so the program's G43 H numbers are the tool numbers. `stock` is the occurrence the tool cuts and
 * `sacrificial` those it may run into without harm, such as a spoilboard under through holes.
 * `toolPart` is the occurrence that shows the tool in the spindle, whose shape changes with the tool,
 * and `loadedTool` the number of the tool in the spindle when the job starts.
 * `target` names a part of the artifact that no occurrence uses: the finished part, in the stock's
 * frame, to compare the machined stock with. A looping job starts again when the program ends.
 * `controller` is the controller the machine's steppers are nominally wired to: the machine's own
 * limits are the motors' and drives', and the controller's step rate caps them further.
 */
typedef SceneArtifactMachining = {
	var program:String;
	var axes:Array<String>;
	var spindle:String;
	var workOffset:Array<Float>;
	var tools:Array<SceneArtifactTool>;
	@:optional var stock:String;
	@:optional var sacrificial:Array<String>;
	@:optional var toolPart:String;
	@:optional var loadedTool:Int;
	@:optional var target:String;
	@:optional var loop:Bool;
	@:optional var controller:SceneArtifactController;
}

/**
 * The nominal wiring of a machine's stepper drivers: every stepper driven at `microsteps` per full
 * step from a controller that generates at most `stepTickHz` step edges a second per channel. The
 * device binding turns them into each axis's step-rate ceiling.
 */
typedef SceneArtifactController = {
	var microsteps:Int;
	var stepTickHz:Int;
}

/** One tool of a machining job's tool table; see SceneArtifactMachining. */
typedef SceneArtifactTool = {
	var number:Int;
	var length:Float;
	var profile:Array<Array<Float>>;
}

/** Versioned, producer-independent scene geometry exchange format. */
class SceneArtifact {
	public static inline var VERSION:Int = 14;
	public static inline var MAX_BYTES:Int = 150000000;
	static inline var MAX_VERTICES:Int = 2000000;
	static inline var MAX_TRIANGLES:Int = 4000000;
	static inline var MAX_FACE_RANGES:Int = 100000;

	public static function encode(data:SceneArtifactData):Bytes {
		validateHeader(data);
		var unit = data.lengthUnit == null ? LengthUnit.fromScale(data.metresPerUnit) : data.lengthUnit;
		var unitText = Bytes.ofString(unit);
		var names:Array<{id:Bytes, name:Bytes, finish:Bytes, materialId:Bytes, materialSpec:Bytes,
			density:Float, volume:Float, center:Array<Float>, inertia:Array<Float>, faces:Bytes}> = [];
		var assembly = data.assembly == null ? Bytes.alloc(0) : Bytes.ofString(AssemblyCodec.encode(data.assembly));
		var assemblyDefinition = data.assemblyDefinition == null ? Bytes.alloc(0)
			: Bytes.ofString(AssemblyDefinitionCodec.encode(data.assemblyDefinition));
		var assemblyState = data.assemblyState == null ? Bytes.alloc(0)
			: Bytes.ofString(AssemblyDefinitionCodec.encodeState(data.assemblyDefinition, data.assemblyState));
		var recipeDocument = data.recipeDocument == null ? Bytes.alloc(0) : Bytes.ofString(data.recipeDocument);
		var recipeDiagnostics = data.recipeDiagnostics == null ? Bytes.alloc(0)
			: Bytes.ofString(haxe.Json.stringify(data.recipeDiagnostics));
		if (assembly.length > 2000000 || assemblyDefinition.length > 2000000 || assemblyState.length > 2000000)
			throw "Scene artifact assembly metadata is too large";
		if (recipeDocument.length > 2000000) throw "Scene artifact recipe document is too large";
		if (recipeDiagnostics.length > 2000000) throw "Scene artifact recipe diagnostics are too large";
		var machining = data.machining == null ? Bytes.alloc(0) : Bytes.ofString(haxe.Json.stringify(data.machining));
		if (machining.length > 8000000) throw "Scene artifact machining job is too large";
		var mobileBase = data.mobileBase == null ? Bytes.alloc(0) : Bytes.ofString(haxe.Json.stringify(data.mobileBase));
		var mission = data.mission == null ? Bytes.alloc(0) : Bytes.ofString(haxe.Json.stringify(data.mission));
		if (mission.length > 1000000) throw "Scene artifact mission is too large";
		var robotTools = data.robotTools == null ? Bytes.alloc(0) : Bytes.ofString(haxe.Json.stringify(data.robotTools));
		if (robotTools.length > 100000) throw "Scene artifact robot tools are too large";
		var robotSensors = data.robotSensors == null ? Bytes.alloc(0) : Bytes.ofString(haxe.Json.stringify(data.robotSensors));
		if (robotSensors.length > 100000) throw "Scene artifact robot sensors are too large";
		var length = 40 + unitText.length + assembly.length + assemblyDefinition.length + assemblyState.length + recipeDocument.length + recipeDiagnostics.length + 4
			+ machining.length + 4 + mobileBase.length + 4 + mission.length + 4 + robotTools.length + 4 + robotSensors.length + 4;
		for (part in data.parts) {
			validatePart(part, true);
			var id = Bytes.ofString(part.id), name = Bytes.ofString(part.name);
			if (id.length == 0 || id.length > 4096 || name.length == 0 || name.length > 4096)
				throw "Scene artifact has an invalid part ID or name";
			var finish = Bytes.ofString(part.appearance == null ? "neutral" : part.appearance.finish);
			var idValue = part.materialId == null ? "neutral" : part.materialId;
			var materialId = Bytes.ofString(idValue);
			var spec = part.materialSpec == null ? MaterialLibrary.require(idValue).physical.spec : part.materialSpec;
			var materialSpec = Bytes.ofString(spec);
			var density = part.materialDensity == null ? MaterialLibrary.require(idValue).physical.density : part.materialDensity;
			var meshProperties = MeshMassProperties.compute(part.vertices, part.indices);
			var faceDescriptors = part.faceDescriptors;
			var faces = faceDescriptors == null ? Bytes.alloc(0) : Bytes.ofString(faceDescriptors);
			names.push({id: id, name: name, finish: finish, materialId: materialId,
				materialSpec: materialSpec, density: density, faces: faces,
				volume: part.volume == null ? meshProperties.volume : part.volume,
				center: part.centerOfMass == null ? meshProperties.centerOfMass : part.centerOfMass,
				inertia: part.inertia == null ? meshProperties.inertia : part.inertia});
			length += 168 + finish.length + materialId.length + materialSpec.length + id.length + name.length + part.vertices.length + part.normals.length
				+ part.indices.length + (part.edgeSegments == null ? 0 : part.edgeSegments.length)
				+ (part.edgeIds == null ? 0 : part.edgeIds.length)
				+ part.faceRanges.length * 12 + 4 + faces.length;
			if (length > MAX_BYTES) throw "Scene artifact exceeds the 150 MB limit";
		}
		var result = Bytes.alloc(length), offset = 0;
		for (byte in [77, 84, 82, 71]) result.set(offset++, byte); // MTRG.
		offset = putInt(result, offset, VERSION);
		result.setDouble(offset, data.metresPerUnit); offset += 8;
		offset = putInt(result, offset, unitText.length);
		result.blit(offset, unitText, 0, unitText.length); offset += unitText.length;
		offset = putInt(result, offset, data.parts.length);
		for (index in 0...data.parts.length) {
			var part = data.parts[index], text = names[index];
			offset = putInt(result, offset, text.id.length);
			result.blit(offset, text.id, 0, text.id.length); offset += text.id.length;
			offset = putInt(result, offset, text.name.length);
			result.blit(offset, text.name, 0, text.name.length); offset += text.name.length;
			for (color in [part.red, part.green, part.blue]) {
				result.setFloat(offset, color); offset += 4;
			}
			offset = putInt(result, offset, text.finish.length);
			result.blit(offset, text.finish, 0, text.finish.length); offset += text.finish.length;
			var appearance = part.appearance == null ? Appearances.neutral() : part.appearance;
			result.setFloat(offset, appearance.metallic); offset += 4;
			result.setFloat(offset, appearance.roughness); offset += 4;
			offset = putInt(result, offset, text.materialId.length);
			result.blit(offset, text.materialId, 0, text.materialId.length); offset += text.materialId.length;
			result.setDouble(offset, text.density); offset += 8;
			offset = putInt(result, offset, text.materialSpec.length);
			result.blit(offset, text.materialSpec, 0, text.materialSpec.length); offset += text.materialSpec.length;
			result.setDouble(offset, text.volume); offset += 8;
			for (coordinate in text.center) { result.setDouble(offset, coordinate); offset += 8; }
			for (component in text.inertia) { result.setDouble(offset, component); offset += 8; }
			offset = putInt(result, offset, part.vertexCount);
			offset = putInt(result, offset, part.indexCount);
			offset = putInt(result, offset, part.faceRanges.length);
			offset = putInt(result, offset, part.edgeSegments == null ? 0 : part.edgeSegments.length);
			for (buffer in [part.vertices, part.normals, part.indices]) {
				result.blit(offset, buffer, 0, buffer.length); offset += buffer.length;
			}
			if (part.edgeSegments != null) {
				result.blit(offset, part.edgeSegments, 0, part.edgeSegments.length);
				offset += part.edgeSegments.length;
			}
			if (part.edgeIds != null) {
				result.blit(offset, part.edgeIds, 0, part.edgeIds.length);
				offset += part.edgeIds.length;
			}
			for (range in part.faceRanges) {
				offset = putInt(result, offset, range.faceIndex);
				offset = putInt(result, offset, range.firstIndex);
				offset = putInt(result, offset, range.indexCount);
			}
			offset = putInt(result, offset, text.faces.length);
			result.blit(offset, text.faces, 0, text.faces.length); offset += text.faces.length;
		}
		offset = putInt(result, offset, assembly.length);
		result.blit(offset, assembly, 0, assembly.length); offset += assembly.length;
		offset = putInt(result, offset, assemblyDefinition.length);
		result.blit(offset, assemblyDefinition, 0, assemblyDefinition.length); offset += assemblyDefinition.length;
		offset = putInt(result, offset, assemblyState.length);
		result.blit(offset, assemblyState, 0, assemblyState.length); offset += assemblyState.length;
		offset = putInt(result, offset, recipeDocument.length);
		result.blit(offset, recipeDocument, 0, recipeDocument.length); offset += recipeDocument.length;
		offset = putInt(result, offset, recipeDiagnostics.length);
		result.blit(offset, recipeDiagnostics, 0, recipeDiagnostics.length); offset += recipeDiagnostics.length;
		offset = putInt(result, offset, machining.length);
		result.blit(offset, machining, 0, machining.length); offset += machining.length;
		offset = putInt(result, offset, mobileBase.length);
		result.blit(offset, mobileBase, 0, mobileBase.length); offset += mobileBase.length;
		offset = putInt(result, offset, mission.length);
		result.blit(offset, mission, 0, mission.length); offset += mission.length;
		offset = putInt(result, offset, robotTools.length);
		result.blit(offset, robotTools, 0, robotTools.length); offset += robotTools.length;
		offset = putInt(result, offset, robotSensors.length);
		result.blit(offset, robotSensors, 0, robotSensors.length); offset += robotSensors.length;
		if (offset != result.length) throw "Scene artifact size mismatch";
		return result;
	}

	public static function decode(source:Bytes):SceneArtifactData {
		if (source == null || source.length > MAX_BYTES) throw "Scene artifact is missing or too large";
		return new SceneArtifactReader(source).read();
	}

	static function validateHeader(data:SceneArtifactData):Void {
		if (data == null || !finite(data.metresPerUnit) || data.metresPerUnit <= 0.0 ||
			data.parts == null || data.parts.length == 0 || data.parts.length > 1000)
			throw "Scene artifact has invalid units or part count";
		if (data.lengthUnit != null &&
			Math.abs(LengthUnit.metresPerUnit(data.lengthUnit) - data.metresPerUnit) > 1e-12)
			throw "Scene artifact length unit and scale disagree";
		var ids = new Map<String, Bool>();
		for (part in data.parts) {
			if (part.id == null || StringTools.trim(part.id).length == 0 || ids.exists(part.id))
				throw "Scene artifact has a duplicate or empty part ID";
			ids.set(part.id, true);
		}
		var assembly = data.assembly;
		if (assembly != null) {
			AssemblyCodec.validate(assembly);
			if (data.assemblyDefinition == null && assembly.instances.length != data.parts.length)
				throw "Assembly must place every scene artifact part";
			for (instance in assembly.instances) if (data.assemblyDefinition == null && !ids.exists(instance.id))
				throw 'Assembly instance "${instance.id}" has no geometry part';
		}
		if (data.assemblyState != null && data.assemblyDefinition == null)
			throw "Scene artifact assembly state has no definition";
		var assemblyDefinition = data.assemblyDefinition;
		if (assemblyDefinition != null) {
			AssemblyDefinitionCodec.validate(assemblyDefinition);
			var topLevelCount = assemblyDefinition.occurrences.length;
			assemblyDefinition = AssemblyDefinitionFlattener.flatten(assemblyDefinition);
			if (assembly != null && assembly.instances.length != assemblyDefinition.occurrences.length &&
				assembly.instances.length != topLevelCount)
				throw "Assembly snapshot does not match its occurrences";
			for (definition in assemblyDefinition.definitions) if (!ids.exists(definition.id))
				throw 'Assembly component definition "${definition.id}" has no geometry part';
			if (data.assemblyState != null)
				AssemblyDefinitionCodec.validateState(assemblyDefinition, data.assemblyState);
		}
		if (data.machining != null) validateMachining(data.machining, ids, data.assemblyDefinition);
		if (data.mobileBase != null) validateMobileBase(data.mobileBase, data.assemblyDefinition);
		if (data.robotTools != null) validateRobotTools(data.robotTools, data);
		if (data.robotSensors != null) validateRobotSensors(data.robotSensors, data);
		if (data.mission != null) validateMission(data.mission, data);
	}

	static function validateRobotTools(tools:Array<SceneArtifactRobotTool>, data:SceneArtifactData):Void {
		function fail(detail:String):Void throw 'Scene artifact robot tool $detail';
		var definition = data.assemblyDefinition;
		if (definition == null) fail("needs the robot's assembly definition");
		var flat = AssemblyDefinitionFlattener.flatten(cast definition);
		var channels = new Map<String, Bool>();
		var torches = 0;
		for (tool in tools) {
			if (tool == null || (tool.kind != "suction" && tool.kind != "torch"))
				fail('kind "${tool == null ? null : tool.kind}" is unknown');
			var contact = tool.contact;
			var occurrence = contact == null ? [] : [for (item in flat.occurrences) if (item.id == contact.occurrence) item];
			if (occurrence.length != 1) fail("touches with no occurrence of the assembly");
			var found = false;
			for (component in flat.definitions) if (component.id == occurrence[0].definition)
				for (connector in component.connectors) if (connector.name == contact.connector) found = true;
			if (!found) fail('has no contact connector "${contact.connector}" on "${contact.occurrence}"');
			if (tool.channel == null || tool.channel.length == 0 || channels.exists(tool.channel))
				fail("needs a channel of its own");
			channels.set(tool.channel, true);
			if (tool.sensor != null && tool.sensor.length == 0) fail("has an empty sensor name");
			if (tool.kind == "suction") {
				if (tool.torch != null) fail("is a suction tool with a torch description");
				continue;
			}
			// A torch is worked on three channels and reports on one sensor; the weld circuit closes through its work.
			var welder = tool.torch;
			if (welder == null) {
				fail("is a torch with no welder description");
				return;
			}
			if (tool.sensor == null) fail("is a torch with no weld sensor");
			if (++torches > 1) fail("is a second torch: the robot welds with one");
			for (name in [welder.wireSpeedChannel, welder.voltageChannel]) {
				if (name == null || name.length == 0 || channels.exists(name)) fail("needs wire speed and voltage channels of its own");
				channels.set(name, true);
			}
			if (welder.groundedWork == null || welder.groundedWork.length == 0)
				fail("is a torch with no grounded work: the arc has nothing to return through");
			var grounded = new Map<String, Bool>();
			for (id in welder.groundedWork) {
				if (grounded.exists(id) || [for (item in flat.occurrences) if (item.id == id) item].length != 1)
					fail('names grounded work "$id", which is not a distinct occurrence of the assembly');
				grounded.set(id, true);
			}
			if (!(finite(welder.maxCurrentA) && welder.maxCurrentA > 0 && welder.maxCurrentA <= 2000))
				fail("needs a supply rating between 0 and 2000 A");
			if (!(finite(welder.efficiency) && welder.efficiency > 0 && welder.efficiency <= 1))
				fail("needs a supply efficiency in (0, 1]");
			if (!(finite(welder.wireDiameterMm) && welder.wireDiameterMm >= 0.5 && welder.wireDiameterMm <= 4.0))
				fail("needs a wire between 0.5 and 4 mm");
			if (!(finite(welder.stickoutMm) && welder.stickoutMm > 0 && welder.stickoutMm <= 50))
				fail("needs a stickout between 0 and 50 mm");
			if (!(finite(welder.maxWireSpeedMPerMin) && welder.maxWireSpeedMPerMin >= 1 && welder.maxWireSpeedMPerMin <= 60))
				fail("needs a top wire speed between 1 and 60 m/min");
			if (!(finite(welder.depositionEfficiency) && welder.depositionEfficiency > 0.3 && welder.depositionEfficiency <= 1))
				fail("needs a deposition efficiency between 0.3 and 1");
		}
	}

	static function validateRobotSensors(sensors:Array<SceneArtifactRobotSensor>, data:SceneArtifactData):Void {
		function fail(detail:String):Void throw 'Scene artifact robot sensor $detail';
		var definition = data.assemblyDefinition;
		if (definition == null) fail("needs the robot's assembly definition");
		var flat = AssemblyDefinitionFlattener.flatten(cast definition);
		var ids = new Map<String, Bool>();
		for (sensor in sensors) {
			if (sensor == null || sensor.kind != "lidar") fail('kind "${sensor == null ? null : sensor.kind}" is unknown');
			if (sensor.id == null || sensor.id.length == 0 || ids.exists(sensor.id)) fail("needs an id of its own");
			ids.set(sensor.id, true);
			var mount = sensor.mount;
			var occurrence = mount == null ? [] : [for (item in flat.occurrences) if (item.id == mount.occurrence) item];
			if (occurrence.length != 1) fail('"${sensor.id}" is mounted on no occurrence of the assembly');
			var found = false;
			for (component in flat.definitions) if (component.id == occurrence[0].definition)
				for (connector in component.connectors) if (connector.name == mount.connector) found = true;
			if (!found) fail('"${sensor.id}" has no mount connector "${mount.connector}" on "${mount.occurrence}"');
			if (sensor.rayCount < 2 || sensor.rayCount > 360) fail('"${sensor.id}" needs 2 to 360 rays');
			if (!finite(sensor.maxRange) || sensor.maxRange <= 0 || !finite(sensor.updateRate) || sensor.updateRate <= 0)
				fail('"${sensor.id}" needs a positive range and rate');
		}
	}

	/** Robot sensors from their JSON section, typed field by field. */
	static function decodeRobotSensors(decoded:Dynamic):Array<SceneArtifactRobotSensor> {
		function fail():Dynamic throw "Scene artifact robot sensors are invalid";
		function text(value:Dynamic, name:String):String {
			var item:Dynamic = Reflect.field(value, name);
			return Std.isOfType(item, String) ? item : fail();
		}
		function number(value:Dynamic, name:String):Float {
			var item:Dynamic = Reflect.field(value, name);
			return Std.isOfType(item, Float) || Std.isOfType(item, Int) ? (item:Float) : fail();
		}
		if (!Std.isOfType(decoded, Array)) fail();
		return [for (raw in (cast decoded:Array<Dynamic>)) {
			var mount:Dynamic = Reflect.field(raw, "mount");
			if (mount == null) fail();
			var rays = number(raw, "rayCount");
			if (rays != Math.floor(rays)) fail();
			var sensor:SceneArtifactRobotSensor = {kind: text(raw, "kind"), id: text(raw, "id"),
				mount: {occurrence: text(mount, "occurrence"), connector: text(mount, "connector")},
				rayCount: Std.int(rays), maxRange: number(raw, "maxRange"), updateRate: number(raw, "updateRate")};
			sensor;
		}];
	}

	/** Robot tools from their JSON section, typed field by field. */
	static function decodeRobotTools(decoded:Dynamic):Array<SceneArtifactRobotTool> {
		function fail():Dynamic throw "Scene artifact robot tools are invalid";
		function text(value:Dynamic, name:String):String {
			var item:Dynamic = Reflect.field(value, name);
			return Std.isOfType(item, String) ? item : fail();
		}
		if (!Std.isOfType(decoded, Array)) fail();
		return [for (raw in (cast decoded:Array<Dynamic>)) {
			var contact:Dynamic = Reflect.field(raw, "contact");
			if (contact == null) fail();
			var tool:SceneArtifactRobotTool = {kind: text(raw, "kind"),
				contact: {occurrence: text(contact, "occurrence"), connector: text(contact, "connector")},
				channel: text(raw, "channel")};
			if (Reflect.field(raw, "sensor") != null) tool.sensor = text(raw, "sensor");
			var welder:Dynamic = Reflect.field(raw, "torch");
			if (welder != null) {
				function number(name:String):Float {
					var item:Dynamic = Reflect.field(welder, name);
					return Std.isOfType(item, Float) || Std.isOfType(item, Int) ? (item:Float) : fail();
				}
				var work:Dynamic = Reflect.field(welder, "groundedWork");
				if (!Std.isOfType(work, Array)) fail();
				tool.torch = {wireSpeedChannel: text(welder, "wireSpeedChannel"), voltageChannel: text(welder, "voltageChannel"),
					groundedWork: [for (id in (cast work:Array<Dynamic>)) Std.isOfType(id, String) ? (id:String) : fail()],
					maxCurrentA: number("maxCurrentA"), efficiency: number("efficiency"),
					wireDiameterMm: number("wireDiameterMm"), stickoutMm: number("stickoutMm"),
					maxWireSpeedMPerMin: number("maxWireSpeedMPerMin"), depositionEfficiency: number("depositionEfficiency")};
			}
			tool;
		}];
	}

	static function finiteFloorPose(pose:Null<SceneArtifactFloorPose>):Bool
		return pose != null && finite(pose.x) && finite(pose.y) && finite(pose.yaw);

	static function validateMission(mission:SceneArtifactMission, data:SceneArtifactData):Void {
		function fail(detail:String):Void throw 'Scene artifact mission $detail';
		if (mission.steps == null || mission.steps.length == 0) fail("has no steps");
		var definition = data.assemblyDefinition;
		var flat = definition == null ? null : AssemblyDefinitionFlattener.flatten(definition);
		/** Whether `place` names an existing connector of an existing occurrence. */
		function exists(place:Null<SceneArtifactPlace>):Bool {
			if (place == null || flat == null) return false;
			var occurrence = [for (item in flat.occurrences) if (item.id == place.occurrence) item];
			if (occurrence.length != 1) return false;
			for (component in flat.definitions) if (component.id == occurrence[0].definition)
				for (connector in component.connectors) if (connector.name == place.connector) return true;
			return false;
		}
		var handles = false, welds = false;
		for (index in 0...mission.steps.length) {
			var step = mission.steps[index];
			if (step == null) fail('step $index is empty');
			switch step.kind {
				case "goTo":
					if (data.mobileBase == null) fail('step $index drives, but the assembly has no mobile base');
					if (!finiteFloorPose(step.pose)) fail('step $index needs a finite pose');
				case "pick" | "place":
					handles = true;
					if (!exists(step.at)) fail('step $index names no connector of an occurrence in the assembly');
				case "weld":
					welds = true;
					validateWeld(step.weld, index, flat, fail);
				default: fail('step $index has unknown kind "${step.kind}"');
			}
		}
		var suctions = data.robotTools == null ? 0 : [for (tool in data.robotTools) if (tool.kind == "suction") tool].length;
		if (handles && suctions != 1) fail('picks and places, but the robot has $suctions suction tools, not one');
		var torches = data.robotTools == null ? 0 : [for (tool in data.robotTools) if (tool.kind == "torch") tool].length;
		if (welds && torches != 1) fail('welds, but the robot has $torches torches, not one');
		var tools = data.robotTools;
		if (welds && torches == 1 && tools != null) {
			var feeder = [for (tool in tools) if (tool.kind == "torch") tool][0].torch;
			if (feeder != null) for (index in 0...mission.steps.length) {
				var weld = mission.steps[index].weld;
				if (weld != null && weld.process != null && weld.process.wireSpeed > feeder.maxWireSpeedMPerMin)
					fail('step $index needs ${weld.process.wireSpeed} m/min of wire, past the feeder\'s top speed of ${feeder.maxWireSpeedMPerMin} m/min');
			}
		}
		if (welds && handles) fail("picks and places and welds: the robot's arm carries one tool, a suction cup or a torch");
	}

	static function validateWeld(weld:Null<SceneArtifactWeld>, index:Int, flat:Null<AssemblyDefinition>, fail:String -> Void):Void {
		if (weld == null) { fail('step $index welds nothing'); return; }
		if (flat == null || weld.metal == null || [for (item in flat.occurrences) if (item.id == weld.metal) item].length != 1)
			fail('step $index puts its weld metal on "${weld.metal}", which is not an occurrence of the assembly');
		if (weld.frame != null && weld.frame != "" && (flat == null || [for (item in flat.occurrences) if (item.id == weld.frame) item].length != 1))
			fail('step $index places its path by "${weld.frame}", which is not an occurrence of the assembly');
		if (weld.path == null || weld.path.length == 0) { fail('step $index welds no path'); return; }
		if (weld.path.length > 64) fail('step $index welds a path of more than 64 segments');
		function pose(value:Null<SceneArtifactTorchPose>, label:String):Void {
			if (value == null || value.position == null || value.position.length != 3 || value.rotation == null || value.rotation.length != 4) {
				fail('step $index has no $label pose');
				return;
			}
			for (number in value.position.concat(value.rotation)) if (!finite(number)) fail('step $index has a $label pose that is not finite');
			var norm = Math.sqrt(value.rotation[0] * value.rotation[0] + value.rotation[1] * value.rotation[1] +
				value.rotation[2] * value.rotation[2] + value.rotation[3] * value.rotation[3]);
			if (Math.abs(norm - 1) > 1e-3) fail('step $index has a $label rotation that is not a unit quaternion');
		}
		var previous:Null<SceneArtifactWeldSegment> = null;
		for (number in 0...weld.path.length) {
			var segment = weld.path[number];
			if (segment == null) { fail('step $index has an empty segment'); return; }
			var label = weld.path.length == 1 ? "" : ' segment $number';
			if (segment.kind != "line") fail('step $index$label is a "${segment.kind}", and only lines are welded');
			var halves = segment.seam == null ? [] : segment.seam.split("|");
			if (halves.length != 2) fail('step $index$label names no seam (member:face|member:face), got "${segment.seam}"');
			// Each half starts with the occurrence whose geometry names the face.
			for (half in halves) {
				var colon = half.indexOf(":");
				var member = colon > 0 ? half.substr(0, colon) : half;
				if (flat == null || [for (item in flat.occurrences) if (item.id == member) item].length != 1)
					fail('step $index$label welds a seam of "$member", which is not an occurrence of the assembly');
			}
			if (segment.joint != "fillet") fail('step $index$label welds a "${segment.joint}" joint, and only fillets are deposited');
			pose(segment.start, "start");
			pose(segment.stop, "stop");
			var length = 0.0;
			for (axis in 0...3) length += (segment.stop.position[axis] - segment.start.position[axis]) * (segment.stop.position[axis] - segment.start.position[axis]);
			if (!(length > 1e-8)) fail('step $index$label has a seam with no length');
			if (previous != null) {
				var gap = 0.0;
				for (axis in 0...3) gap += (segment.start.position[axis] - previous.stop.position[axis]) * (segment.start.position[axis] - previous.stop.position[axis]);
				if (!(gap < 1e-10)) fail('step $index$label does not start where the segment before it ends');
			}
			previous = segment;
			if (segment.normals == null || segment.normals.length != 2) fail('step $index$label needs the two faces\' normals');
			else for (normal in segment.normals) {
				if (normal == null || normal.length != 3) { fail('step $index$label has a face normal that is not a vector'); continue; }
				var norm = 0.0;
				for (component in normal) {
					if (!finite(component)) fail('step $index$label has a face normal that is not finite');
					norm += component * component;
				}
				if (Math.abs(Math.sqrt(norm) - 1) > 1e-3) fail('step $index$label has a face normal that is not a unit vector');
			}
			var a = segment.normals[0], b = segment.normals[1];
			var cosine = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
			if (Math.abs(cosine) > 1 - 1e-6) fail('step $index$label has faces that are parallel, which make no corner to fill');
			// The wire (+Z of the tip pose) points into the corner from its open side: against the sum of the outward normals.
			for (end in [segment.start, segment.stop]) {
				var q = end.rotation;
				var wire = [2 * (q[0] * q[2] + q[3] * q[1]), 2 * (q[1] * q[2] - q[3] * q[0]), 1 - 2 * (q[0] * q[0] + q[1] * q[1])];
				if (!(wire[0] * (a[0] + b[0]) + wire[1] * (a[1] + b[1]) + wire[2] * (a[2] + b[2]) < 0))
					fail('step $index$label has a wire that points out of the corner, not into it');
			}
		}
		if (!(finite(weld.legSize) && weld.legSize > 0 && weld.legSize <= 0.05)) fail('step $index needs a leg size between 0 and 50 mm');
		var process = weld.process;
		if (process == null) { fail('step $index has no process'); return; }
		if (!(finite(process.wireSpeed) && process.wireSpeed >= 1 && process.wireSpeed <= 30)) fail('step $index needs a wire speed between 1 and 30 m/min');
		if (!(finite(process.voltage) && process.voltage > 0 && process.voltage <= 60)) fail('step $index needs a voltage between 0 and 60 V');
		if (!(finite(process.travelSpeed) && process.travelSpeed > 0 && process.travelSpeed <= 0.05)) fail('step $index needs a travel speed between 0 and 50 mm/s');
		if (!(finite(process.approach) && process.approach > 0 && process.approach <= 0.5)) fail('step $index needs an approach distance between 0 and 500 mm');
		for (seconds in [process.startDwell, process.craterDwell])
			if (!(finite(seconds) && seconds >= 0 && seconds <= 10)) fail('step $index needs dwell times between 0 and 10 s');
		if (!(finite(process.burnback) && process.burnback >= 0.05 && process.burnback <= 10))
			fail('step $index needs a burnback between 0.05 and 10 s: with none the wire sticks in the pool');
	}

	/** A weld from its JSON section, typed field by field. */
	static function decodeWeld(raw:Dynamic):SceneArtifactWeld {
		function fail():Dynamic throw "Scene artifact weld is invalid";
		function number(value:Dynamic, name:String):Float {
			var item:Dynamic = Reflect.field(value, name);
			return Std.isOfType(item, Float) || Std.isOfType(item, Int) ? (item:Float) : fail();
		}
		function numbers(value:Dynamic, count:Int):Array<Float> {
			if (!Std.isOfType(value, Array) || (cast value:Array<Dynamic>).length != count) fail();
			return [for (item in (cast value:Array<Dynamic>)) Std.isOfType(item, Float) || Std.isOfType(item, Int) ? (item:Float) : fail()];
		}
		function pose(value:Dynamic):SceneArtifactTorchPose {
			if (value == null) fail();
			return {position: numbers(Reflect.field(value, "position"), 3), rotation: numbers(Reflect.field(value, "rotation"), 4)};
		}
		function segment(item:Dynamic):SceneArtifactWeldSegment {
			var kind:Dynamic = Reflect.field(item, "kind"), seam:Dynamic = Reflect.field(item, "seam"), joint:Dynamic = Reflect.field(item, "joint");
			var normals:Dynamic = Reflect.field(item, "normals");
			if (!Std.isOfType(kind, String) || !Std.isOfType(seam, String) || !Std.isOfType(joint, String) || !Std.isOfType(normals, Array)) fail();
			return {kind: kind, seam: seam, joint: joint, start: pose(Reflect.field(item, "start")), stop: pose(Reflect.field(item, "stop")),
				normals: [for (normal in (cast normals:Array<Dynamic>)) numbers(normal, 3)]};
		}
		var metal:Dynamic = Reflect.field(raw, "metal"), process:Dynamic = Reflect.field(raw, "process"), path:Dynamic = Reflect.field(raw, "path");
		if (!Std.isOfType(metal, String) || process == null) fail();
		// A weld written before paths existed carried its one straight seam on itself.
		var segments:Array<SceneArtifactWeldSegment> = path == null
			? [segment({kind: "line", seam: Reflect.field(raw, "seam"), joint: Reflect.field(raw, "joint"), start: Reflect.field(raw, "start"),
				stop: Reflect.field(raw, "stop"), normals: Reflect.field(raw, "normals")})]
			: Std.isOfType(path, Array) ? [for (item in (cast path:Array<Dynamic>)) segment(item)] : fail();
		var frame:Dynamic = Reflect.field(raw, "frame");
		if (frame != null && !Std.isOfType(frame, String)) fail();
		var weld:SceneArtifactWeld = {metal: metal, path: segments, legSize: number(raw, "legSize"),
			process: {wireSpeed: number(process, "wireSpeed"), voltage: number(process, "voltage"),
				travelSpeed: number(process, "travelSpeed"), approach: number(process, "approach"),
				startDwell: number(process, "startDwell"), craterDwell: number(process, "craterDwell"), burnback: number(process, "burnback")}};
		if (frame != null) weld.frame = frame;
		return weld;
	}

	/** A mission from its JSON section, typed field by field. */
	static function decodeMission(decoded:Dynamic):SceneArtifactMission {
		function fail():Dynamic throw "Scene artifact mission is invalid";
		function number(value:Dynamic, name:String):Float {
			var item:Dynamic = Reflect.field(value, name);
			return Std.isOfType(item, Float) || Std.isOfType(item, Int) ? (item:Float) : fail();
		}
		function place(value:Dynamic):SceneArtifactPlace {
			var occurrence:Dynamic = Reflect.field(value, "occurrence"), connector:Dynamic = Reflect.field(value, "connector");
			if (!Std.isOfType(occurrence, String) || !Std.isOfType(connector, String)) fail();
			return {occurrence: occurrence, connector: connector};
		}
		var steps:Dynamic = Reflect.field(decoded, "steps");
		if (!Std.isOfType(steps, Array)) fail();
		var mission:SceneArtifactMission = {steps: [for (raw in (cast steps:Array<Dynamic>)) {
			var kind:Dynamic = Reflect.field(raw, "kind");
			if (!Std.isOfType(kind, String)) fail();
			var step:SceneArtifactMissionStep = {kind: kind};
			var pose:Dynamic = Reflect.field(raw, "pose");
			if (pose != null) step.pose = {x: number(pose, "x"), y: number(pose, "y"), yaw: number(pose, "yaw")};
			var at:Dynamic = Reflect.field(raw, "at");
			if (at != null) step.at = place(at);
			var weld:Dynamic = Reflect.field(raw, "weld");
			if (weld != null) step.weld = decodeWeld(weld);
			step;
		}]};
		var loop:Dynamic = Reflect.field(decoded, "loop");
		if (loop != null) mission.loop = Std.isOfType(loop, Bool) ? (loop:Bool) : fail();
		return mission;
	}

	static function validateMobileBase(base:SceneArtifactMobileBase, definition:Null<AssemblyDefinition>):Void {
		function fail(detail:String):Void throw 'Scene artifact mobile base $detail';
		if (definition == null) fail("needs the robot's assembly definition");
		var flat = AssemblyDefinitionFlattener.flatten(definition);
		var prefix = base.robot == null ? "" : base.robot + "/";
		if (base.robot != null && [for (item in flat.occurrences) if (StringTools.startsWith(item.id, prefix)) item].length == 0)
			fail('robot "${base.robot}" has no occurrences');
		for (wheel in [base.leftWheel, base.rightWheel]) {
			var joint = [for (joint in flat.joints) if (joint.id == wheel) joint];
			if (joint.length != 1) fail('wheel "$wheel" is not a joint');
			var type = Std.string(joint[0].type);
			if (type != "continuous" && type != "revolute") fail('wheel "$wheel" is a $type joint, not a rotary one');
			if (!StringTools.startsWith(joint[0].child, prefix)) fail('wheel "$wheel" is not part of the robot');
		}
		if (base.origin != null && !finiteFloorPose(base.origin)) fail("needs a finite origin");
		if (base.leftWheel == base.rightWheel) fail("needs two different wheels");
		for (value in [base.wheelRadius, base.trackWidth, base.maxLinearSpeed, base.maxAngularSpeed,
				base.maxLinearAcceleration, base.maxAngularAcceleration])
			if (!finite(value) || value <= 0) fail("needs positive dimensions and limits");
		if ((base.footprintLength == null) != (base.footprintWidth == null)) fail("needs both footprint sides or neither");
		if (base.footprintLength != null) {
			var length:Float = cast base.footprintLength, width:Float = cast base.footprintWidth;
			if (!finite(length) || length <= 0 || !finite(width) || width <= 0) fail("needs a positive footprint");
		}
	}

	/** A mobile base from its JSON section, typed field by field. */
	static function decodeMobileBase(decoded:Dynamic):SceneArtifactMobileBase {
		function fail():Dynamic throw "Scene artifact mobile base is invalid";
		function text(name:String):String {
			var value:Dynamic = Reflect.field(decoded, name);
			return Std.isOfType(value, String) ? value : fail();
		}
		function number(name:String):Float {
			var value:Dynamic = Reflect.field(decoded, name);
			return Std.isOfType(value, Float) || Std.isOfType(value, Int) ? (value:Float) : fail();
		}
		var base:SceneArtifactMobileBase = {leftWheel: text("leftWheel"), rightWheel: text("rightWheel"),
			wheelRadius: number("wheelRadius"), trackWidth: number("trackWidth"),
			maxLinearSpeed: number("maxLinearSpeed"), maxAngularSpeed: number("maxAngularSpeed"),
			maxLinearAcceleration: number("maxLinearAcceleration"), maxAngularAcceleration: number("maxAngularAcceleration")};
		if (Reflect.field(decoded, "footprintLength") != null) base.footprintLength = number("footprintLength");
		if (Reflect.field(decoded, "footprintWidth") != null) base.footprintWidth = number("footprintWidth");
		function floorPose(value:Dynamic):SceneArtifactFloorPose {
			if (value == null) fail();
			function field(name:String):Float {
				var item:Dynamic = Reflect.field(value, name);
				return Std.isOfType(item, Float) || Std.isOfType(item, Int) ? (item:Float) : fail();
			}
			return {x: field("x"), y: field("y"), yaw: field("yaw")};
		}
		if (Reflect.field(decoded, "robot") != null) base.robot = text("robot");
		var origin:Dynamic = Reflect.field(decoded, "origin");
		if (origin != null) base.origin = floorPose(origin);
		return base;
	}

	static function validateMachining(machining:SceneArtifactMachining, parts:Map<String, Bool>,
			definition:Null<AssemblyDefinition>):Void {
		function fail(detail:String):Void throw 'Scene artifact machining job $detail';
		if (machining.program == null) fail("has no program");
		if (definition == null) throw "Scene artifact machining job needs the machine's assembly definition";
		var occurrences = new Map<String, Bool>(), joints = new Map<String, Bool>();
		for (occurrence in definition.occurrences) occurrences.set(occurrence.id, true);
		for (joint in definition.joints) joints.set(joint.id, true);
		if (machining.axes == null || machining.axes.length != 3) fail("needs three axes");
		for (index in 0...3) {
			var axis = machining.axes[index];
			if (!joints.exists(axis) || machining.axes.indexOf(axis) != index) fail('axis "$axis" is not a distinct joint');
		}
		if (!occurrences.exists(machining.spindle)) fail('spindle "${machining.spindle}" is not an occurrence');
		if (machining.stock != null && !occurrences.exists(machining.stock))
			fail('stock "${machining.stock}" is not an occurrence');
		if (machining.sacrificial != null) for (id in machining.sacrificial)
			if (!occurrences.exists(id)) fail('sacrificial part "$id" is not an occurrence');
		if (machining.toolPart != null && !occurrences.exists(machining.toolPart))
			fail('tool part "${machining.toolPart}" is not an occurrence');
		if (machining.workOffset == null || machining.workOffset.length != 3 ||
				!finite(machining.workOffset[0]) || !finite(machining.workOffset[1]) || !finite(machining.workOffset[2]))
			fail("needs a finite work offset");
		if (machining.tools == null || machining.tools.length == 0) fail("needs a tool table");
		var numbers = new Map<Int, Bool>();
		for (tool in machining.tools) {
			if (tool.number <= 0 || numbers.exists(tool.number)) fail('has a duplicate or invalid tool number ${tool.number}');
			numbers.set(tool.number, true);
			if (!finite(tool.length) || tool.length <= 0) fail('tool ${tool.number} needs a positive length');
			if (tool.profile == null || tool.profile.length == 0) fail('tool ${tool.number} has no profile');
			for (row in tool.profile) {
				if (row == null) throw 'Scene artifact machining job tool ${tool.number} has an empty profile row';
				for (value in row) if (!finite(value)) fail('tool ${tool.number} profile is not finite');
			}
		}
		if (machining.loadedTool != null && !numbers.exists(machining.loadedTool))
			fail('starts with tool ${machining.loadedTool}, which is not in its tool table');
		if (machining.target != null && !parts.exists(machining.target))
			fail('target "${machining.target}" is not one of its parts');
		var controller = machining.controller;
		if (controller != null && (controller.microsteps < 1 || controller.microsteps > 256 || controller.stepTickHz < 1))
			fail("has a controller with no microsteps or step rate");
	}

	/** A machining job from its JSON section, typed field by field. */
	static function decodeMachining(decoded:Dynamic):SceneArtifactMachining {
		function fail():Dynamic throw "Scene artifact machining job is invalid";
		function text(value:Dynamic):String return Std.isOfType(value, String) ? value : fail();
		function number(value:Dynamic):Float
			return Std.isOfType(value, Float) || Std.isOfType(value, Int) ? (value:Float) : fail();
		function list(value:Dynamic):Array<Dynamic> return Std.isOfType(value, Array) ? cast value : fail();
		var tools:Array<SceneArtifactTool> = [for (tool in list(Reflect.field(decoded, "tools"))) {
			var toolNumber:Dynamic = Reflect.field(tool, "number");
			if (!Std.isOfType(toolNumber, Int)) fail();
			{number: (toolNumber:Int), length: number(Reflect.field(tool, "length")),
				profile: [for (row in list(Reflect.field(tool, "profile"))) [for (value in list(row)) number(value)]]};
		}];
		var machining:SceneArtifactMachining = {program: text(Reflect.field(decoded, "program")),
			axes: [for (axis in list(Reflect.field(decoded, "axes"))) text(axis)],
			spindle: text(Reflect.field(decoded, "spindle")),
			workOffset: [for (value in list(Reflect.field(decoded, "workOffset"))) number(value)],
			tools: tools};
		var stock:Dynamic = Reflect.field(decoded, "stock");
		if (stock != null) machining.stock = text(stock);
		var sacrificial:Dynamic = Reflect.field(decoded, "sacrificial");
		if (sacrificial != null) machining.sacrificial = [for (id in list(sacrificial)) text(id)];
		var toolPart:Dynamic = Reflect.field(decoded, "toolPart");
		if (toolPart != null) machining.toolPart = text(toolPart);
		var loadedTool:Dynamic = Reflect.field(decoded, "loadedTool");
		if (loadedTool != null) machining.loadedTool = Std.isOfType(loadedTool, Int) ? (loadedTool:Int) : fail();
		var target:Dynamic = Reflect.field(decoded, "target");
		if (target != null) machining.target = text(target);
		var loop:Dynamic = Reflect.field(decoded, "loop");
		if (loop != null) machining.loop = Std.isOfType(loop, Bool) ? (loop:Bool) : fail();
		var controller:Dynamic = Reflect.field(decoded, "controller");
		if (controller != null) {
			var microsteps:Dynamic = Reflect.field(controller, "microsteps"), tick:Dynamic = Reflect.field(controller, "stepTickHz");
			if (!Std.isOfType(microsteps, Int) || !Std.isOfType(tick, Int)) fail();
			machining.controller = {microsteps: (microsteps:Int), stepTickHz: (tick:Int)};
		}
		return machining;
	}

	static function validatePart(part:SceneArtifactPart, requireEdgeIds:Bool = false):Void {
		if (part.name == null || StringTools.trim(part.name).length == 0 ||
			part.vertexCount <= 0 || part.vertexCount > MAX_VERTICES || part.indexCount <= 0 ||
			part.indexCount % 3 != 0 || part.indexCount / 3 > MAX_TRIANGLES ||
			part.faceRanges == null || part.faceRanges.length > MAX_FACE_RANGES ||
			part.vertices == null || part.vertices.length != part.vertexCount * 24 ||
			part.normals == null || part.normals.length != part.vertexCount * 24 ||
			part.indices == null || part.indices.length != part.indexCount * 4)
			throw 'Scene artifact part "${part.id}" has invalid mesh streams';
		if (part.edgeSegments != null) {
			if (part.edgeSegments.length % 48 != 0 || part.edgeSegments.length > MAX_BYTES)
				throw 'Scene artifact part "${part.id}" has invalid edge segments';
			for (offset in 0...Std.int(part.edgeSegments.length / 8))
				if (!finite(part.edgeSegments.getDouble(offset * 8)))
					throw 'Scene artifact part "${part.id}" has a non-finite edge endpoint';
		}
		if ((requireEdgeIds && part.edgeSegments != null && part.edgeSegments.length > 0 && part.edgeIds == null) ||
			(part.edgeIds != null &&
				(part.edgeSegments == null || part.edgeIds.length != Std.int(part.edgeSegments.length / 48) * 4)))
			throw 'Scene artifact part "${part.id}" has inconsistent edge identities';
		for (color in [part.red, part.green, part.blue])
			if (!finite(color) || color < 0.0 || color > 1.0)
				throw 'Scene artifact part "${part.id}" has an invalid color';
		var appearance = part.appearance;
		if (part.materialId != null && MaterialLibrary.get(part.materialId) == null &&
			(part.materialDensity == null || part.materialSpec == null))
			throw 'Scene artifact part "${part.id}" needs resolved custom material properties';
		if (part.materialId != null && (StringTools.trim(part.materialId).length == 0 ||
			Bytes.ofString(part.materialId).length > 4096))
			throw 'Scene artifact part "${part.id}" has an invalid material ID';
		if (part.materialDensity != null && (!finite(part.materialDensity) || part.materialDensity <= 0))
			throw 'Scene artifact part "${part.id}" has an invalid density';
		if (part.materialSpec != null && (StringTools.trim(part.materialSpec).length == 0 ||
			Bytes.ofString(part.materialSpec).length > 4096))
			throw 'Scene artifact part "${part.id}" has an invalid material specification';
		if (part.volume != null && (!finite(part.volume) || part.volume <= 0) ||
			part.centerOfMass != null && part.centerOfMass.length != 3 ||
			part.inertia != null && part.inertia.length != 9)
			throw 'Scene artifact part "${part.id}" has invalid mass properties';
		if (part.centerOfMass != null) for (coordinate in part.centerOfMass)
			if (!finite(coordinate)) throw 'Scene artifact part "${part.id}" has invalid centre of mass';
		if (part.inertia != null) for (component in part.inertia)
			if (!finite(component)) throw 'Scene artifact part "${part.id}" has invalid inertia';
		if (appearance != null && (appearance.finish == null ||
			StringTools.trim(appearance.finish).length == 0 ||
			Bytes.ofString(appearance.finish).length > 4096 ||
			!finite(appearance.metallic) || appearance.metallic < 0 || appearance.metallic > 1 ||
			!finite(appearance.roughness) || appearance.roughness < 0 || appearance.roughness > 1))
			throw 'Scene artifact part "${part.id}" has an invalid appearance';
		for (range in part.faceRanges)
			if (range.faceIndex < 0 || range.firstIndex < 0 || range.indexCount < 0 ||
				range.firstIndex % 3 != 0 || range.indexCount % 3 != 0 ||
				range.firstIndex + range.indexCount > part.indexCount)
				throw 'Scene artifact part "${part.id}" has an invalid face range';
		for (vertex in 0...part.vertexCount) for (axis in 0...3) {
			var offset = vertex * 24 + axis * 8;
			if (!finite(part.vertices.getDouble(offset)) || !finite(part.normals.getDouble(offset)))
				throw 'Scene artifact part "${part.id}" has a non-finite vertex or normal';
		}
		for (index in 0...part.indexCount) {
			var vertex = part.indices.getInt32(index * 4);
			if (vertex < 0 || vertex >= part.vertexCount)
				throw 'Scene artifact part "${part.id}" has an out-of-range index';
		}
	}

	static inline function finite(value:Float):Bool return value == value && value - value == 0.0;

	static function putInt(bytes:Bytes, offset:Int, value:Int):Int {
		bytes.set(offset, value); bytes.set(offset + 1, value >>> 8);
		bytes.set(offset + 2, value >>> 16); bytes.set(offset + 3, value >>> 24);
		return offset + 4;
	}
}

private class SceneArtifactReader {
	final source:Bytes;
	var offset:Int = 0;
	public function new(source:Bytes) this.source = source;

	public function read():SceneArtifactData {
		for (expected in [77, 84, 82, 71]) if (readByte() != expected)
			throw "Scene artifact has an invalid signature";
		var version = readInt();
		if (version != 2 && version != 3 && version != 4 && version != 5 && version != 6 && version != 7 &&
			version != 8 && version != 9 && version != 10 && version != 11 && version != 12 && version != 13 && version != SceneArtifact.VERSION)
			throw 'Unsupported scene artifact version $version';
		var metresPerUnit = readDouble();
		var lengthUnit = version >= 8 ? readText() : null;
		var count = readInt();
		if (!Math.isFinite(metresPerUnit) || metresPerUnit <= 0.0 || count <= 0 || count > 1000)
			throw "Scene artifact has invalid units or part count";
		var ids = new Map<String, Bool>(), parts:Array<SceneArtifactPart> = [];
		for (_ in 0...count) {
			var id = readText(), name = readText();
			if (ids.exists(id)) throw 'Scene artifact has duplicate part ID "$id"';
			ids.set(id, true);
			var red = readFloat(), green = readFloat(), blue = readFloat();
			var appearance:Appearance = version >= 7
				? {finish: readText(), metallic: readFloat(), roughness: readFloat()}
				: Appearances.neutral();
			var materialId = version >= 8 ? readText() :
				(MaterialLibrary.get(appearance.finish) == null ? "neutral" : appearance.finish);
			var density = version >= 8 ? readDouble() : null;
			var materialSpec = version >= 8 ? readText() : null;
			var volume = version >= 8 ? readDouble() : null;
			var center = version >= 8 ? [for (_ in 0...3) readDouble()] : null;
			var inertia = version >= 8 ? [for (_ in 0...9) readDouble()] : null;
			var vertexCount = readInt(), indexCount = readInt(), rangeCount = readInt();
			var edgeByteCount = version >= 4 ? readInt() : 0;
			if (vertexCount <= 0 || vertexCount > 2000000 || indexCount <= 0 ||
				indexCount % 3 != 0 || indexCount / 3 > 4000000 || rangeCount < 0 || rangeCount > 100000 ||
				edgeByteCount < 0 || edgeByteCount % 48 != 0 || edgeByteCount > SceneArtifact.MAX_BYTES)
				throw 'Scene artifact part "$id" has invalid mesh counts';
		var vertices = readBytes(vertexCount * 24), normals = readBytes(vertexCount * 24),
				indices = readBytes(indexCount * 4), edgeSegments = readBytes(edgeByteCount);
			var edgeIds = version >= 5 ? readBytes(Std.int(edgeByteCount / 48) * 4) : null;
			var faceRanges:Array<SceneArtifactFaceRange> = [];
			for (_ in 0...rangeCount)
				faceRanges.push({faceIndex: readInt(), firstIndex: readInt(), indexCount: readInt()});
			var faceDescriptors = version >= 11 ? readOpaqueText(16000000, 'part "$id" face descriptors') : "";
			var part:SceneArtifactPart = {id: id, name: name, red: red, green: green, blue: blue,
				appearance: appearance, materialId: materialId, materialDensity: density, materialSpec: materialSpec,
				volume: volume, centerOfMass: center, inertia: inertia,
				vertexCount: vertexCount, indexCount: indexCount, vertices: vertices, normals: normals,
				indices: indices, edgeSegments: edgeSegments, edgeIds: edgeIds,
				faceRanges: faceRanges};
			if (faceDescriptors.length > 0) part.faceDescriptors = faceDescriptors;
			@:privateAccess SceneArtifact.validatePart(part, version >= 5);
			parts.push(part);
		}
		var assembly:Null<AssemblyRecord> = null;
		if (version >= 3) {
			var length = readInt();
			if (length < 0 || length > 2000000) throw "Scene artifact assembly metadata is too large";
			if (length > 0) assembly = AssemblyCodec.decode(readBytes(length).getString(0, length));
		}
		var assemblyDefinition:Null<AssemblyDefinition> = null, assemblyState:Null<AssemblyStateRecord> = null;
		if (version >= 6) {
			var definitionLength = readInt();
			if (definitionLength < 0 || definitionLength > 2000000)
				throw "Scene artifact assembly definition is too large";
			if (definitionLength > 0)
				assemblyDefinition = AssemblyDefinitionCodec.decode(readBytes(definitionLength).getString(0, definitionLength));
			var stateLength = readInt();
			if (stateLength < 0 || stateLength > 2000000)
				throw "Scene artifact assembly state is too large";
			if (stateLength > 0) {
				if (assemblyDefinition == null) throw "Scene artifact assembly state has no definition";
				assemblyState = AssemblyDefinitionCodec.decodeState(assemblyDefinition,
					readBytes(stateLength).getString(0, stateLength));
			}
		}
		var recipeDocument:Null<String> = null;
		if (version >= 9) {
			var documentLength = readInt();
			if (documentLength < 0 || documentLength > 2000000)
				throw "Scene artifact recipe document is too large";
			if (documentLength > 0) recipeDocument = readBytes(documentLength).getString(0, documentLength);
		}
		var recipeDiagnostics:Null<Array<String>> = null;
		if (version >= 10) {
			var diagnosticsLength = readInt();
			if (diagnosticsLength < 0 || diagnosticsLength > 2000000)
				throw "Scene artifact recipe diagnostics are too large";
			if (diagnosticsLength > 0) {
				var decoded:Dynamic = haxe.Json.parse(readBytes(diagnosticsLength).getString(0, diagnosticsLength));
				if (!Std.isOfType(decoded, Array)) throw "Scene artifact recipe diagnostics are invalid";
				for (entry in (cast decoded:Array<Dynamic>))
					if (!Std.isOfType(entry, String)) throw "Scene artifact recipe diagnostics are invalid";
				recipeDiagnostics = cast decoded;
			}
		}
		var machining:Null<SceneArtifactMachining> = null;
		if (version >= 12) {
			var machiningLength = readInt();
			if (machiningLength < 0 || machiningLength > 8000000) throw "Scene artifact machining job is too large";
			if (machiningLength > 0) {
				machining = @:privateAccess SceneArtifact.decodeMachining(haxe.Json.parse(readBytes(machiningLength).getString(0, machiningLength)));
			}
		}
		var mobileBase:Null<SceneArtifactMobileBase> = null;
		if (version >= 13) {
			var mobileLength = readInt();
			if (mobileLength < 0 || mobileLength > 100000) throw "Scene artifact mobile base is too large";
			if (mobileLength > 0)
				mobileBase = @:privateAccess SceneArtifact.decodeMobileBase(haxe.Json.parse(readBytes(mobileLength).getString(0, mobileLength)));
		}
		var mission:Null<SceneArtifactMission> = null;
		if (version >= 13) {
			var missionLength = readInt();
			if (missionLength < 0 || missionLength > 1000000) throw "Scene artifact mission is too large";
			if (missionLength > 0)
				mission = @:privateAccess SceneArtifact.decodeMission(haxe.Json.parse(readBytes(missionLength).getString(0, missionLength)));
		}
		var robotTools:Null<Array<SceneArtifactRobotTool>> = null;
		if (version >= 13) {
			var toolsLength = readInt();
			if (toolsLength < 0 || toolsLength > 100000) throw "Scene artifact robot tools are too large";
			if (toolsLength > 0)
				robotTools = @:privateAccess SceneArtifact.decodeRobotTools(haxe.Json.parse(readBytes(toolsLength).getString(0, toolsLength)));
		}
		var robotSensors:Null<Array<SceneArtifactRobotSensor>> = null;
		if (version >= 14) {
			var sensorsLength = readInt();
			if (sensorsLength < 0 || sensorsLength > 100000) throw "Scene artifact robot sensors are too large";
			if (sensorsLength > 0)
				robotSensors = @:privateAccess SceneArtifact.decodeRobotSensors(haxe.Json.parse(readBytes(sensorsLength).getString(0, sensorsLength)));
		}
		if (offset != source.length) throw "Scene artifact contains trailing data";
		var result:SceneArtifactData = {metresPerUnit: metresPerUnit, lengthUnit: lengthUnit,
			parts: parts, assembly: assembly,
			assemblyDefinition: assemblyDefinition, assemblyState: assemblyState, recipeDocument: recipeDocument,
			recipeDiagnostics: recipeDiagnostics, machining: machining, mobileBase: mobileBase, mission: mission,
			robotTools: robotTools, robotSensors: robotSensors};
		@:privateAccess SceneArtifact.validateHeader(result);
		return result;
	}

	/** A length-prefixed text that may be empty, up to `limit` bytes. */
	function readOpaqueText(limit:Int, what:String):String {
		var length = readInt();
		if (length < 0 || length > limit) throw 'Scene artifact $what are too large';
		return length == 0 ? "" : readBytes(length).getString(0, length);
	}

	function readText():String {
		var length = readInt();
		if (length <= 0 || length > 4096) throw "Scene artifact has an invalid part ID or name";
		var value = readBytes(length).getString(0, length);
		if (StringTools.trim(value).length == 0) throw "Scene artifact has an empty part ID or name";
		return value;
	}

	function readByte():Int {
		if (offset >= source.length) throw "Scene artifact ended unexpectedly";
		return source.get(offset++);
	}
	function readInt():Int {
		var a = readByte(), b = readByte(), c = readByte(), d = readByte();
		return a | (b << 8) | (c << 16) | (d << 24);
	}
	function readDouble():Float {
		var value = readBytes(8);
		return value.getDouble(0);
	}
	function readFloat():Float {
		var value = readBytes(4);
		return value.getFloat(0);
	}
	function readBytes(length:Int):Bytes {
		if (length < 0 || length > source.length - offset)
			throw "Scene artifact has an invalid buffer length";
		var result = Bytes.alloc(length);
		result.blit(0, source, offset, length);
		offset += length;
		return result;
	}
}
