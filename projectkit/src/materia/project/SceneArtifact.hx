package materia.project;

import haxe.io.Bytes;
import haxeon.wire.JsonWire;
import materia.project.ThreadRenderData;
import materia.project.ThreadRenderData.ThreadRenderDataValues;
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
	/** Resolved physical mass; assembly coordinates still follow the scene unit. */
	@:optional var physicalMassKg:Float;
	/** Whether the mass is explicitly declared, so a later material edit does not replace it. */
	@:optional var physicalMassDeclared:Bool;
	/** Centroidal inertia in kg m², row-major, about `centerOfMass`. */
	@:optional var physicalInertiaKgM2:Array<Float>;
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
		without the B-rep author mates on faces.
	*/
	@:optional var faceDescriptors:String;
	/** Named semantic intervals derived beside this exact tessellation. */
	@:optional var threads:Array<ThreadRenderData>;
}

typedef SceneArtifactData = {
	@:optional var lengthUnit:String;
	/** Metres represented by one coordinate in every mesh stream. */
	var metresPerUnit:Float;
	var parts:Array<SceneArtifactPart>;
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
  /** Moving belts and an optional axis program supplied by the machine's generator. */
	@:optional var machineMotion:SceneArtifactMachineMotion;
	/** Independently clocked control domains derived from assembly signal wiring. */
	@:optional var controllers:Array<SceneArtifactCellController>;
	/** Digital wires between cabinet terminals. */
	@:optional var signals:Array<SceneArtifactCellSignal>;
  /** Project-owned runtime settings resolved by an artifact publisher, independent of source files. */
  @:optional var project:SceneArtifactProject;
	/** A print job the generator made for its own printer; see SceneArtifactPrinting. */
	@:optional var printing:SceneArtifactPrinting;
}

typedef SceneArtifactCellController = {
	var id:String;
	var kind:String;
	var joints:Array<String>;
	var sensors:Array<String>;
	var channels:Array<String>;
}

typedef SceneArtifactSignalPort = {var controller:String; var port:String;}

typedef SceneArtifactCellSignal = {
	var name:String;
	var from:SceneArtifactSignalPort;
	var to:SceneArtifactSignalPort;
}

typedef SceneArtifactProject = {
  @:optional var job:String;
  var dynamicParts:Array<String>;
  var motions:Array<SceneArtifactProjectMotion>;
}

typedef SceneArtifactProjectMotion = {
  var joint:String;
  var loop:Bool;
  var keys:Array<{time:Float, position:Float}>;
}

@:wire typedef SceneArtifactMachineMotion = {
  @:id(1) var belts:Array<SceneArtifactBeltVisual>;
  @:id(2) @:optional var program:SceneArtifactAxisProgram;
  /** Execute the authored machine program on a streamed stepper board in the app. */
  @:id(3) @:optional var virtualDevice:Bool;
  /** Contact surfaces of CAD hulls, keyed by assembly occurrence. */
  @:id(4) @:optional var contacts:Array<SceneArtifactHullContact>;
  /** Timed digital process outputs executed by the native plan runtime. */
  @:id(5) @:optional var digitalProgram:SceneArtifactDigitalProgram;
  /** Feedback-driven press controller, independent of the robot mission. */
  @:id(6) @:optional var pressBrake:SceneArtifactPressBrake;
}

/** Press travel and gauge setup use millimetres; axis planning converts to SI once. */
@:wire typedef SceneArtifactPressBrakeStep = {
  @:id(1) var gaugeXmm:Float;
  @:id(2) var gaugeRmm:Float;
  @:id(3) var clampTravelMm:Float;
  @:id(4) var depthTravelMm:Float;
  @:id(5) var withdrawalMm:Float;
}

/** Runtime bindings are authored semantic ids; geometry supplies each program step. */
@:wire typedef SceneArtifactPressBrake = {
  @:id(1) @:optional var controllerId:String;
  @:id(2) var ramJoint:String;
  @:id(3) var depthAxis:String;
  @:id(4) var gaugeX:String;
  @:id(5) @:optional var gaugeR:String;
  @:id(6) var topSwitch:String;
  @:id(7) var fingerSwitches:Array<String>;
  @:id(8) var steps:Array<SceneArtifactPressBrakeStep>;
  @:id(9) var positionToleranceMm:Float;
  @:id(10) var requestTimeoutSeconds:Float;
  /** Explicit simulation unloading criterion, not a chamber-pressure measurement. */
  @:id(11) var releaseTravelMm:Float;
  @:id(12) var releaseClosingForceN:Float;
}

@:wire typedef SceneArtifactBeltVisual = {
  @:id(1) var occurrence:String;
  @:id(2) var pitch:Float;
  @:id(3) var width:Float;
  @:id(4) var thickness:Float;
  @:id(5) var driverJoint:String;
  @:id(6) var driverWrap:Int;
  @:id(7) var driverSign:Float;
  @:id(8) var wraps:Array<SceneArtifactBeltWrap>;
}

@:wire typedef SceneArtifactBeltWrap = {
  @:id(1) var occurrence:String;
  @:id(2) var connector:String;
  @:id(3) var radius:Float;
  @:id(4) var side:Int;
}

/** Metre-valued independent-axis waypoints, run as exact-stop moves. */
@:wire typedef SceneArtifactAxisProgram = {
  @:id(1) var axes:Array<String>;
  @:id(2) var waypoints:Array<Array<Float>>;
  @:id(3) var loop:Bool;
  /** Allowed measured endpoint error in axis SI units, including device step resolution. */
  @:id(4) @:optional var positionTolerance:Float;
}


@:wire typedef SceneArtifactHullContact = {
  @:id(1) var occurrence:String;
  @:id(2) var surface:SceneContactSurface;
}

@:wire typedef SceneArtifactDigitalProgram = {
  @:id(1) var durationSec:Float;
  @:id(2) var events:Array<SceneArtifactDigitalEvent>;
  @:id(3) var loop:Bool;
}

@:wire typedef SceneArtifactDigitalEvent = {
  @:id(1) var timeSec:Float;
  @:id(2) var channel:String;
  @:id(3) var value:Bool;
}

/**
 * A sensor on the assembly's robot, as its part declares it. A `lidar` scans the
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
 * A tool on the assembly's robot and how it is worked, as its parts' declared
 * capabilities say: a `suction` tool seals on what touches the occurrence `contact.occurrence` at its
 * connector `contact.connector` while the digital process channel `channel` is on, and reports the
 * vacuum it pulls on the sensor `sensor` when it has one. A `gripper` has physical jaws, closes on
 * the same kind of digital channel, and uses its native joint switch `sensor` as grasp feedback.
 *
 * A `torch` tool is a MIG/MAG welding torch: `contact` is the connector at its wire tip (the tool point),
 * `channel` is the digital channel that lights its arc, `sensor` is its `tool_weld` sensor, and `torch`
 * describes the welder behind it. A `screwdriver` carries its bit, capacities and named tool joints;
 * its process sensor and tightening observer are supplied by the driving process. Tool data requires
 * the current scene schema; older versions are rejected.
 */
typedef SceneArtifactRobotTool = {
	var kind:String;
	/** Exact CAD members of the complete mounted tool, including its plate and services. */
	var members:Array<String>;
	var contact:SceneArtifactPlace;
	var channel:String;
	@:optional var sensor:String;
	@:optional var gripper:SceneArtifactGripper;
	@:optional var torch:SceneArtifactTorch;
	@:optional var screwdriver:SceneArtifactScrewdriver;
}

/** Nominal bit identity, portable without a dependency on MachineKit. */
typedef SceneArtifactScrewdriverBit = {var kind:String; var size:String;}
/** Tool capacity in Nm and rad/s, travel in mm, spring rate in N/mm. Joint names identify real tool motion. */
typedef SceneArtifactScrewdriver = {
	var bit:SceneArtifactScrewdriverBit;
	var directionChannel:String;
	var programChannel:String;
	var vacuumChannel:String;
	var feedJoint:String;
	var spindleJoint:String;
	var strokeEncoder:String;
	var maxTorque:Float;
	var maxSpeed:Float;
	var strokeMm:Float;
	var springRate:Float;
}

/** Assumed physical limits used to choose and orient a grasp from the blank's planar faces. */
typedef SceneArtifactGripper = {
	/** Exact CAD members which may contact held stock. */
	var padMembers:Array<String>;
	/** Exact CAD members of this gripper's mechanical envelope. */
	var envelopeMembers:Array<String>;
	var minimumPartWidthMm:Float;
	var nominalPartWidthMm:Float;
	var maximumPartWidthMm:Float;
	var fingerLengthMm:Float;
	/** Assumed sliding coefficient of the physical finger pads. */
	var fingerPadFrictionCoefficient:Float;
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
 * The assembly is a differential-drive robot that rolls on the floor. Its root body
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
	/** Battery, conversion, auxiliary-load and dock data derived from the CAD machine. */
	@:optional var energy:SceneArtifactMobileEnergy;
}

typedef SceneArtifactMobileEnergy = {
	/** Runtime scenario state; electrical ratings below come from the CAD pack and service components. */
	var initialChargeFraction:Float;
	var batteryOccurrence:String;
	var nominalVolts:Float;
	var capacityWh:Float;
	var fullVoltage:Float;
	var emptyVoltage:Float;
	var cutoffVoltage:Float;
	var internalResistanceOhms:Float;
	var maxDischargeCurrentA:Float;
	var maxChargePowerW:Float;
	var chargeEfficiency:Float;
	var inverterContinuousWatts:Float;
	var inverterEfficiency:Float;
	var armSupplyEfficiency:Float;
	var driveSupplyEfficiency:Float;
	var computerSupplyEfficiency:Float;
	/** Electrical output to mechanical joint power; explicit motor and driver model assumptions. */
	var armDriveEfficiency:Float;
	var driveMotorEfficiency:Float;
	var driveGearEfficiency:Float;
	var computerLoadWatts:Float;
	var chargerOccurrence:String;
	var chargerPowerWatts:Float;
	var dockPose:SceneArtifactFloorPose;
	/** Physical charging-contact alignment bounds, in metres and radians. */
	var dockPositionToleranceMeters:Float;
	var dockHeadingToleranceRadians:Float;
}

/**
 * Work the assembly's robot does on its own when the simulation runs: its
 * steps in order, starting over after the last when `loop`. Steps name what they act on in the
 * assembly, so where they lead follows the model. Picking and placing work the robot's one suction
 * tool (see SceneArtifactRobotTool).
 */
/** Physical initial displacement in SI units from the saved pose of a home-monitored joint. */
typedef SceneArtifactPowerUpOffset = {
	var joint:String;
	var offset:Float;
}

/** Startup racking in axis SI units, identified by the physical side home switch. */
typedef SceneArtifactPowerUpSideOffset = {
	var homeSwitch:String;
	var offset:Float;
}

typedef SceneArtifactMission = {
	var steps:Array<SceneArtifactMissionStep>;
	@:optional var loop:Bool;
	@:optional var powerUpOffsets:Array<SceneArtifactPowerUpOffset>;
	@:optional var powerUpSideOffsets:Array<SceneArtifactPowerUpSideOffset>;
	@:optional var controllerId:String;
}

/** Generic geometric labels, with optional physical arm turn counts. */
typedef SceneArtifactArmConfiguration = {
	var shoulder:String;
	var elbow:String;
	var wrist:String;
	@:optional var turns:Array<Int>;
}

/** One digital output command owned by a cell controller. */
typedef SceneArtifactMissionSignal = {
	var name:String;
	var value:Bool;
}

/** A controller input condition with a simulation-time deadline. */
typedef SceneArtifactMissionWait = {
	var name:String;
	var value:Bool;
	var timeout:Float;
	/** Earlier output step to repeat after an operator removes the cause and resets the fault. */
	@:optional var retryStep:Int;
}

/** A waypoint is exactly one of a joint target or a robot-root-frame Cartesian pose. */
typedef SceneArtifactMissionWaypoint = {
	@:optional var joints:Array<SceneArtifactJointTarget>;
	@:optional var pose:SceneArtifactTorchPose;
}

/**
 * One step of a mission.
 * - `goTo`: drive the mobile base to `pose`, in the assembly frame on the floor.
 * - `pick`: take hold of the occurrence `at.occurrence` with the tool, meeting it at its connector
 *   `at.connector`, wherever the part is when the step starts.
 * - `place`: set what the tool holds down with its base on the connector `at.connector` of
 *   `at.occurrence`, and let go.
 * - `moveJoints`: move the chain ending at `at` to absolute joint coordinates in radians or metres.
 * - `signal`: set one digital output on the mission's controller.
 * - `waitFor`: wait for one digital input on the mission's controller until its simulation-time timeout.
 * - `moveVia`: move the chain ending at `at` through exact-stop joint or robot-root-frame pose waypoints.
 * - `weld`: weld the seam `weld` describes with the robot's torch.
 * - `findWork`: measure the named work frame from contact probes on saved nominal CAD faces.
 * - `stow`: return the welding arm to its CAD ready posture using its measured joints and live cell clearance.
 */
typedef SceneArtifactMissionStep = {
	var kind:String;
	/** Hard pin for all arm motion in this step, including entry and return. */
	@:optional var configuration:SceneArtifactArmConfiguration;
	@:optional var pose:SceneArtifactFloorPose;
	@:optional var at:SceneArtifactPlace;
	/** Requested part heading in world radians; omission leaves tool spin free. */
	@:optional var yaw:Float;
	/** Requested handling-tool roll about world +Z in radians; independent of part heading. */
	@:optional var toolYaw:Float;
	@:optional var weld:SceneArtifactWeld;
	@:optional var joints:Array<SceneArtifactJointTarget>;
	@:optional var contactWork:SceneContactRegistration.SceneContactWork;
	@:optional var signal:SceneArtifactMissionSignal;
	@:optional var waitFor:SceneArtifactMissionWait;
	@:optional var poses:Array<SceneArtifactMissionWaypoint>;
	/** Simulation-time deadline for an asynchronous handling or motion step. */
	@:optional var timeoutSeconds:Float;
	/** Cartesian pickup/place approach height in metres. */
	@:optional var approachDistance:Float;
	/** Logical tool channel used when a robot has more than one handling tool. */
	@:optional var tool:String;
}

/** Absolute mechanical joint position, in radians or metres. */
typedef SceneArtifactJointTarget = {
	var joint:String;
	var position:Float;
}

/**
 * A weld to run and how. What is welded is a result of the CAD, never authored: `path` is the seams as the CAD
 * found them, run one after the other as one weld (a straight seam is a path of one segment, the four sides of a
 * tube a path of four), and the rest is what the generator derived from them, so a player needs no CAD to weld
 * them. Lengths are metres, whatever the scene's unit.
 * - `path` is the segments in order, each starting where the one before ends. The torch strikes the arc at the
 *   start of the first, keeps it up along the whole path and ends it at the end of the last. Where one segment's
 *   torch angles differ from the next one's, as at the corner of a tube, the torch turns from one to the other
 *   as it passes the joint. All passes share this CAD path and together deposit the requested leg.
 * - `frame` is the occurrence the path is relative to, the workpiece's reference member: every pose and normal in the
 *   path is in that occurrence's own frame (metres, with its normals turned with it). A player finds the occurrence where it
 *   stands when the step starts and places the path by it, so the weld follows a workpiece that is not where it was
 *   designed. Without a `frame` the path is in the assembly as designed (the form welds took before it existed).
 * - `metal` is the occurrence that carries the weld metal: the part the bead is shown on as the welder lays it.
 * - `legSize` is the leg the weldment asks for.
 * - `passes` give ordered processes, face offsets, weaving and cooling over that path. The generator derives
 *   their deposited areas from the target leg; the measured final leg is checked against that target.
 *
 */
typedef SceneArtifactWeld = {
	@:optional var frame:String;
	var metal:String;
	var path:Array<SceneArtifactWeldSegment>;
	var legSize:Float;
	var passes:Array<SceneArtifactWeldPass>;
}

/** Ordered passes over the CAD path. Offsets are metres along its two outward face normals. */
typedef SceneArtifactWeldPass = {
	var name:String;
	var offset:Array<Float>;
	var process:SceneArtifactWeldProcess;
	@:optional var weave:SceneArtifactWeldWeave;
	var interpassDwell:Float;
}

/** Frequency is spatial, so timing changes never change the authored weave. */
typedef SceneArtifactWeldWeave = {
	var pattern:String;
	var amplitude:Float;
	var cyclesPerMetre:Float;
	var edgeDwell:Float;
}

/** Physical CAD identity is separate from a seam's display name. */
typedef SceneArtifactWeldFace = {
	var member:String;
	var face:String;
}

/**
 * One primitive of a weld path, the piece of a seam the CAD found. `kind` says what shape it is: only a `line` so far,
 * straight from `start` to `stop` with the torch holding its orientation; an `arc` for a curved seam would add its
 * centre and sweep, and be another kind.
 * - `seam` is its name as the CAD found it (`member:face|member:face`, the two faces the weld metal fills against,
 *   each in the geometry of the occurrence `member`).
 * - `faces` carry the two CAD member/face identities; consumers never parse `seam`.
 * - `joint` is how the members meet; only a `fillet` is deposited so far.
 * - `start` and `stop` are the poses of the wire tip at the two ends: +Z along the wire out of the torch, +X along
 *   the direction of travel as far as it is square to the wire. They carry the work and travel angles.
 * - `normals` are the two faces' outward unit normals, which say where the weld metal sits (the fillet
 *   fills the corner they leave open).
 */
typedef SceneArtifactWeldSegment = {
	var kind:String;
	var seam:String;
	var faces:Array<SceneArtifactWeldFace>;
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
 * version 20). Lengths are metres, whatever the scene's unit.
 *
 * The machine's `axes` are three prismatic joints whose positions are machine X, Y and Z, and machine
	 * coordinates name where the spindle's gauge line is: the origin of the `spindle` occurrence, whose
	 * +Z runs up the spindle axis. `workOffset` is G54 in machine coordinates. Each tool in `tools` hangs
 * `length` below the gauge line and has the shape `profile` (a toolpath cutter profile's `encode`),
	 * so the program's G43 H numbers are the tool numbers. `stocks` are the occurrences the tool may cut and
 * `sacrificial` those it may run into without harm, such as a spoilboard under through holes.
 * `toolPart` is the occurrence that shows the tool in the spindle, whose shape changes with the tool,
 * and `loadedTool` the number of the tool in the spindle when the job starts.
 * `target` names a part of the artifact that no occurrence uses: the finished part, in the stock's
 * frame, to compare the machined stock with. A looping job starts again when the program ends.
 * `controller` is the controller the machine's steppers are nominally wired to: the machine's own
 * limits are the motors' and drives', and the controller's step rate caps them further.
 */
typedef SceneArtifactMachining = {
	@:optional var powerUpOffsets:Array<SceneArtifactPowerUpOffset>;
	@:optional var powerUpSideOffsets:Array<SceneArtifactPowerUpSideOffset>;
	var program:String;
	var axes:Array<String>;
	var spindle:String;
	/** Process-channel namespace, explicitly selected by the machine generator. */
	var channelScope:String;
	var workOffset:Array<Float>;
	var tools:Array<SceneArtifactTool>;
	var stocks:Array<String>;
	@:optional var sacrificial:Array<String>;
	@:optional var toolPart:String;
	@:optional var loadedTool:Int;
	@:optional var target:String;
	@:optional var loop:Bool;
	@:optional var controller:SceneArtifactController;
	@:optional var controllerId:String;
}

/**
 * The controller generates at most `stepTickHz` step edges a second per channel. Driver
 * microstepping and input-rate limits belong to the machine's actuators; binding combines them.
 */
typedef SceneArtifactController = {
	var stepTickHz:Int;
}

/** One tool of a machining job's tool table; see SceneArtifactMachining. */
typedef SceneArtifactTool = {
	var number:Int;
	var length:Float;
	var profile:Array<Array<Float>>;
}


/**
 * A print job the generator made for its own printer. Lengths are metres, whatever the scene's unit.
 *
 * `source` is the program as the printer's G-code (Klipper flavour: relative extrusion, `M104`/`M109`/`M140`/`M190`,
 * `M106`/`M107`), in machine coordinates: the positions of the three `axes`, prismatic joints whose values are
 * machine X, Y and Z (the bed's Z grows as the bed goes down, so the nozzle rises relative to it). The machine starts
 * where the assembly's initial pose puts it, referenced: a leading `G28` is not run, because the virtual device has
 * no switch inputs for a CoreXY yet. The `extruder` is the joint of the filament drive gear: it turns
 * `1 / gearRadius` radians per metre of filament, and the hotend melts at most `maxFlow` cubic metres a second.
 * `pressureAdvance` seconds (0 is off) is how far the extruder runs ahead of the flow it feeds, averaged over
 * `pressureAdvanceWindow` seconds; `rampLength` is how far along the path its rate ramps. `meltLag` seconds is how long
 * the melt takes to follow the filament: what flows out of the nozzle is the feed through a lag of that time, which
 * pressure advance (of the same time) cancels. `controller` is the
 * controller the steppers are wired to, as in a machining job.
 *
 * Each heater (the hotend, the bed) is run by the controller: the host sets its target on the analog channel
 * `target`, the board's PID and watchdog do the rest, and its report is the `tool_heater` sensor `sensor`, in the
 * board's slot `slot`, mounted on the occurrence `mount`. The numbers are the controller's tuning and the part's
 * assumed thermal plant. A heater's `fan` is the analog channel (duty 0 to 1) of the part fan that blows on it;
 * `feedsExtruder` says that pushing filament while it is below `minExtrusionTemperature` is a fault. `deposit`, when
 * given, is the occurrence that shows what is printed: the simulation lays the material on it as geometry of its own,
 * in the frame of the occurrence, whose origin is the machine's zero (the bed's front left corner at its surface),
 * and it rides the bed. `shaping`, when given, names the axes whose motion is input-shaped and with what: the shaper
 * (`zv`, `mzv` or `ei`) cancels the vibration of a mode at `frequency` hertz and `damping` ratio, and the other joints
 * are delayed to stay in step (see `InputShaper`).
 */
typedef SceneArtifactPrinting = {
	var source:String;
	var axes:Array<String>;
	var extruder:String;
	var filamentDiameter:Float;
	var gearRadius:Float;
	var maxFlow:Float;
	var pressureAdvance:Float;
	var pressureAdvanceWindow:Float;
	var meltLag:Float;
	var rampLength:Float;
	var heaters:Array<SceneArtifactHeater>;
	var controller:SceneArtifactController;
	/** The material's minimum extrusion temperature, degrees Celsius: the static check and the controller refuse to extrude colder. */
	var minExtrusionTemperature:Float;
	@:optional var deposit:String;
	@:optional var shaping:Array<SceneArtifactShaper>;
}

/** One axis's input shaper in a print job; see SceneArtifactPrinting. */
typedef SceneArtifactShaper = {
	var axis:String;
	var kind:String;
	var frequency:Float;
	var damping:Float;
}

/** One heater of a print job; see SceneArtifactPrinting. */
typedef SceneArtifactHeater = {
	var name:String;
	var sensor:String;
	var slot:Int;
	var mount:String;
	var target:String;
	@:optional var fan:String;
	/** The heater feeds the extruder: pushing filament while it is cold is a fault. */
	var feedsExtruder:Bool;
	var r0Ohm:Float;
	var t0Celsius:Float;
	var beta:Float;
	var pullupOhm:Float;
	var vRef:Float;
	var kp:Float;
	var ki:Float;
	var kd:Float;
	var maxDuty:Float;
	var maxTemperature:Float;
	var bandCelsius:Float;
	var pidRangeCelsius:Float;
	var gainWindowSeconds:Float;
	var gainMinCelsius:Float;
	var hysteresisCelsius:Float;
	var hysteresisWindowSeconds:Float;
	var heatCapacity:Float;
	var power:Float;
	var loss:Float;
	var fanLoss:Float;
	var ambient:Float;
}

/** Versioned, producer-independent scene geometry exchange format. */
class SceneArtifact {
	public static inline var VERSION:Int = 33;
	public static inline var MAX_BYTES:Int = 150000000;
	static inline var MAX_VERTICES:Int = 2000000;
	static inline var MAX_TRIANGLES:Int = 4000000;
	static inline var MAX_FACE_RANGES:Int = 100000;

	public static function encode(data:SceneArtifactData):Bytes {
		validateHeader(data);
		var unit = data.lengthUnit == null ? LengthUnit.fromScale(data.metresPerUnit) : data.lengthUnit;
		var unitText = Bytes.ofString(unit);
		var names:Array<{id:Bytes, name:Bytes, finish:Bytes, materialId:Bytes, materialSpec:Bytes,
			density:Float, volume:Float, center:Array<Float>, inertia:Array<Float>, physicalMass:Float,
			physicalMassDeclared:Bool, physicalInertia:Array<Float>, faces:Bytes, threads:Bytes}> = [];
		var assemblyDefinition = data.assemblyDefinition == null ? Bytes.alloc(0)
			: Bytes.ofString(AssemblyDefinitionCodec.encode(data.assemblyDefinition));
		var assemblyState = data.assemblyState == null ? Bytes.alloc(0)
			: Bytes.ofString(AssemblyDefinitionCodec.encodeState(data.assemblyDefinition, data.assemblyState));
		var recipeDocument = data.recipeDocument == null ? Bytes.alloc(0) : Bytes.ofString(data.recipeDocument);
		var recipeDiagnostics = data.recipeDiagnostics == null ? Bytes.alloc(0)
			: Bytes.ofString(haxe.Json.stringify(data.recipeDiagnostics));
		if (assemblyDefinition.length > 2000000 || assemblyState.length > 2000000)
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
    var project = data.project == null ? Bytes.alloc(0) : Bytes.ofString(haxe.Json.stringify(data.project));
    if (project.length > 2000000) throw "Scene artifact project settings are too large";
    var authoredMotion = data.machineMotion;
    var machineMotion = authoredMotion == null ? Bytes.alloc(0) : Bytes.ofString(JsonWire.encode(authoredMotion));
    if (machineMotion.length > 1000000) throw "Scene artifact machine motion is too large";
		var controllers = data.controllers == null ? Bytes.alloc(0) : Bytes.ofString(haxe.Json.stringify(data.controllers));
		var signals = data.signals == null ? Bytes.alloc(0) : Bytes.ofString(haxe.Json.stringify(data.signals));
		if (controllers.length > 100000 || signals.length > 100000) throw "Scene artifact cell wiring is too large";
		var printing = data.printing == null ? Bytes.alloc(0) : Bytes.ofString(haxe.Json.stringify(data.printing));
		if (printing.length > 40000000) throw "Scene artifact print job is too large";
		var length = 36 + unitText.length + assemblyDefinition.length + assemblyState.length + recipeDocument.length + recipeDiagnostics.length + 4
			+ machining.length + 4 + mobileBase.length + 4 + mission.length + 4 + robotTools.length + 4 + robotSensors.length + 4 + machineMotion.length + 4
			+ controllers.length + 4 + signals.length + 4 + project.length + 4
			+ printing.length + 4;
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
			var volume = part.volume == null ? meshProperties.volume : part.volume;
			var physicalMass = part.physicalMassKg == null
				? volume * density * Math.pow(data.metresPerUnit, 3) : cast part.physicalMassKg;
			var physicalInertia = part.physicalInertiaKgM2 == null
				? [for (value in (part.inertia == null ? meshProperties.inertia : part.inertia))
					value * density * Math.pow(data.metresPerUnit, 5) *
					(physicalMass / (volume * density * Math.pow(data.metresPerUnit, 3)))]
				: part.physicalInertiaKgM2.copy();
			var faceDescriptors = part.faceDescriptors;
			var faces = faceDescriptors == null ? Bytes.alloc(0) : Bytes.ofString(faceDescriptors);
			var threads = part.threads == null || part.threads.length == 0 ? Bytes.alloc(0) : Bytes.ofString(JsonWire.encode(part.threads));
			names.push({id: id, name: name, finish: finish, materialId: materialId,
				materialSpec: materialSpec, density: density, faces: faces, threads: threads,
				volume: volume,
				center: part.centerOfMass == null ? meshProperties.centerOfMass : part.centerOfMass,
				inertia: part.inertia == null ? meshProperties.inertia : part.inertia,
				physicalMass: physicalMass,
				physicalMassDeclared: part.physicalMassDeclared == null ? part.physicalMassKg != null : part.physicalMassDeclared,
				physicalInertia: physicalInertia});
			length += 249 + finish.length + materialId.length + materialSpec.length + id.length + name.length + part.vertices.length + part.normals.length
				+ part.indices.length + (part.edgeSegments == null ? 0 : part.edgeSegments.length)
				+ (part.edgeIds == null ? 0 : part.edgeIds.length)
				+ part.faceRanges.length * 12 + 4 + faces.length + 4 + threads.length;
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
			result.setDouble(offset, text.physicalMass); offset += 8;
			result.set(offset++, text.physicalMassDeclared ? 1 : 0);
			for (component in text.physicalInertia) { result.setDouble(offset, component); offset += 8; }
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
			offset = putInt(result, offset, text.threads.length);
			result.blit(offset, text.threads, 0, text.threads.length); offset += text.threads.length;
		}
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
		offset = putInt(result, offset, machineMotion.length);
		result.blit(offset, machineMotion, 0, machineMotion.length); offset += machineMotion.length;
		offset = putInt(result, offset, controllers.length);
		result.blit(offset, controllers, 0, controllers.length); offset += controllers.length;
		offset = putInt(result, offset, signals.length);
		result.blit(offset, signals, 0, signals.length); offset += signals.length;
    offset = putInt(result, offset, project.length);
    result.blit(offset, project, 0, project.length); offset += project.length;
		offset = putInt(result, offset, printing.length);
		result.blit(offset, printing, 0, printing.length); offset += printing.length;
		if (offset != result.length) throw "Scene artifact size mismatch";
		return result;
	}

  /** Decode the portable project settings without retaining dynamic JSON object graphs. */
  public static function decodeProject(raw:Dynamic):SceneArtifactProject {
    var parts:Dynamic = Reflect.field(raw, "dynamicParts"), motions:Dynamic = Reflect.field(raw, "motions");
    var job:Dynamic = Reflect.field(raw, "job");
    if (!Std.isOfType(parts, Array) || !Std.isOfType(motions, Array) ||
        (job != null && (!Std.isOfType(job, String) || job.length == 0 || job.length > 500)))
      throw "Invalid artifact project settings";
    var dynamicParts:Array<String> = [];
    for (id in (cast parts:Array<Dynamic>)) {
      if (!Std.isOfType(id, String)) throw "Invalid project dynamic part";
      dynamicParts.push(id);
    }
    var tracks:Array<SceneArtifactProjectMotion> = [];
    for (motion in (cast motions:Array<Dynamic>)) {
      var joint:Dynamic = Reflect.field(motion, "joint"), loop:Dynamic = Reflect.field(motion, "loop");
      if (!Std.isOfType(joint, String) || !Std.isOfType(loop, Bool) || !Std.isOfType(Reflect.field(motion, "keys"), Array))
        throw "Invalid project motion";
      var keys:Array<{time:Float, position:Float}> = [];
      for (key in (cast Reflect.field(motion, "keys"):Array<Dynamic>)) {
        var time:Dynamic = Reflect.field(key, "time"), position:Dynamic = Reflect.field(key, "position");
        for (value in [time, position])
          if (!Std.isOfType(value, Int) && !Std.isOfType(value, Float)) throw "Invalid project motion key";
        keys.push({time: time, position: position});
      }
      tracks.push({joint: joint, loop: loop, keys: keys});
    }
    return {job: job, dynamicParts: dynamicParts, motions: tracks};
  }

  static function validateProject(project:SceneArtifactProject, data:SceneArtifactData):Void {
    decodeProject(project);
    if (project.dynamicParts.length > 1000 || project.motions.length > 64) throw "Too many project settings";
    var definition = data.assemblyDefinition == null ? null : AssemblyDefinitionFlattener.flattenView(data.assemblyDefinition);
    var occurrences = definition == null ? [for (part in data.parts) part.id] : [for (item in definition.occurrences) item.id];
    var joints = definition == null ? [] : [for (item in definition.joints) item.id];
    var seen = new Map<String, Bool>();
    for (id in project.dynamicParts) {
      if (occurrences.indexOf(id) < 0 || seen.exists(id)) throw "Invalid project dynamic part";
      seen.set(id, true);
    }
    seen = new Map();
    for (motion in project.motions) {
      if (joints.indexOf(motion.joint) < 0 || seen.exists(motion.joint) || motion.keys.length < 2 || motion.keys.length > 10000)
        throw "Invalid project joint motion";
      seen.set(motion.joint, true);
      var previous = -1.0;
      for (key in motion.keys) {
        if (!Math.isFinite(key.time) || !Math.isFinite(key.position) || key.time < 0 || key.time <= previous ||
            (previous < 0 && key.time != 0)) throw "Invalid project motion key";
        previous = key.time;
      }
      if (motion.loop && Math.abs(motion.keys[0].position - motion.keys[motion.keys.length - 1].position) > 1e-9)
        throw "Project motion loop must return to its starting position";
    }
  }

  static function validateMachineMotion(motion:SceneArtifactMachineMotion, data:SceneArtifactData):Void {
    var source = data.assemblyDefinition;
    if (source == null) throw "Machine motion needs an assembly";
    if (motion.belts == null || motion.belts.length > 128)
      throw "Machine motion needs a bounded belt list";
    var definition = AssemblyDefinitionFlattener.flattenView(source);
    var occurrences = [for (value in definition.occurrences) value.id];
    var joints = [for (value in definition.joints) value.id];
    for (belt in motion.belts) {
      if (occurrences.indexOf(belt.occurrence) < 0 || joints.indexOf(belt.driverJoint) < 0 ||
          belt.wraps == null || belt.wraps.length < 2 || belt.wraps.length > 128 ||
          belt.driverWrap < 0 || belt.driverWrap >= belt.wraps.length ||
          (belt.driverSign != 1.0 && belt.driverSign != -1.0))
        throw "Invalid moving belt attachments";
      for (value in [belt.pitch, belt.width, belt.thickness])
        if (!Math.isFinite(value) || value <= 0.0) throw "Invalid moving belt dimensions";
      for (wrap in belt.wraps) {
        if (occurrences.indexOf(wrap.occurrence) < 0 || wrap.connector == null || wrap.connector.length == 0 ||
            !Math.isFinite(wrap.radius) || wrap.radius <= 0.0 || (wrap.side != 1 && wrap.side != -1))
          throw "Invalid moving belt wrap";
        var occurrence = [for (value in definition.occurrences) if (value.id == wrap.occurrence) value][0];
        var found = false;
        for (component in definition.definitions) if (component.id == occurrence.definition)
          for (connector in component.connectors) if (connector.name == wrap.connector) found = true;
        if (!found) throw "Moving belt wrap connector is missing";
      }
    }
    var contacts = motion.contacts;
    if (contacts != null) {
      if (contacts.length > 1000) throw "Machine contact surface list is too large";
      var seen = new Map<String, Bool>();
      for (contact in contacts) {
        if (occurrences.indexOf(contact.occurrence) < 0 || seen.exists(contact.occurrence))
          throw "Machine contact surfaces need distinct known occurrences";
        seen.set(contact.occurrence, true);
        var surface = contact.surface;
        if (surface == null || surface.friction == null || surface.friction.length != 3 || surface.filter == null)
          throw "Machine contact surface needs friction and a filter";
        for (value in surface.friction)
          if (!finite(value) || value < 0) throw "Machine contact friction must be finite and non-negative";
        if (surface.frictionDimensions != 0 && surface.frictionDimensions != 1 && surface.frictionDimensions != 3 &&
            surface.frictionDimensions != 4 && surface.frictionDimensions != 6)
          throw "Machine contact friction dimensions must be 0, 1, 3, 4 or 6";
        for (value in [surface.contactTimeConstant, surface.contactDampingRatio])
          if (!finite(value) || value < 0) throw "Machine contact compliance must be finite and non-negative";
      }
    }
    var press = motion.pressBrake;
    if (press != null) {
      if (motion.digitalProgram != null || motion.virtualDevice == true)
        throw "Press controller cannot share timed outputs or a virtual axis device";
      var owned = [press.ramJoint, press.depthAxis, press.gaugeX];
      if (press.gaugeR != null) owned.push(press.gaugeR);
      var seen = new Map<String, Bool>();
      for (id in owned) {
        if (id == null || joints.indexOf(id) < 0 || seen.exists(id))
          throw "Press controller needs distinct known ram and positioning axes";
        for (joint in definition.joints) if (joint.id == id && joint.type != materia.assembly.AssemblyDefinition.AssemblyJointType.Prismatic)
          throw 'Press controller axis "$id" must have linear travel';
        seen.set(id, true);
      }
      if (press.fingerSwitches == null || press.fingerSwitches.length != 2 ||
          press.fingerSwitches[0] == press.fingerSwitches[1] ||
          press.fingerSwitches.indexOf(press.topSwitch) >= 0)
        throw "Press controller needs distinct top and two finger switches";
      for (id in [press.topSwitch].concat(press.fingerSwitches)) {
        var found = false;
        var sensors = definition.sensors;
        if (sensors != null) for (sensor in sensors) if (sensor.id == id && sensor.kind == "joint_switch") found = true;
        if (!found) throw 'Press controller switch "$id" is missing';
      }
      for (value in [press.positionToleranceMm, press.requestTimeoutSeconds, press.releaseTravelMm, press.releaseClosingForceN])
        if (!finite(value) || value <= 0) throw "Press controller needs positive finite tolerances and unloading limits";
      if (press.steps == null || press.steps.length == 0 || press.steps.length > 10000)
        throw "Press controller needs a bounded nonempty program";
      function pressTravel(id:String, millimetres:Float):Void {
        for (joint in definition.joints) if (joint.id == id) {
          var bounds = joint.limits;
          var coordinate = millimetres * 0.001 / data.metresPerUnit;
          if (bounds == null || bounds.lower == null || bounds.upper == null ||
              coordinate < bounds.lower || coordinate > bounds.upper)
            throw 'Press program exceeds bounded travel of axis "$id"';
        }
      }
      for (step in press.steps) {
        for (value in [step.gaugeXmm, step.gaugeRmm, step.clampTravelMm, step.depthTravelMm, step.withdrawalMm])
          if (!finite(value) || value < 0) throw "Press program coordinates must be finite and nonnegative";
        if (step.depthTravelMm <= step.clampTravelMm || step.clampTravelMm <= press.positionToleranceMm ||
            press.releaseTravelMm >= step.depthTravelMm - step.clampTravelMm || step.withdrawalMm <= 0 ||
            (press.gaugeR == null && step.gaugeRmm != 0))
          throw "Press program needs ordered clamp/depth travel, withdrawal and supported R";
        pressTravel(press.ramJoint, step.clampTravelMm);
        pressTravel(press.ramJoint, step.depthTravelMm);
        pressTravel(press.depthAxis, step.depthTravelMm);
        pressTravel(press.gaugeX, step.gaugeXmm);
        pressTravel(press.gaugeX, step.gaugeXmm + step.withdrawalMm);
        if (press.gaugeR != null) pressTravel(press.gaugeR, step.gaugeRmm);
      }
      var controls = data.controllers;
      if (controls != null && controls.length > 0) {
        var matches = [for (control in controls) if (control.id == press.controllerId) control];
        if (matches.length != 1) throw "Press controller binding is missing";
        for (id in owned) if (matches[0].joints.indexOf(id) < 0) throw 'Press controller does not own joint "$id"';
        for (id in [press.topSwitch].concat(press.fingerSwitches))
          if (matches[0].sensors.indexOf(id) < 0) throw 'Press controller does not own switch "$id"';
      } else if (press.controllerId != null) throw "Press controller binding names an absent cabinet";
    }
    var digital = motion.digitalProgram;
    if (digital != null) {
      if (motion.program != null || motion.virtualDevice == true || data.machining != null || data.mission != null)
        throw "Machine digital program needs exclusive execution ownership";
      if (!finite(digital.durationSec) || digital.durationSec <= 0 || digital.durationSec * 1e9 > 9007199254740991.0 ||
          digital.events == null || digital.events.length == 0 || digital.events.length > 10000)
        throw "Machine digital program needs a bounded duration and output list";
      var previous = 0.0;
      var times = new Map<String, Float>();
      for (event in digital.events) {
        if (!finite(event.timeSec) || event.timeSec < previous || event.timeSec >= digital.durationSec ||
            event.channel == null || event.channel.length == 0)
          throw "Machine digital events need ordered times before the end and a channel";
        var last = times.get(event.channel);
        if (last != null && last == event.timeSec) throw "Machine digital output is assigned twice at one time";
        times.set(event.channel, event.timeSec);
        previous = event.timeSec;
      }
    }
    var program = motion.program;
    if (motion.virtualDevice == true && program == null) throw "A virtual machine device needs an axis program";
    if (program != null) {
      if (program.positionTolerance != null && (!Math.isFinite(program.positionTolerance) || program.positionTolerance <= 0.0))
        throw "Machine program position tolerance must be finite and positive";
      if (program.axes == null || program.axes.length < 1 || program.axes.length > 64 ||
          program.waypoints == null || program.waypoints.length < 1 || program.waypoints.length > 10000)
        throw "Invalid machine axis program";
      var seen = new Map<String, Bool>();
      for (id in program.axes) {
        if (joints.indexOf(id) < 0 || seen.exists(id)) throw "Machine program needs distinct known axes";
        seen.set(id, true);
      }
      for (point in program.waypoints) {
        if (point == null || point.length != program.axes.length) throw "Machine waypoint joint count mismatch";
        for (value in point) if (!Math.isFinite(value)) throw "Machine waypoint must be finite";
      }
    }
  }

	public static function decode(source:Bytes):SceneArtifactData {
		if (source == null || source.length > MAX_BYTES) throw "Scene artifact is missing or too large";
		return new SceneArtifactReader(source).read();
	}

	/** Decode while borrowing mesh buffers from source. Keep source immutable while using the result.
	 * Views retain the source buffer; decode() still returns independent mesh buffers. */
	public static function decodeView(source:Bytes):SceneArtifactData {
		if (source == null || source.length > MAX_BYTES) throw "Scene artifact is missing or too large";
		return new SceneArtifactReader(source, true).read();
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
		if (data.assemblyState != null && data.assemblyDefinition == null)
			throw "Scene artifact assembly state has no definition";
		var assemblyDefinition = data.assemblyDefinition;
		if (assemblyDefinition != null) {
			AssemblyDefinitionCodec.validate(assemblyDefinition);
			assemblyDefinition = AssemblyDefinitionFlattener.flattenView(assemblyDefinition);
			for (definition in assemblyDefinition.definitions) if (!ids.exists(definition.id))
				throw 'Assembly component definition "${definition.id}" has no geometry part';
			if (data.assemblyState != null)
				AssemblyDefinitionCodec.validateState(assemblyDefinition, data.assemblyState);
		}
		if (data.project != null) validateProject(data.project, data);
		if (data.machining != null && data.mission != null &&
			(data.machining.controllerId == null || data.mission.controllerId == null ||
			 data.machining.controllerId == data.mission.controllerId))
			throw "Scene artifact CNC job and mission need distinct controller owners";
		if (data.printing != null && (data.machining != null || data.mission != null))
			throw "Scene artifact cannot combine a print job with machining or a mission";
		if (data.printing != null) validatePrinting(data.printing, data);
		if (data.machining != null) validateMachining(data.machining, ids, data.assemblyDefinition);
		if (data.mobileBase != null) validateMobileBase(data.mobileBase, data.assemblyDefinition);
		if (data.robotTools != null) validateRobotTools(data.robotTools, data);
		if (data.robotSensors != null) validateRobotSensors(data.robotSensors, data);
    if (data.machineMotion != null) validateMachineMotion(data.machineMotion, data);
		if (data.controllers != null || data.signals != null) validateCell(data);
		if (data.mission != null) validateMission(data.mission, data);
	}

	static function validateCell(data:SceneArtifactData):Void {
		var controllers = data.controllers;
		if (controllers == null || controllers.length == 0 || controllers.length > 32 || data.assemblyDefinition == null)
			throw "Scene artifact cell needs controllers and an assembly";
		var flat = AssemblyDefinitionFlattener.flatten(data.assemblyDefinition);
		var validJoints = new Map<String, Bool>();
		for (joint in flat.joints) if (Std.string(joint.role) == "tree" && Std.string(joint.type) != "fixed")
			validJoints.set(joint.id, true);
		var validSensors = new Map<String, Bool>();
		if (flat.sensors != null) for (sensor in flat.sensors) validSensors.set(sensor.id, true);
		var owners = new Map<String, String>();
		var sensorOwners = new Map<String, String>();
		var cabinetIds = new Map<String, Bool>();
		var outputs = new Map<String, Array<String>>();
		for (cabinet in controllers) {
			if (cabinet == null || cabinet.id == null || cabinet.id.length == 0 || cabinetIds.exists(cabinet.id) ||
				(cabinet.kind != "cnc" && cabinet.kind != "robot") || cabinet.joints == null ||
				cabinet.sensors == null || cabinet.channels == null)
				throw "Scene artifact cell has an invalid controller";
			cabinetIds.set(cabinet.id, true);
			var channelIds = new Map<String, Bool>();
			for (channel in cabinet.channels) {
				if (channel == null || channel.length == 0 || channelIds.exists(channel))
					throw 'Scene artifact controller "${cabinet.id}" has a duplicate or empty channel';
				channelIds.set(channel, true);
			}
			outputs.set(cabinet.id, cabinet.channels);
			for (joint in cabinet.joints) {
				if (!validJoints.exists(joint) || owners.exists(joint))
					throw 'Scene artifact cell joint "$joint" has no unique owner';
				owners.set(joint, cabinet.id);
			}
			for (sensor in cabinet.sensors) {
				if (!validSensors.exists(sensor) || sensorOwners.exists(sensor))
					throw 'Scene artifact cell sensor "$sensor" has no unique owner';
				sensorOwners.set(sensor, cabinet.id);
			}
		}
		for (joint in validJoints.keys()) if (!owners.exists(joint))
			throw 'Scene artifact cell joint "$joint" has no controller';
		if (flat.couplings != null) for (coupling in flat.couplings)
			if (owners.get(coupling.source) != owners.get(coupling.target))
				throw 'Scene artifact coupling "${coupling.id}" crosses controllers';
		if (flat.sensors != null) for (sensor in flat.sensors) {
			var owner = sensorOwners.get(sensor.id);
			if (owner == null) throw 'Scene artifact cell sensor "${sensor.id}" has no controller';
			if (sensor.joint != null && owners.exists(sensor.joint) && owners.get(sensor.joint) != owner)
				throw 'Scene artifact sensor "${sensor.id}" is wired to "$owner" but observes joint "${sensor.joint}" owned by "${owners.get(sensor.joint)}"';
		}
		if (flat.actuators != null) for (actuator in flat.actuators) {
			var cabinet = owners.get(actuator.joint), channels = cabinet == null ? null : outputs.get(cabinet);
			function requireChannel(channel:String):Void {
				if (channels == null || channels.indexOf(channel) < 0)
					throw 'Scene artifact actuator "${actuator.id}" has no output "$channel" on its joint owner';
			}
			var pneumatic = actuator.pneumatic;
			if (pneumatic != null) {
				requireChannel(pneumatic.channelA);
				if (pneumatic.channelB != null) requireChannel(pneumatic.channelB);
			}
			var speed = actuator.processVelocity;
			if (speed != null) {requireChannel(speed.speedChannel); requireChannel(speed.directionChannel);}
			var brake = actuator.brake;
			if (brake != null) requireChannel(brake.releaseChannel);
		}
		var signalIds = new Map<String, Bool>(), destinations = new Map<String, Bool>();
		if (data.signals != null) for (signal in data.signals) {
			if (signal == null || signal.name == null || signal.name.length == 0 || signal.from == null || signal.to == null ||
				!cabinetIds.exists(signal.from.controller) || !cabinetIds.exists(signal.to.controller) ||
				signal.from.port == null || signal.from.port.length == 0 || signal.to.port == null || signal.to.port.length == 0 || signalIds.exists(signal.name) ||
				destinations.exists(signal.to.controller + "/" + signal.to.port))
				throw "Scene artifact cell has an invalid signal";
			signalIds.set(signal.name, true);
			destinations.set(signal.to.controller + "/" + signal.to.port, true);
			var sourceChannels = outputs.get(signal.from.controller);
			if (sourceChannels == null || sourceChannels.indexOf(signal.name) < 0)
				throw 'Scene artifact signal "${signal.name}" has no source output channel';
		}
		if (data.machining != null && (data.machining.controllerId == null ||
			[for (c in controllers) if (c.id == data.machining.controllerId && c.kind == "cnc") c].length != 1))
			throw "Scene artifact machining job needs its CNC controller";
		var machining = data.machining;
		if (machining != null) for (axis in machining.axes)
			if (owners.get(axis) != machining.controllerId)
				throw 'Scene artifact CNC axis "$axis" belongs to a different controller';
		var motion = data.machineMotion;
		if (motion != null && motion.program != null) {
			var axes = motion.program.axes;
			if (axes.length == 0 || !owners.exists(axes[0])) throw "Scene artifact axis program has no controller";
			for (axis in axes) if (owners.get(axis) != owners.get(axes[0]))
				throw 'Scene artifact axis program crosses controllers at "$axis"';
		}
		var printing = data.printing;
		if (printing != null) for (axis in printing.axes) if (owners.get(axis) != owners.get(printing.extruder))
			throw 'Scene artifact print axis "$axis" and the extruder belong to different controllers';
		if (data.mission != null && (data.mission.controllerId == null ||
			[for (c in controllers) if (c.id == data.mission.controllerId && c.kind == "robot") c].length != 1))
			throw "Scene artifact mission needs its robot controller";
	}

	static function decodeControllers(raw:Dynamic):Array<SceneArtifactCellController> {
		function fail():Dynamic throw "Scene artifact controllers are invalid";
		function name(value:Dynamic):String return Std.isOfType(value, String) ? value : fail();
		function names(value:Dynamic):Array<String> {
			if (!Std.isOfType(value, Array)) fail();
			return [for (item in (cast value:Array<Dynamic>)) name(item)];
		}
		if (!Std.isOfType(raw, Array)) fail();
		return [for (item in (cast raw:Array<Dynamic>)) {
			id: name(Reflect.field(item, "id")), kind: name(Reflect.field(item, "kind")),
			joints: names(Reflect.field(item, "joints")), sensors: names(Reflect.field(item, "sensors")),
			channels: names(Reflect.field(item, "channels"))
		}];
	}

	static function decodeSignals(raw:Dynamic):Array<SceneArtifactCellSignal> {
		function fail():Dynamic throw "Scene artifact signals are invalid";
		function name(value:Dynamic):String return Std.isOfType(value, String) ? value : fail();
		function port(value:Dynamic):SceneArtifactSignalPort {
			if (value == null) fail();
			return {controller: name(Reflect.field(value, "controller")), port: name(Reflect.field(value, "port"))};
		}
		if (!Std.isOfType(raw, Array)) fail();
		return [for (item in (cast raw:Array<Dynamic>)) {
			name: name(Reflect.field(item, "name")), from: port(Reflect.field(item, "from")),
			to: port(Reflect.field(item, "to"))
		}];
	}

	static function validateRobotTools(tools:Array<SceneArtifactRobotTool>, data:SceneArtifactData):Void {
		function fail(detail:String):Void throw 'Scene artifact robot tool $detail';
		var definition = data.assemblyDefinition;
		if (definition == null) fail("needs the robot's assembly definition");
		var flat = AssemblyDefinitionFlattener.flattenView(cast definition);
		var channels = new Map<String, Bool>();
		var torches = 0, drivers = 0;
		for (tool in tools) {
			if (tool == null || (tool.kind != "suction" && tool.kind != "gripper" && tool.kind != "torch" && tool.kind != "screwdriver"))
				fail('kind "${tool == null ? null : tool.kind}" is unknown');
			var contact = tool.contact;
			var occurrence = contact == null ? [] : [for (item in flat.occurrences) if (item.id == contact.occurrence) item];
			if (occurrence.length != 1) fail("touches with no occurrence of the assembly");
			var found = false;
			for (component in flat.definitions) if (component.id == occurrence[0].definition)
				for (connector in component.connectors) if (connector.name == contact.connector) found = true;
			if (!found) fail('has no contact connector "${contact.connector}" on "${contact.occurrence}"');
			if (tool.members == null || tool.members.length == 0 || tool.members.indexOf(contact.occurrence) < 0)
				fail("needs its exact CAD tool members, including the contact occurrence");
			var memberIds = new Map<String, Bool>();
			for (member in tool.members) {
				if (memberIds.exists(member) || [for (item in flat.occurrences) if (item.id == member) item].length != 1)
					fail('has an invalid or repeated CAD tool member "$member"');
				memberIds.set(member, true);
			}
			if (tool.channel == null || tool.channel.length == 0 || channels.exists(tool.channel))
				fail("needs a channel of its own");
			channels.set(tool.channel, true);
			if (tool.sensor != null && tool.sensor.length == 0) fail("has an empty sensor name");
			if (tool.kind != "screwdriver" && tool.screwdriver != null) fail("has screwdriver data on another tool kind");
			if (tool.kind == "screwdriver") {
				function driverProblem(detail:String):Void fail('"${contact.occurrence}" $detail');
				if (++drivers > 1 || tool.torch != null || tool.gripper != null || tool.sensor == null)
					driverProblem("needs one screwdriver, a sensor and no other process description");
				var driver = tool.screwdriver;
				if (driver == null) { driverProblem("has no screwdriver description"); return; }
				if (driver.bit == null || ["hex-socket", "hexalobular", "cross", "hex-head", "slot"].indexOf(driver.bit.kind) < 0 ||
					driver.bit.size == null || driver.bit.size.length == 0) driverProblem("has an invalid bit drive");
				if (driver.bit.kind != "hexalobular") {
					var size = Std.parseFloat(driver.bit.size);
					if (!finite(size) || Std.string(size) != driver.bit.size || (driver.bit.kind == "cross" ? size < 0 || size != Math.floor(size) : size <= 0))
						driverProblem("has an invalid numeric bit size");
				}
				for (value in [driver.maxTorque, driver.maxSpeed, driver.strokeMm, driver.springRate])
					if (!finite(value) || value <= 0) driverProblem("needs finite positive capacity, stroke and spring rate");
				for (name in [driver.directionChannel, driver.programChannel, driver.vacuumChannel]) {
					if (name == null || name.length == 0 || channels.exists(name)) driverProblem("needs distinct driver channels");
					channels.set(name, true);
				}
				var feed = [for (joint in flat.joints) if (joint.id == driver.feedJoint && joint.type == "prismatic") joint];
				var spindle = [for (joint in flat.joints) if (joint.id == driver.spindleJoint && joint.type == "continuous") joint];
				if (feed.length != 1 || spindle.length != 1 || spindle[0].child != contact.occurrence || spindle[0].parent != feed[0].child)
					driverProblem("needs its real feed and spindle joints on the contact chain");
				var limits = feed[0].limits;
				var millimetresPerUnit = LengthUnit.metresPerUnit(flat.lengthUnit == null ? "mm" : flat.lengthUnit) * 1000;
				if (limits == null || limits.lower == null || limits.upper == null ||
					Math.abs(((cast limits.upper:Float) - (cast limits.lower:Float)) * millimetresPerUnit - driver.strokeMm) > 1e-6)
					driverProblem("feed limits disagree with its stated stroke");
				var rotation = spindle[0].limits;
				if (rotation == null || rotation.velocity == null || rotation.effort == null ||
					Math.abs((cast rotation.velocity:Float) - driver.maxSpeed) > 1e-6 ||
					Math.abs((cast rotation.effort:Float) - driver.maxTorque) > 1e-6)
					driverProblem("spindle limits disagree with its stated capacity");
				if (limits.effort == null || Math.abs((cast limits.effort:Float) - driver.springRate * driver.strokeMm) > 1e-6)
					driverProblem("feed effort disagrees with its spring at full stroke");
				var encoders = flat.encoders == null ? [] : flat.encoders;
				if ([for (encoder in encoders) if (encoder.id == driver.strokeEncoder && encoder.joint == driver.feedJoint) encoder].length != 1)
					driverProblem("needs its feed stroke encoder");
				continue;
			}
			if (tool.kind == "suction") {
				if (tool.torch != null) fail("is a suction tool with a torch description");
				continue;
			}
			if (tool.kind == "gripper") {
				if (tool.torch != null) fail("is a gripper with a torch description");
				if (tool.sensor == null) fail("has no jaw-state sensor");
				var gripper = tool.gripper;
				if (gripper == null) {
					fail("is a gripper with no jaw description");
					return;
				}
				if (!finite(gripper.minimumPartWidthMm) ||
					!finite(gripper.nominalPartWidthMm) || !finite(gripper.maximumPartWidthMm) ||
					!finite(gripper.fingerLengthMm) || !finite(gripper.fingerPadFrictionCoefficient) ||
					!(gripper.fingerPadFrictionCoefficient > 0) || !(gripper.minimumPartWidthMm > 0) ||
					gripper.minimumPartWidthMm > gripper.nominalPartWidthMm ||
					gripper.nominalPartWidthMm > gripper.maximumPartWidthMm || !(gripper.fingerLengthMm > 0))
					fail("has invalid jaw width or finger reach data");
				if (gripper.padMembers == null || gripper.padMembers.length == 0 ||
					gripper.envelopeMembers == null || gripper.envelopeMembers.length == 0)
					fail("needs named CAD pads and its gripper envelope");
				for (member in gripper.envelopeMembers) if (!memberIds.exists(member))
					fail('gripper envelope member "$member" is outside its mounted tool');
				for (member in gripper.padMembers) if (gripper.envelopeMembers.indexOf(member) < 0)
					fail('gripper pad "$member" is outside its envelope');
				var sensors = flat.sensors == null ? [] : flat.sensors;
				var foundSwitch = [for (sensor in sensors) if (sensor.id == tool.sensor && sensor.kind == "joint_switch") sensor];
				if (foundSwitch.length != 1) fail('has no physical joint switch "${tool.sensor}"');
				var actuators = flat.actuators == null ? [] : flat.actuators;
				var cylinders = [for (actuator in actuators) if (actuator.joint == foundSwitch[0].joint &&
					actuator.pneumatic != null && actuator.pneumatic.channelB != null) actuator];
				if (cylinders.length != 1)
					fail('has no unique double-solenoid cylinder on jaw switch joint "${foundSwitch[0].joint}"');
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
		var flat = AssemblyDefinitionFlattener.flattenView(cast definition);
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



	/** A print job from its JSON section, typed field by field. */
	static function decodePrinting(decoded:Dynamic):SceneArtifactPrinting {
		function fail():Dynamic throw "Scene artifact print job is invalid";
		function array(value:Dynamic):Array<Dynamic> return Std.isOfType(value, Array) ? cast value : fail();
		function number(value:Dynamic):Float return Std.isOfType(value, Float) || Std.isOfType(value, Int) ? (value:Float) : fail();
		function field(value:Dynamic, name:String):Float return number(Reflect.field(value, name));
		function text(value:Dynamic, name:String):String {
			var item:Dynamic = Reflect.field(value, name);
			return Std.isOfType(item, String) ? item : fail();
		}
		function flag(value:Dynamic, name:String):Bool {
			var item:Dynamic = Reflect.field(value, name);
			return Std.isOfType(item, Bool) ? item : fail();
		}
		function integer(value:Dynamic, name:String):Int {
			var result = field(value, name);
			return result == Math.floor(result) ? Std.int(result) : fail();
		}
		var controller:Dynamic = Reflect.field(decoded, "controller");
		if (controller == null) fail();
		var job:SceneArtifactPrinting = {source: text(decoded, "source"),
			axes: [for (axis in array(Reflect.field(decoded, "axes"))) Std.isOfType(axis, String) ? (axis:String) : fail()],
			extruder: text(decoded, "extruder"),
			filamentDiameter: field(decoded, "filamentDiameter"), gearRadius: field(decoded, "gearRadius"),
			maxFlow: field(decoded, "maxFlow"), pressureAdvance: field(decoded, "pressureAdvance"),
			pressureAdvanceWindow: field(decoded, "pressureAdvanceWindow"), meltLag: field(decoded, "meltLag"), rampLength: field(decoded, "rampLength"),
			minExtrusionTemperature: field(decoded, "minExtrusionTemperature"),
			heaters: [for (raw in array(Reflect.field(decoded, "heaters"))) {
				var heater:SceneArtifactHeater = {name: text(raw, "name"), sensor: text(raw, "sensor"), slot: integer(raw, "slot"),
					mount: text(raw, "mount"), target: text(raw, "target"), feedsExtruder: flag(raw, "feedsExtruder"),
					r0Ohm: field(raw, "r0Ohm"), t0Celsius: field(raw, "t0Celsius"), beta: field(raw, "beta"),
					pullupOhm: field(raw, "pullupOhm"), vRef: field(raw, "vRef"), kp: field(raw, "kp"), ki: field(raw, "ki"),
					kd: field(raw, "kd"), maxDuty: field(raw, "maxDuty"), maxTemperature: field(raw, "maxTemperature"),
					bandCelsius: field(raw, "bandCelsius"), pidRangeCelsius: field(raw, "pidRangeCelsius"),
					gainWindowSeconds: field(raw, "gainWindowSeconds"), gainMinCelsius: field(raw, "gainMinCelsius"),
					hysteresisCelsius: field(raw, "hysteresisCelsius"), hysteresisWindowSeconds: field(raw, "hysteresisWindowSeconds"),
					heatCapacity: field(raw, "heatCapacity"), power: field(raw, "power"), loss: field(raw, "loss"),
					fanLoss: field(raw, "fanLoss"), ambient: field(raw, "ambient")};
				var fan:Dynamic = Reflect.field(raw, "fan");
				if (fan != null) heater.fan = Std.isOfType(fan, String) ? fan : fail();
				heater;
			}],
			controller: {stepTickHz: integer(controller, "stepTickHz")}};
		var deposit:Dynamic = Reflect.field(decoded, "deposit");
		if (deposit != null) job.deposit = Std.isOfType(deposit, String) ? deposit : fail();
		var shaping:Dynamic = Reflect.field(decoded, "shaping");
		if (shaping != null)
			job.shaping = [for (raw in array(shaping)) {axis: text(raw, "axis"), kind: text(raw, "kind"), frequency: field(raw, "frequency"),
				damping: field(raw, "damping")}];
		return job;
	}

	static function validatePrinting(job:SceneArtifactPrinting, data:SceneArtifactData):Void {
		function fail(detail:String):Void throw 'Scene artifact print job $detail';
		if (data.assemblyDefinition == null) fail("needs the printer's assembly definition");
		var definition = AssemblyDefinitionFlattener.flattenView(cast data.assemblyDefinition);
		var joints = new Map<String, String>();
		for (joint in definition.joints) joints.set(joint.id, Std.string(joint.type));
		var occurrences = [for (item in definition.occurrences) item.id];
		if (job.source == null || job.source.length == 0) fail("has no program");
		if (job.axes == null || job.axes.length != 3) fail("needs three axes");
		for (index in 0...3) {
			var axis = job.axes[index];
			if (!joints.exists(axis) || joints.get(axis) != "prismatic" || job.axes.indexOf(axis) != index)
				fail('axis "$axis" is not a distinct prismatic joint');
		}
		if (!joints.exists(job.extruder) || job.axes.indexOf(job.extruder) >= 0) fail('extruder "${job.extruder}" is not a joint of its own');
		for (value in [job.filamentDiameter, job.gearRadius, job.maxFlow, job.rampLength, job.pressureAdvanceWindow])
			if (!finite(value) || value <= 0.0) fail("has a filament, gear, flow, ramp or window size that is not positive");
		if (!finite(job.pressureAdvance) || job.pressureAdvance < 0.0 || job.pressureAdvance > 1.0) fail("has a pressure advance outside 0 to 1 s");
		if (!finite(job.meltLag) || job.meltLag < 0.0 || job.meltLag > 1.0) fail("has a melt lag outside 0 to 1 s");
		if (!finite(job.minExtrusionTemperature) || job.minExtrusionTemperature < 0.0) fail("has a minimum extrusion temperature below 0");
		if (job.controller == null || job.controller.stepTickHz <= 0) fail("needs a controller step rate");
		if (job.deposit != null && occurrences.indexOf(job.deposit) < 0) fail('shows what is printed on "${job.deposit}", which is no occurrence');
		var shaped = new Map<String, Bool>();
		if (job.shaping != null) for (shaper in job.shaping) {
			if (job.axes.indexOf(shaper.axis) < 0 || shaped.exists(shaper.axis)) fail('shapes axis "${shaper.axis}", which is not one of its axes or is shaped twice');
			shaped.set(shaper.axis, true);
			if (["zv", "mzv", "ei"].indexOf(shaper.kind) < 0) fail('has an input shaper of unknown kind "${shaper.kind}"');
			if (!finite(shaper.frequency) || shaper.frequency < 1.0 || shaper.frequency > 1000.0) fail("has an input shaper frequency outside 1 to 1000 Hz");
			if (!finite(shaper.damping) || shaper.damping < 0.0 || shaper.damping > 0.9) fail("has an input shaper damping outside 0 to 0.9");
		}
		if (job.heaters == null || job.heaters.length == 0 || job.heaters.length > 2) fail("needs one or two heaters");
		var slots = new Map<Int, Bool>(), channels = new Map<String, Bool>();
		for (heater in job.heaters) {
			if (heater.slot < 0 || heater.slot > 7 || slots.exists(heater.slot)) fail('heater "${heater.name}" has a bad or repeated sensor slot');
			slots.set(heater.slot, true);
			if (occurrences.indexOf(heater.mount) < 0) fail('heater "${heater.name}" is mounted on no occurrence');
			for (channel in [heater.target, heater.fan]) if (channel != null) {
				if (channel.length == 0 || channels.exists(channel)) fail('channel "$channel" is empty or repeated');
				channels.set(channel, true);
			}
			for (value in [heater.r0Ohm, heater.t0Celsius, heater.beta, heater.pullupOhm, heater.vRef, heater.kp, heater.ki, heater.kd,
					heater.maxDuty, heater.maxTemperature, heater.bandCelsius, heater.pidRangeCelsius, heater.gainWindowSeconds,
					heater.gainMinCelsius, heater.hysteresisCelsius, heater.hysteresisWindowSeconds, heater.heatCapacity, heater.power,
					heater.loss, heater.fanLoss, heater.ambient])
				if (!finite(value)) fail('heater "${heater.name}" has a value that is not a number');
		}
	}

	/** Robot tools from their JSON section, typed field by field. */
	static function decodeRobotTools(decoded:Dynamic):Array<SceneArtifactRobotTool> {
		function fail():Dynamic throw "Scene artifact robot tools are invalid";
		function text(value:Dynamic, name:String):String {
			var item:Dynamic = Reflect.field(value, name);
			return Std.isOfType(item, String) ? item : fail();
		}
		function members(value:Dynamic, name:String):Array<String> {
			var raw:Dynamic = Reflect.field(value, name);
			if (!Std.isOfType(raw, Array)) fail();
			return [for (item in (cast raw:Array<Dynamic>)) {
				if (!Std.isOfType(item, String)) fail();
				(item:String);
			}];
		}
		if (!Std.isOfType(decoded, Array)) fail();
		return [for (raw in (cast decoded:Array<Dynamic>)) {
			var contact:Dynamic = Reflect.field(raw, "contact");
			if (contact == null) fail();
			var tool:SceneArtifactRobotTool = {kind: text(raw, "kind"), members: members(raw, "members"),
				contact: {occurrence: text(contact, "occurrence"), connector: text(contact, "connector")},
				channel: text(raw, "channel")};
			if (Reflect.field(raw, "sensor") != null) tool.sensor = text(raw, "sensor");
			var gripper:Dynamic = Reflect.field(raw, "gripper");
			if (gripper != null) {
				function number(name:String):Float {
					var item:Dynamic = Reflect.field(gripper, name);
					return Std.isOfType(item, Float) || Std.isOfType(item, Int) ? (item:Float) : fail();
				}
				tool.gripper = {padMembers: members(gripper, "padMembers"), envelopeMembers: members(gripper, "envelopeMembers"),
					minimumPartWidthMm: number("minimumPartWidthMm"),
					nominalPartWidthMm: number("nominalPartWidthMm"),
					maximumPartWidthMm: number("maximumPartWidthMm"), fingerLengthMm: number("fingerLengthMm"),
					fingerPadFrictionCoefficient: number("fingerPadFrictionCoefficient")};
			}
			var driver:Dynamic = Reflect.field(raw, "screwdriver");
			if (driver != null) {
				function number(name:String):Float {
					var item:Dynamic = Reflect.field(driver, name);
					return Std.isOfType(item, Float) || Std.isOfType(item, Int) ? (item:Float) : fail();
				}
				var bit:Dynamic = Reflect.field(driver, "bit");
				if (bit == null) fail();
				tool.screwdriver = {bit: {kind: text(bit, "kind"), size: text(bit, "size")},
					directionChannel: text(driver, "directionChannel"), programChannel: text(driver, "programChannel"),
					vacuumChannel: text(driver, "vacuumChannel"), feedJoint: text(driver, "feedJoint"),
					spindleJoint: text(driver, "spindleJoint"), strokeEncoder: text(driver, "strokeEncoder"),
					maxTorque: number("maxTorque"), maxSpeed: number("maxSpeed"), strokeMm: number("strokeMm"), springRate: number("springRate")};
			}
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
		validatePowerUpOffsets(mission.powerUpOffsets, data.assemblyDefinition);
		validatePowerUpSideOffsets(mission.powerUpSideOffsets, data.assemblyDefinition);
		if (mission.steps == null || mission.steps.length == 0) fail("has no steps");
		var definition = data.assemblyDefinition;
		var flat = definition == null ? null : AssemblyDefinitionFlattener.flattenView(definition);
		var missionController:Null<SceneArtifactCellController> = null;
		if (mission.controllerId != null && data.controllers != null)
			for (controller in data.controllers) if (controller.id == mission.controllerId) missionController = controller;
		function requiresRobotController(index:Int):Void {
			if (mission.controllerId == null || missionController == null || missionController.kind != "robot")
				fail('step $index needs a robot mission controller');
		}
		function hasSignal(name:String, output:Bool):Bool {
			if (data.signals == null || mission.controllerId == null) return false;
			for (wire in data.signals) if (wire.name == name &&
				(output ? wire.from.controller == mission.controllerId : wire.to.controller == mission.controllerId)) return true;
			return false;
		}
		function hasInput(name:String):Bool {
			// A wait can observe either a local field sensor or a signal routed from
			// another controller. Both are inputs owned by this controller.
			return (missionController != null && missionController.sensors.indexOf(name) >= 0) || hasSignal(name, false);
		}
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
		var handlingTools:Array<SceneArtifactRobotTool> = data.robotTools == null ? [] : [for (tool in data.robotTools)
			if (tool.kind == "gripper" || tool.kind == "suction") tool];
		for (index in 0...mission.steps.length) {
			var step = mission.steps[index];
			if (step == null) fail('step $index is empty');
			if (step.configuration != null) {
				var pin:SceneArtifactArmConfiguration = step.configuration;
				if (step.kind != "pick" && step.kind != "place" && step.kind != "weld" &&
					step.kind != "moveJoints" && step.kind != "moveVia")
					fail('step $index cannot carry an arm configuration pin');
				if ((pin.shoulder != "front" && pin.shoulder != "back") ||
					(pin.elbow != "up" && pin.elbow != "down") || (pin.wrist != "no-flip" && pin.wrist != "flip"))
					fail('step $index has an invalid arm configuration');
				if (pin.turns != null && pin.turns.length != 6) fail('step $index configuration turns need six arm joints');
			}
			if (step.timeoutSeconds != null &&
				(!finite(step.timeoutSeconds) || step.timeoutSeconds <= 0 || step.timeoutSeconds > 3600))
				fail('step $index needs a finite timeout between 0 and 3600 seconds');
			if (step.approachDistance != null && step.kind != "pick" && step.kind != "place")
				fail('step $index can set a handling approach only for pick or place');
			if (step.approachDistance != null &&
				(!finite(step.approachDistance) || step.approachDistance <= 0 || step.approachDistance > 1))
				fail('step $index needs a finite handling approach between 0 and 1 metre');
			if (step.tool != null && step.kind != "pick" && step.kind != "place")
				fail('step $index can select a tool only for pick or place');
			if (step.toolYaw != null && step.kind != "pick" && step.kind != "place")
				fail('step $index can set a tool yaw only for pick or place');
			switch step.kind {
				case "goTo":
					if (data.mobileBase == null) fail('step $index drives, but the assembly has no mobile base');
					if (!finiteFloorPose(step.pose)) fail('step $index needs a finite pose');
				case "pick" | "place":
					handles = true;
					if (step.yaw != null && !finite(step.yaw)) fail('step $index needs a finite yaw');
					if (step.toolYaw != null && !finite(step.toolYaw)) fail('step $index needs a finite tool yaw');
					if (!exists(step.at)) fail('step $index names no connector of an occurrence in the assembly');
					if (handlingTools.length == 0) fail('step $index needs a handling tool');
					if (handlingTools.length > 1 && step.tool == null)
						fail('step $index must select a tool because this robot has multiple handling tools');
					if (step.tool != null && [for (tool in handlingTools) if (tool.channel == step.tool) tool].length != 1)
						fail('step $index names no unique handling tool channel "${step.tool}"');
				case "moveJoints":
					if (!exists(step.at)) fail('step $index names no connector of an occurrence in the assembly');
					if (step.joints == null || step.joints.length == 0 || step.joints.length > 64)
						fail('step $index needs between one and 64 joint targets');
					var seen = new Map<String, Bool>();
					for (target in step.joints) {
						if (target == null || !finite(target.position) || seen.exists(target.joint))
							fail('step $index has an invalid or repeated joint target');
						seen.set(target.joint, true);
						var mechanism:AssemblyDefinition = cast flat;
						var found = [for (joint in mechanism.joints) if (joint.id == target.joint) joint];
						if (found.length != 1 || Std.string(found[0].role) != "tree" ||
							(Std.string(found[0].type) != "revolute" && Std.string(found[0].type) != "continuous" && Std.string(found[0].type) != "prismatic"))
							fail('step $index names no movable joint "${target.joint}"');
						var joint = found[0];
						var scale = Std.string(joint.type) == "prismatic" ? data.metresPerUnit : 1.0;
						if ((joint.limits.lower != null && target.position < joint.limits.lower * scale) ||
							(joint.limits.upper != null && target.position > joint.limits.upper * scale))
							fail('step $index targets joint "${target.joint}" past its limits');
					}
				case "signal":
					requiresRobotController(index);
					if (step.signal == null || step.signal.name == null || step.signal.name.length == 0)
						fail('step $index needs a named digital output');
					else if (!hasSignal(step.signal.name, true))
						fail('step $index writes no output wire owned by controller "${mission.controllerId}"');
				case "waitFor":
					requiresRobotController(index);
					var condition = step.waitFor;
					if (condition == null || condition.name == null || condition.name.length == 0)
						fail('step $index needs a named digital input');
					else {
						if (!hasInput(condition.name))
							fail('step $index reads no input signal or sensor owned by controller "${mission.controllerId}"');
						if (!finite(condition.timeout) || condition.timeout <= 0 || condition.timeout > 3600)
							fail('step $index needs a finite timeout between 0 and 3600 seconds');
							if (condition.retryStep != null) {
								var retry = condition.retryStep;
							var retryCommand = retry >= 0 && retry < mission.steps.length ? mission.steps[retry] : null;
							if (retry < 0 || retry >= index || retryCommand == null || retryCommand.kind != "signal" ||
								retryCommand.signal == null)
								fail('step $index retryStep must name an earlier digital output command');
							else if (retryCommand.signal.value != true)
								fail('step $index retryStep must repeat an asserted output request');
						}
					}
				case "moveVia":
					requiresRobotController(index);
					if (!exists(step.at)) fail('step $index names no connector of an occurrence in the assembly');
					if (step.poses == null || step.poses.length == 0 || step.poses.length > 64)
						fail('step $index needs between one and 64 via-points');
					var jointSpace:Null<Bool> = null;
					for (waypoint in step.poses) {
						if (waypoint == null || (waypoint.joints == null) == (waypoint.pose == null)) {
							fail('step $index via-points need exactly one joint or pose target');
							continue;
						}
						var usesJoints = waypoint.joints != null;
						if (jointSpace != null && jointSpace != usesJoints)
							fail('step $index cannot mix joint and pose via-points');
						jointSpace = usesJoints;
						if (usesJoints) {
							var targets = waypoint.joints;
							if (targets == null || targets.length == 0 || targets.length > 64)
							fail('step $index joint via-point needs between one and 64 targets');
							var seen = new Map<String, Bool>();
							for (target in targets) {
								if (target == null || target.joint == null || !finite(target.position) || seen.exists(target.joint))
									fail('step $index has an invalid or repeated via-point joint');
								seen.set(target.joint, true);
								if (missionController == null || missionController.joints.indexOf(target.joint) < 0)
									fail('step $index targets joint "${target.joint}" outside its controller');
								var mechanism:AssemblyDefinition = cast flat;
								var found = [for (joint in mechanism.joints) if (joint.id == target.joint) joint];
								if (found.length != 1 || Std.string(found[0].role) != "tree" ||
									(Std.string(found[0].type) != "revolute" && Std.string(found[0].type) != "continuous" && Std.string(found[0].type) != "prismatic"))
									fail('step $index names no movable joint "${target.joint}"');
								else {
									var joint = found[0], scale = Std.string(joint.type) == "prismatic" ? data.metresPerUnit : 1.0;
									if ((joint.limits.lower != null && target.position < joint.limits.lower * scale) ||
										(joint.limits.upper != null && target.position > joint.limits.upper * scale))
										fail('step $index targets joint "${target.joint}" past its limits');
								}
							}
						} else {
							var pose = waypoint.pose;
							if (pose == null || pose.position == null || pose.position.length != 3 || pose.rotation == null || pose.rotation.length != 4)
								fail('step $index has a via pose without three position and four rotation values');
							else {
								for (value in pose.position.concat(pose.rotation)) if (!finite(value)) fail('step $index has a non-finite via pose');
								var norm = Math.sqrt(pose.rotation[0] * pose.rotation[0] + pose.rotation[1] * pose.rotation[1] +
									pose.rotation[2] * pose.rotation[2] + pose.rotation[3] * pose.rotation[3]);
								if (Math.abs(norm - 1) > 1e-3) fail('step $index has a via-pose rotation that is not a unit quaternion');
							}
						}
					}
				case "weld":
					welds = true;
					validateWeld(step.weld, index, flat, fail);
				case "findWork":
					welds = true;
					if (!exists(step.at)) fail('step $index names no contact connector');
					var contactTool = data.robotTools == null ? [] : [for (tool in data.robotTools)
						if (tool.kind == "torch" && step.at != null && tool.contact.occurrence == step.at.occurrence && tool.contact.connector == step.at.connector) tool];
					if (contactTool.length != 1) fail('step $index needs the torch contact connector');
					SceneContactRegistration.validate(cast step.contactWork, flat == null ? [] : [for (item in flat.occurrences) item.id]);
				case "stow":
					welds = true;
				default: fail('step $index has unknown kind "${step.kind}"');
			}
		}
		var suctions = data.robotTools == null ? 0 : [for (tool in data.robotTools) if (tool.kind == "suction") tool].length;
		var grippers = data.robotTools == null ? 0 : [for (tool in data.robotTools) if (tool.kind == "gripper") tool].length;
		if (handles && suctions + grippers == 0) fail("picks and places, but the robot has no suction or gripper tool");
		if (handles && suctions > 0 && grippers > 0)
			fail("picks and places with both suction and gripper tools; the robot must use one handling tool type");
		var torches = data.robotTools == null ? 0 : [for (tool in data.robotTools) if (tool.kind == "torch") tool].length;
		if (welds && torches != 1) fail('welds, but the robot has $torches torches, not one');
		var tools = data.robotTools;
		if (welds && torches == 1 && tools != null) {
			var feeder = [for (tool in tools) if (tool.kind == "torch") tool][0].torch;
			if (feeder != null) for (index in 0...mission.steps.length) {
				var weld = mission.steps[index].weld;
				if (weld != null && weld.passes != null) for (pass in weld.passes)
					if (pass.process.wireSpeed > feeder.maxWireSpeedMPerMin)
						fail('step $index pass "${pass.name}" needs ${pass.process.wireSpeed} m/min of wire, past the feeder\'s top speed of ${feeder.maxWireSpeedMPerMin} m/min');
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
			if (segment.seam == null || segment.seam.length == 0) fail('step $index$label needs a seam name');
			if (segment.faces == null || segment.faces.length != 2) {
				fail('step $index$label needs two CAD seam faces');
				return;
			}
			for (face in segment.faces) {
				if (face == null || face.face == null || face.face.length == 0) {
					fail('step $index$label needs a named CAD face'); return;
				}
				var member = face.member;
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
		if (weld.passes == null || weld.passes.length == 0 || weld.passes.length > 16) {
			fail('step $index needs between 1 and 16 weld passes'); return;
		}
		for (pass in weld.passes) {
			if (pass == null) { fail('step $index has an empty pass'); return; }
			if (pass.name == null || pass.name.length == 0) fail('step $index has an unnamed pass');
			if (pass.offset == null || pass.offset.length != 2) { fail('step $index needs two face offsets'); return; }
			for (offset in pass.offset) if (!(finite(offset) && offset >= 0 && offset <= 0.05)) fail('step $index needs face offsets between 0 and 50 mm');
			if (!(finite(pass.interpassDwell) && pass.interpassDwell >= 0 && pass.interpassDwell <= 10)) fail('step $index needs interpass dwell between 0 and 10 s');
			var weave = pass.weave;
			if (weave != null) {
				if (weave.pattern != "sine" && weave.pattern != "triangle" && weave.pattern != "zigzag") fail('step $index has an unknown weave pattern');
				if (!(finite(weave.amplitude) && weave.amplitude > 0 && weave.amplitude <= 0.025)) fail('step $index needs weave amplitude between 0 and 25 mm');
				if (!(finite(weave.cyclesPerMetre) && weave.cyclesPerMetre > 0 && weave.cyclesPerMetre <= 1000)) fail('step $index needs weave frequency between 0 and 1000 cycles/m');
				if (!(finite(weave.edgeDwell) && weave.edgeDwell >= 0 && weave.edgeDwell <= 10)) fail('step $index needs edge dwell between 0 and 10 s');
			}
			var process = pass.process;
			if (process == null) { fail('step $index has no process'); return; }
			if (!(finite(process.wireSpeed) && process.wireSpeed >= 1 && process.wireSpeed <= 30)) fail('step $index needs a wire speed between 1 and 30 m/min');
			if (!(finite(process.voltage) && process.voltage > 0 && process.voltage <= 60)) fail('step $index needs a voltage between 0 and 60 V');
			if (!(finite(process.travelSpeed) && process.travelSpeed > 0 && process.travelSpeed <= 0.05)) fail('step $index needs a travel speed between 0 and 50 mm/s');
			if (!(finite(process.approach) && process.approach > 0 && process.approach <= 0.5)) fail('step $index needs an approach distance between 0 and 500 mm');
			for (seconds in [process.startDwell, process.craterDwell])
				if (!(finite(seconds) && seconds >= 0 && seconds <= 10)) fail('step $index needs dwell times between 0 and 10 s');
			if (!(finite(process.burnback) && process.burnback >= 0.05 && process.burnback <= 10))
				fail('step $index needs a burnback between 0.05 and 10 s: with none the wire sticks in the pool');
			if (weave != null && !(2 * process.travelSpeed * weave.edgeDwell * weave.cyclesPerMetre < 1))
				fail('step $index has edge dwells that consume the entire weave period');
		}
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
		function seamFaces(value:Dynamic):Array<SceneArtifactWeldFace> {
			if (!Std.isOfType(value, Array)) return fail();
			var result:Array<SceneArtifactWeldFace> = [];
			for (item in (cast value:Array<Dynamic>)) {
				var member:Dynamic = Reflect.field(item, "member"), face:Dynamic = Reflect.field(item, "face");
				if (!Std.isOfType(member, String) || !Std.isOfType(face, String)) return fail();
				result.push({member: member, face: face});
			}
			return result;
		}
		function segment(item:Dynamic):SceneArtifactWeldSegment {
			var kind:Dynamic = Reflect.field(item, "kind"), seam:Dynamic = Reflect.field(item, "seam"), joint:Dynamic = Reflect.field(item, "joint");
			var normals:Dynamic = Reflect.field(item, "normals");
			if (!Std.isOfType(kind, String) || !Std.isOfType(seam, String) || !Std.isOfType(joint, String) || !Std.isOfType(normals, Array)) fail();
			return {kind: kind, seam: seam, faces: seamFaces(Reflect.field(item, "faces")), joint: joint, start: pose(Reflect.field(item, "start")), stop: pose(Reflect.field(item, "stop")),
				normals: [for (normal in (cast normals:Array<Dynamic>)) numbers(normal, 3)]};
		}
		var metal:Dynamic = Reflect.field(raw, "metal"), path:Dynamic = Reflect.field(raw, "path"), passes:Dynamic = Reflect.field(raw, "passes");
		if (!Std.isOfType(metal, String) || !Std.isOfType(passes, Array) || Reflect.hasField(raw, "process")) fail();
		if (!Std.isOfType(path, Array)) fail();
		var segments:Array<SceneArtifactWeldSegment> = [for (item in (cast path:Array<Dynamic>)) segment(item)];
		var frame:Dynamic = Reflect.field(raw, "frame");
		if (frame != null && !Std.isOfType(frame, String)) fail();
		var decodedPasses:Array<SceneArtifactWeldPass> = [for (item in (cast passes:Array<Dynamic>)) {
			var name:Dynamic = Reflect.field(item, "name"), process:Dynamic = Reflect.field(item, "process");
			if (!Std.isOfType(name, String) || process == null) fail();
			var pass:SceneArtifactWeldPass = {name: name, offset: numbers(Reflect.field(item, "offset"), 2),
				interpassDwell: number(item, "interpassDwell"),
				process: {wireSpeed: number(process, "wireSpeed"), voltage: number(process, "voltage"),
					travelSpeed: number(process, "travelSpeed"), approach: number(process, "approach"),
					startDwell: number(process, "startDwell"), craterDwell: number(process, "craterDwell"), burnback: number(process, "burnback")}};
			var weave:Dynamic = Reflect.field(item, "weave");
			if (weave != null) {
				var pattern:Dynamic = Reflect.field(weave, "pattern");
				if (!Std.isOfType(pattern, String)) fail();
				pass.weave = {pattern: pattern, amplitude: number(weave, "amplitude"), cyclesPerMetre: number(weave, "cyclesPerMetre"), edgeDwell: number(weave, "edgeDwell")};
			}
			pass;
		}];
		var weld:SceneArtifactWeld = {metal: metal, path: segments, legSize: number(raw, "legSize"), passes: decodedPasses};
		if (frame != null) weld.frame = frame;
		return weld;
	}

	/** A mission from its JSON section, typed field by field. */
	static function decodePowerUpOffsets(decoded:Dynamic):Array<SceneArtifactPowerUpOffset> {
		if (!Std.isOfType(decoded, Array)) throw "Power-up offsets require an array";
		return [for (raw in (cast decoded:Array<Dynamic>)) {
			if (raw == null) throw "Power-up offset is null";
			for (field in Reflect.fields(raw)) if (field != "joint" && field != "offset")
				throw 'Unknown power-up offset field "$field"';
			var joint:Dynamic = Reflect.field(raw, "joint"), offset:Dynamic = Reflect.field(raw, "offset");
			if (!Std.isOfType(joint, String) || (!Std.isOfType(offset, Float) && !Std.isOfType(offset, Int)))
				throw "Power-up offset requires joint name and SI number";
			{joint: (joint:String), offset: (offset:Float)};
		}];
	}

	static function decodePowerUpSideOffsets(decoded:Dynamic):Array<SceneArtifactPowerUpSideOffset> {
		if (!Std.isOfType(decoded, Array)) throw "Power-up side offsets require an array";
		return [for (raw in (cast decoded:Array<Dynamic>)) {
			if (raw == null) throw "Power-up side offset is null";
			for (field in Reflect.fields(raw)) if (field != "homeSwitch" && field != "offset")
				throw 'Unknown power-up side offset field "$field"';
			var id:Dynamic = Reflect.field(raw, "homeSwitch"), offset:Dynamic = Reflect.field(raw, "offset");
			if (!Std.isOfType(id, String) || (!Std.isOfType(offset, Float) && !Std.isOfType(offset, Int)))
				throw "Power-up side offset requires a switch ID and SI number";
			{homeSwitch: (id:String), offset: (offset:Float)};
		}];
	}

	static function validatePowerUpSideOffsets(offsets:Null<Array<SceneArtifactPowerUpSideOffset>>,
			definition:Null<AssemblyDefinition>):Void {
		if (offsets == null) return;
		if (definition == null || offsets.length > 512) throw "Power-up sides require a bounded machine layout";
		var flat = AssemblyDefinitionFlattener.flattenView(definition), seen = new Map<String, Bool>();
		for (entry in offsets) {
			if (entry == null || entry.homeSwitch == null || !finite(entry.offset)) throw "Invalid power-up side offset";
			var drive:Null<String> = null;
			if (flat.switches != null) for (contact in flat.switches)
				if (contact.id == entry.homeSwitch && contact.role == "home" && contact.driveJoint != contact.joint) {
					for (joint in flat.joints) if (joint.id == contact.joint && Std.string(joint.type) == "prismatic")
						drive = contact.driveJoint;
				}
			if (drive == null || seen.exists(drive)) throw "Power-up sides require distinct home motor drives";
			seen.set(drive, true);
		}
	}

	static function validatePowerUpOffsets(offsets:Null<Array<SceneArtifactPowerUpOffset>>,
			definition:Null<AssemblyDefinition>):Void {
		if (offsets == null) return;
		if (definition == null || offsets.length > 512) throw "Power-up offsets require a bounded machine layout";
		var flat = AssemblyDefinitionFlattener.flattenView(definition), seen = new Map<String, Bool>();
		for (entry in offsets) {
			if (entry == null || entry.joint == null || seen.exists(entry.joint) || !finite(entry.offset))
				throw "Power-up offsets require distinct joints and finite SI displacements";
			seen.set(entry.joint, true);
			var found = false, home = false;
			for (joint in flat.joints) if (joint.id == entry.joint && Std.string(joint.type) == "prismatic") found = true;
			if (flat.switches != null) for (contact in flat.switches)
				if (contact.joint == entry.joint && contact.role == "home") home = true;
			if (!found || !home) throw 'Power-up joint "${entry.joint}" requires a prismatic home axis';
		}
	}

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
		function bool(value:Dynamic, name:String):Bool {
			var item:Dynamic = Reflect.field(value, name);
			return Std.isOfType(item, Bool) ? (item:Bool) : fail();
		}
		function missionPose(value:Dynamic):SceneArtifactTorchPose {
			if (value == null) fail();
			var position:Dynamic = Reflect.field(value, "position"), rotation:Dynamic = Reflect.field(value, "rotation");
			if (!Std.isOfType(position, Array) || (cast position:Array<Dynamic>).length != 3 ||
				!Std.isOfType(rotation, Array) || (cast rotation:Array<Dynamic>).length != 4) fail();
			return {position: [for (item in (cast position:Array<Dynamic>))
				Std.isOfType(item, Float) || Std.isOfType(item, Int) ? (item:Float) : fail()],
				rotation: [for (item in (cast rotation:Array<Dynamic>))
				Std.isOfType(item, Float) || Std.isOfType(item, Int) ? (item:Float) : fail()]};
		}
		var steps:Dynamic = Reflect.field(decoded, "steps");
		if (!Std.isOfType(steps, Array)) fail();
		var mission:SceneArtifactMission = {steps: [for (raw in (cast steps:Array<Dynamic>)) {
			var kind:Dynamic = Reflect.field(raw, "kind");
			if (!Std.isOfType(kind, String)) fail();
			var step:SceneArtifactMissionStep = {kind: kind};
			var configuration:Dynamic=Reflect.field(raw,"configuration");
			if(configuration!=null){
				var shoulder:Dynamic=Reflect.field(configuration,"shoulder"),elbow:Dynamic=Reflect.field(configuration,"elbow"),
					wrist:Dynamic=Reflect.field(configuration,"wrist");
				if(!Std.isOfType(shoulder,String) || !Std.isOfType(elbow,String) || !Std.isOfType(wrist,String))fail();
				var pin:SceneArtifactArmConfiguration={shoulder:shoulder,elbow:elbow,wrist:wrist};
				var turns:Dynamic=Reflect.field(configuration,"turns");
				if(turns!=null){
					if(!Std.isOfType(turns,Array))fail();
					pin.turns=[for(turn in (cast turns:Array<Dynamic>)){
						if(!Std.isOfType(turn,Int))fail();
						(turn:Int);
					}];
				}
				step.configuration=pin;
			}
			var pose:Dynamic = Reflect.field(raw, "pose");
			if (pose != null) step.pose = {x: number(pose, "x"), y: number(pose, "y"), yaw: number(pose, "yaw")};
			var at:Dynamic = Reflect.field(raw, "at");
			if (at != null) step.at = place(at);
			if (Reflect.field(raw, "yaw") != null) step.yaw = number(raw, "yaw");
			if (Reflect.field(raw, "toolYaw") != null) step.toolYaw = number(raw, "toolYaw");
			var targets:Dynamic = Reflect.field(raw, "joints");
			if (targets != null) {
				if (!Std.isOfType(targets, Array)) fail();
				step.joints = [for (target in (cast targets:Array<Dynamic>)) {
					var joint:Dynamic = Reflect.field(target, "joint");
					if (!Std.isOfType(joint, String)) fail();
					{joint: (joint:String), position: number(target, "position")};
				}];
			}
			var weld:Dynamic = Reflect.field(raw, "weld");
			if (weld != null) step.weld = decodeWeld(weld);
			var contact:Dynamic = Reflect.field(raw, "contactWork");
			if (contact != null) step.contactWork = SceneContactRegistration.decode(contact);
			var signal:Dynamic = Reflect.field(raw, "signal");
			if (signal != null) {
				var name:Dynamic = Reflect.field(signal, "name");
				if (!Std.isOfType(name, String)) fail();
				step.signal = {name: name, value: bool(signal, "value")};
			}
			var waitFor:Dynamic = Reflect.field(raw, "waitFor");
			if (waitFor != null) {
				var name:Dynamic = Reflect.field(waitFor, "name");
				if (!Std.isOfType(name, String)) fail();
				step.waitFor = {name: name, value: bool(waitFor, "value"), timeout: number(waitFor, "timeout")};
				var retryStep:Dynamic = Reflect.field(waitFor, "retryStep");
				if (retryStep != null) {
					var retry = number(waitFor, "retryStep");
					if (retry < 0 || retry != Std.int(retry)) fail();
					step.waitFor.retryStep = Std.int(retry);
				}
			}
			var poses:Dynamic = Reflect.field(raw, "poses");
			if (poses != null) {
				if (!Std.isOfType(poses, Array)) fail();
				step.poses = [for (waypoint in (cast poses:Array<Dynamic>)) {
					var result:SceneArtifactMissionWaypoint = {};
					var waypointJoints:Dynamic = Reflect.field(waypoint, "joints");
					if (waypointJoints != null) {
						if (!Std.isOfType(waypointJoints, Array)) fail();
						result.joints = [for (target in (cast waypointJoints:Array<Dynamic>)) {
							var joint:Dynamic = Reflect.field(target, "joint");
							if (!Std.isOfType(joint, String)) fail();
							{joint: joint, position: number(target, "position")};
						}];
					}
					var targetPose:Dynamic = Reflect.field(waypoint, "pose");
					if (targetPose != null) result.pose = missionPose(targetPose);
					result;
				}];
			}
			if (Reflect.field(raw, "timeoutSeconds") != null)
				step.timeoutSeconds = number(raw, "timeoutSeconds");
			if (Reflect.field(raw, "approachDistance") != null)
				step.approachDistance = number(raw, "approachDistance");
			var tool:Dynamic = Reflect.field(raw, "tool");
			if (tool != null) {
				if (!Std.isOfType(tool, String) || (tool:String).length == 0) fail();
				step.tool = tool;
			}
			step;
		}]};
		var loop:Dynamic = Reflect.field(decoded, "loop");
		if (loop != null) mission.loop = Std.isOfType(loop, Bool) ? (loop:Bool) : fail();
		var offsets:Dynamic = Reflect.field(decoded, "powerUpOffsets");
		if (offsets != null) mission.powerUpOffsets = decodePowerUpOffsets(offsets);
		var sides:Dynamic = Reflect.field(decoded, "powerUpSideOffsets");
		if (sides != null) mission.powerUpSideOffsets = decodePowerUpSideOffsets(sides);
		var controllerId:Dynamic = Reflect.field(decoded, "controllerId");
		if (controllerId != null) mission.controllerId = Std.isOfType(controllerId, String) ? controllerId : fail();
		return mission;
	}

	static function validateMobileBase(base:SceneArtifactMobileBase, definition:Null<AssemblyDefinition>):Void {
		function fail(detail:String):Void throw 'Scene artifact mobile base $detail';
		if (definition == null) fail("needs the robot's assembly definition");
		var flat = AssemblyDefinitionFlattener.flattenView(definition);
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
		if (base.energy != null) {
			var energy = base.energy;
			var occurrences = new Map<String, Bool>();
			for (item in flat.occurrences) occurrences.set(item.id, true);
			for (name in [energy.batteryOccurrence, energy.chargerOccurrence])
				if (name == null || name.length == 0 || !occurrences.exists(name)) fail('energy profile names no occurrence "$name"');
			if (!finite(energy.initialChargeFraction) || energy.initialChargeFraction < 0 || energy.initialChargeFraction > 1)
				fail("needs a valid initial charge fraction");
			for (value in [energy.nominalVolts, energy.capacityWh, energy.fullVoltage, energy.emptyVoltage,
				energy.cutoffVoltage, energy.internalResistanceOhms, energy.maxDischargeCurrentA,
				energy.maxChargePowerW, energy.inverterContinuousWatts, energy.computerLoadWatts, energy.chargerPowerWatts])
				if (!finite(value) || value <= 0) fail("needs positive battery, load and charger ratings");
			for (value in [energy.chargeEfficiency, energy.inverterEfficiency, energy.armSupplyEfficiency,
				energy.driveSupplyEfficiency, energy.computerSupplyEfficiency, energy.armDriveEfficiency,
				energy.driveMotorEfficiency, energy.driveGearEfficiency])
				if (!finite(value) || value <= 0 || value > 1) fail("needs conversion efficiencies in (0, 1]");
			if (!(energy.emptyVoltage < energy.nominalVolts && energy.nominalVolts < energy.fullVoltage &&
				energy.emptyVoltage < energy.cutoffVoltage && energy.cutoffVoltage < energy.fullVoltage) ||
				!finiteFloorPose(energy.dockPose)) fail("has an invalid voltage curve or dock pose");
			if (!finite(energy.dockPositionToleranceMeters) || energy.dockPositionToleranceMeters <= 0 ||
				energy.dockPositionToleranceMeters > 0.18 || !finite(energy.dockHeadingToleranceRadians) ||
				energy.dockHeadingToleranceRadians <= 0 || energy.dockHeadingToleranceRadians >= Math.PI / 2)
				fail("has invalid charging-contact alignment tolerances");
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
		var energyRaw:Dynamic = Reflect.field(decoded, "energy");
		if (energyRaw != null) {
			function energyText(name:String):String {
				var value:Dynamic = Reflect.field(energyRaw, name);
				return Std.isOfType(value, String) ? value : fail();
			}
			function energyNumber(name:String):Float {
				var value:Dynamic = Reflect.field(energyRaw, name);
				return Std.isOfType(value, Float) || Std.isOfType(value, Int) ? (value:Float) : fail();
			}
			base.energy = {initialChargeFraction: energyNumber("initialChargeFraction"),
				batteryOccurrence: energyText("batteryOccurrence"),
				nominalVolts: energyNumber("nominalVolts"), capacityWh: energyNumber("capacityWh"),
				fullVoltage: energyNumber("fullVoltage"), emptyVoltage: energyNumber("emptyVoltage"),
				cutoffVoltage: energyNumber("cutoffVoltage"),
				internalResistanceOhms: energyNumber("internalResistanceOhms"),
				maxDischargeCurrentA: energyNumber("maxDischargeCurrentA"),
				maxChargePowerW: energyNumber("maxChargePowerW"), chargeEfficiency: energyNumber("chargeEfficiency"),
				inverterContinuousWatts: energyNumber("inverterContinuousWatts"),
				inverterEfficiency: energyNumber("inverterEfficiency"),
				armSupplyEfficiency: energyNumber("armSupplyEfficiency"),
				driveSupplyEfficiency: energyNumber("driveSupplyEfficiency"),
				computerSupplyEfficiency: energyNumber("computerSupplyEfficiency"),
				armDriveEfficiency: energyNumber("armDriveEfficiency"),
				driveMotorEfficiency: energyNumber("driveMotorEfficiency"),
				driveGearEfficiency: energyNumber("driveGearEfficiency"),
				computerLoadWatts: energyNumber("computerLoadWatts"),
				chargerOccurrence: energyText("chargerOccurrence"),
				chargerPowerWatts: energyNumber("chargerPowerWatts"),
				dockPose: floorPose(Reflect.field(energyRaw, "dockPose")),
				dockPositionToleranceMeters: energyNumber("dockPositionToleranceMeters"),
				dockHeadingToleranceRadians: energyNumber("dockHeadingToleranceRadians")};
		}
		return base;
	}

	static function validateMachining(machining:SceneArtifactMachining, parts:Map<String, Bool>,
			definition:Null<AssemblyDefinition>):Void {
		function fail(detail:String):Void throw 'Scene artifact machining job $detail';
		validatePowerUpOffsets(machining.powerUpOffsets, definition);
		validatePowerUpSideOffsets(machining.powerUpSideOffsets, definition);
		if (machining.program == null) fail("has no program");
		if (machining.channelScope == null) fail("has no process-channel scope");
		if (machining.channelScope.length > 0) for (part in machining.channelScope.split("/"))
			if (StringTools.trim(part) != part || part.length == 0 || part == "." || part == "..")
				fail("has an invalid process-channel scope");
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
		if (machining.stocks == null) fail("needs a stock occurrence list");
		var stockIds = new Map<String, Bool>();
		for (stock in machining.stocks) {
			if (!occurrences.exists(stock)) fail('stock "$stock" is not an occurrence');
			if (stockIds.exists(stock)) fail('stock "$stock" is listed more than once');
			stockIds.set(stock, true);
		}
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
		if (controller != null && controller.stepTickHz < 1)
			fail("has a controller with no step rate");
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
			channelScope: text(Reflect.field(decoded, "channelScope")),
			workOffset: [for (value in list(Reflect.field(decoded, "workOffset"))) number(value)],
			tools: tools,
			stocks: [for (stock in list(Reflect.field(decoded, "stocks"))) text(stock)]};
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
			for (field in Reflect.fields(controller)) if (field != "stepTickHz")
				throw 'Machining controller has an unknown field "$field"';
			var tick:Dynamic = Reflect.field(controller, "stepTickHz");
			if (!Std.isOfType(tick, Int)) fail();
			machining.controller = {stepTickHz: (tick:Int)};
		}
		var offsets:Dynamic = Reflect.field(decoded, "powerUpOffsets");
		if (offsets != null) machining.powerUpOffsets = decodePowerUpOffsets(offsets);
		var sides:Dynamic = Reflect.field(decoded, "powerUpSideOffsets");
		if (sides != null) machining.powerUpSideOffsets = decodePowerUpSideOffsets(sides);
		var controllerId:Dynamic = Reflect.field(decoded, "controllerId");
		if (controllerId != null) machining.controllerId = text(controllerId);
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
			part.inertia != null && part.inertia.length != 9 ||
			part.physicalMassKg != null && (!finite(part.physicalMassKg) || part.physicalMassKg <= 0) ||
			part.physicalInertiaKgM2 != null && part.physicalInertiaKgM2.length != 9)
			throw 'Scene artifact part "${part.id}" has invalid mass properties';
		if (part.centerOfMass != null) for (coordinate in part.centerOfMass)
			if (!finite(coordinate)) throw 'Scene artifact part "${part.id}" has invalid centre of mass';
		if (part.inertia != null) for (component in part.inertia)
			if (!finite(component)) throw 'Scene artifact part "${part.id}" has invalid inertia';
		if (part.physicalInertiaKgM2 != null) for (component in part.physicalInertiaKgM2)
			if (!finite(component)) throw 'Scene artifact part "${part.id}" has invalid physical inertia';
		if (appearance != null && (appearance.finish == null ||
			StringTools.trim(appearance.finish).length == 0 ||
			Bytes.ofString(appearance.finish).length > 4096 ||
			!finite(appearance.metallic) || appearance.metallic < 0 || appearance.metallic > 1 ||
			!finite(appearance.roughness) || appearance.roughness < 0 || appearance.roughness > 1))
			throw 'Scene artifact part "${part.id}" has an invalid appearance';
		if (part.threads != null) {
			if (part.threads.length > MAX_FACE_RANGES) throw 'Part ${part.id} has too many threads';
			for (thread in part.threads) {
				ThreadRenderDataValues.validate(thread);
				var found = false;
				for (range in part.faceRanges) if (range.faceIndex == thread.faceIndex) found = true;
				if (!found) throw 'Thread ${part.id}/${thread.faceName} has no tessellated face range';
			}
		}

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
	final borrowStreams:Bool;
	public function new(source:Bytes, borrowStreams:Bool = false) {
		this.source = source;
		this.borrowStreams = borrowStreams;
	}

	public function read():SceneArtifactData {
		for (expected in [77, 84, 82, 71]) if (readByte() != expected)
			throw "Scene artifact has an invalid signature";
		var version = readInt();
		if (version != SceneArtifact.VERSION)
			throw 'schema v$version is unsupported; expected v${SceneArtifact.VERSION}; rebuild the scene artifact from its project source';
		var metresPerUnit = readDouble();
		var lengthUnit = readText();
		var count = readInt();
		if (!Math.isFinite(metresPerUnit) || metresPerUnit <= 0.0 || count <= 0 || count > 1000)
			throw "Scene artifact has invalid units or part count";
		var ids = new Map<String, Bool>(), parts:Array<SceneArtifactPart> = [];
		for (_ in 0...count) {
			var id = readText(), name = readText();
			if (ids.exists(id)) throw 'Scene artifact has duplicate part ID "$id"';
			ids.set(id, true);
			var red = readFloat(), green = readFloat(), blue = readFloat();
			var appearance:Appearance = {finish: readText(), metallic: readFloat(), roughness: readFloat()};
			var materialId = readText();
			var density = readDouble();
			var materialSpec = readText();
			var volume = readDouble();
			var center = [for (_ in 0...3) readDouble()];
			var inertia = [for (_ in 0...9) readDouble()];
			var physicalMassKg = readDouble();
			var physicalMassDeclared = readByte();
			if (physicalMassDeclared != 0 && physicalMassDeclared != 1)
				throw 'Scene artifact part "$id" has invalid physical mass provenance';
			var physicalInertiaKgM2 = [for (_ in 0...9) readDouble()];
			var vertexCount = readInt(), indexCount = readInt(), rangeCount = readInt();
			var edgeByteCount = readInt();
			if (vertexCount <= 0 || vertexCount > 2000000 || indexCount <= 0 ||
				indexCount % 3 != 0 || indexCount / 3 > 4000000 || rangeCount < 0 || rangeCount > 100000 ||
				edgeByteCount < 0 || edgeByteCount % 48 != 0 || edgeByteCount > SceneArtifact.MAX_BYTES)
				throw 'Scene artifact part "$id" has invalid mesh counts';
		var vertices = readStream(vertexCount * 24), normals = readStream(vertexCount * 24),
				indices = readStream(indexCount * 4), edgeSegments = readStream(edgeByteCount);
			var edgeIds = readStream(Std.int(edgeByteCount / 48) * 4);
			var faceRanges:Array<SceneArtifactFaceRange> = [];
			for (_ in 0...rangeCount)
				faceRanges.push({faceIndex: readInt(), firstIndex: readInt(), indexCount: readInt()});
			var faceDescriptors = readOpaqueText(16000000, 'part "$id" face descriptors');
			var threadText = readOpaqueText(16000000, 'part "$id" threads');
			var threads:Array<ThreadRenderData> = threadText.length == 0 ? [] : JsonWire.decode(threadText);
			var part:SceneArtifactPart = {threads: threads, id: id, name: name, red: red, green: green, blue: blue,
				appearance: appearance, materialId: materialId, materialDensity: density, materialSpec: materialSpec,
				volume: volume, centerOfMass: center, inertia: inertia,
				physicalMassKg: physicalMassKg, physicalMassDeclared: physicalMassDeclared == 1,
				physicalInertiaKgM2: physicalInertiaKgM2,
				vertexCount: vertexCount, indexCount: indexCount, vertices: vertices, normals: normals,
				indices: indices, edgeSegments: edgeSegments, edgeIds: edgeIds,
				faceRanges: faceRanges};
			if (faceDescriptors.length > 0) part.faceDescriptors = faceDescriptors;
			@:privateAccess SceneArtifact.validatePart(part, true);
			parts.push(part);
		}
		var assemblyDefinition:Null<AssemblyDefinition> = null, assemblyState:Null<AssemblyStateRecord> = null;
		var definitionLength = readInt();
		if (definitionLength < 0 || definitionLength > 2000000)
			throw "Scene artifact assembly definition is too large";
		if (definitionLength > 0)
			assemblyDefinition = AssemblyDefinitionCodec.decode(readString(definitionLength));
		var stateLength = readInt();
		if (stateLength < 0 || stateLength > 2000000)
			throw "Scene artifact assembly state is too large";
		if (stateLength > 0) {
			if (assemblyDefinition == null) throw "Scene artifact assembly state has no definition";
			assemblyState = AssemblyDefinitionCodec.decodeState(assemblyDefinition,
				readString(stateLength));
		}
		var recipeDocument:Null<String> = null;
		var documentLength = readInt();
		if (documentLength < 0 || documentLength > 2000000)
			throw "Scene artifact recipe document is too large";
		if (documentLength > 0) recipeDocument = readString(documentLength);
		var recipeDiagnostics:Null<Array<String>> = null;
		var diagnosticsLength = readInt();
		if (diagnosticsLength < 0 || diagnosticsLength > 2000000)
			throw "Scene artifact recipe diagnostics are too large";
		if (diagnosticsLength > 0) {
			var decoded:Dynamic = haxe.Json.parse(readString(diagnosticsLength));
			if (!Std.isOfType(decoded, Array)) throw "Scene artifact recipe diagnostics are invalid";
			for (entry in (cast decoded:Array<Dynamic>))
				if (!Std.isOfType(entry, String)) throw "Scene artifact recipe diagnostics are invalid";
			recipeDiagnostics = cast decoded;
		}
		var machining:Null<SceneArtifactMachining> = null;
		var machiningLength = readInt();
		if (machiningLength < 0 || machiningLength > 8000000) throw "Scene artifact machining job is too large";
		if (machiningLength > 0) {
			machining = @:privateAccess SceneArtifact.decodeMachining(haxe.Json.parse(readString(machiningLength)));
		}
		var mobileBase:Null<SceneArtifactMobileBase> = null;
		var mobileLength = readInt();
		if (mobileLength < 0 || mobileLength > 100000) throw "Scene artifact mobile base is too large";
		if (mobileLength > 0)
			mobileBase = @:privateAccess SceneArtifact.decodeMobileBase(haxe.Json.parse(readString(mobileLength)));
		var mission:Null<SceneArtifactMission> = null;
		var missionLength = readInt();
		if (missionLength < 0 || missionLength > 1000000) throw "Scene artifact mission is too large";
		if (missionLength > 0)
			mission = @:privateAccess SceneArtifact.decodeMission(haxe.Json.parse(readString(missionLength)));
		var robotTools:Null<Array<SceneArtifactRobotTool>> = null;
		var toolsLength = readInt();
		if (toolsLength < 0 || toolsLength > 100000) throw "Scene artifact robot tools are too large";
		if (toolsLength > 0)
			robotTools = @:privateAccess SceneArtifact.decodeRobotTools(haxe.Json.parse(readString(toolsLength)));
		var robotSensors:Null<Array<SceneArtifactRobotSensor>> = null;
		var sensorsLength = readInt();
		if (sensorsLength < 0 || sensorsLength > 100000) throw "Scene artifact robot sensors are too large";
		if (sensorsLength > 0)
			robotSensors = @:privateAccess SceneArtifact.decodeRobotSensors(haxe.Json.parse(readString(sensorsLength)));
    var machineMotion:Null<SceneArtifactMachineMotion> = null;
    var machineLength = readInt();
    if (machineLength < 0 || machineLength > 1000000) throw "Scene artifact machine motion is too large";
    if (machineLength > 0) {
      var decodedMotion:SceneArtifactMachineMotion = JsonWire.decode(readBytes(machineLength).getString(0, machineLength));
      machineMotion = decodedMotion;
    }
		var controllers:Null<Array<SceneArtifactCellController>> = null;
		var controllerLength = readInt();
		if (controllerLength < 0 || controllerLength > 100000) throw "Scene artifact controllers are too large";
		if (controllerLength > 0)
			controllers = @:privateAccess SceneArtifact.decodeControllers(haxe.Json.parse(readString(controllerLength)));
		var signals:Null<Array<SceneArtifactCellSignal>> = null;
		var signalLength = readInt();
		if (signalLength < 0 || signalLength > 100000) throw "Scene artifact signals are too large";
		if (signalLength > 0)
			signals = @:privateAccess SceneArtifact.decodeSignals(haxe.Json.parse(readString(signalLength)));
		var project:Null<SceneArtifactProject> = null;
		var projectLength = readInt();
		if (projectLength < 0 || projectLength > 2000000) throw "Scene artifact project settings are too large";
		if (projectLength > 0) project = @:privateAccess SceneArtifact.decodeProject(haxe.Json.parse(readString(projectLength)));
		var printing:Null<SceneArtifactPrinting> = null;
		var printingLength = readInt();
		if (printingLength < 0 || printingLength > 40000000) throw "Scene artifact print job is too large";
		if (printingLength > 0) printing = @:privateAccess SceneArtifact.decodePrinting(haxe.Json.parse(readString(printingLength)));
		if (offset != source.length) throw "Scene artifact contains trailing data";
		var result:SceneArtifactData = {metresPerUnit: metresPerUnit, lengthUnit: lengthUnit,
			parts: parts,
			assemblyDefinition: assemblyDefinition, assemblyState: assemblyState, recipeDocument: recipeDocument,
			recipeDiagnostics: recipeDiagnostics, machining: machining, mobileBase: mobileBase, mission: mission,
			robotTools: robotTools, robotSensors: robotSensors, machineMotion: machineMotion,
			controllers: controllers, signals: signals, project: project,
			printing: printing};
		@:privateAccess SceneArtifact.validateHeader(result);
		return result;
	}

	/** A length-prefixed text that may be empty, up to `limit` bytes. */
	function readOpaqueText(limit:Int, what:String):String {
		var length = readInt();
		if (length < 0 || length > limit) throw 'Scene artifact $what are too large';
		return length == 0 ? "" : readString(length);
	}

	function readText():String {
		var length = readInt();
		if (length <= 0 || length > 4096) throw "Scene artifact has an invalid part ID or name";
		var value = readString(length);
		if (StringTools.trim(value).length == 0) throw "Scene artifact has an empty part ID or name";
		return value;
	}

	function readByte():Int {
		if (offset >= source.length) throw "Scene artifact ended unexpectedly";
		return source.get(offset++);
	}
	inline function require(length:Int):Void {
		if (length < 0 || length > source.length - offset)
			throw "Scene artifact has an invalid buffer length";
	}
	function readInt():Int {
		require(4);
		var value = source.getInt32(offset);
		offset += 4;
		return value;
	}
	function readDouble():Float {
		require(8);
		var value = source.getDouble(offset);
		offset += 8;
		return value;
	}
	function readFloat():Float {
		require(4);
		var value = source.getFloat(offset);
		offset += 4;
		return value;
	}
	function readString(length:Int):String {
		require(length);
		var value = source.getString(offset, length);
		offset += length;
		return value;
	}
	function readStream(length:Int):Bytes {
		if (!borrowStreams) return readBytes(length);
		require(length);
		var value = Bytes.view(source, offset, length);
		offset += length;
		return value;
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
