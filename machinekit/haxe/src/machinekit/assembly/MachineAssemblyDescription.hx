package machinekit.assembly;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Serializable recipe inputs. The constructor IDs are part of the wire schema. */
@:wire enum MemberSource {
	@:id(1) Typed(typeId:String, values:Array<NamedValue>);
	@:id(2) Code(designation:String);
}

@:wire enum SavedValue {
	@:id(1) Number(value:Float);
	@:id(2) Integer(value:Int);
	@:id(3) Boolean(value:Bool);
	@:id(4) Token(value:String);
	@:id(5) Unset;
}

@:wire typedef NamedValue = {
	@:id(1) var name:String;
	@:id(2) var value:SavedValue;
}

/** A member of one level, by its local name. */
@:wire typedef MemberRecord = {
	@:id(1) var occurrence:String;
	@:id(2) var source:MemberSource;
	@:id(3) var material:String;
}

@:wire typedef PortConnectionRecord = {
	@:id(1) var id:String;
	@:id(2) var fromInstance:String;
	@:id(3) var fromPort:String;
	@:id(4) var toInstance:String;
	@:id(5) var toPort:String;
}

@:wire typedef PortExposureRecord = {
	@:id(1) var name:String;
	@:id(2) var instanceId:String;
	@:id(3) var portName:String;
}

@:wire typedef ConnectorExposureRecord = {
	@:id(1) var name:String;
	@:id(2) var instanceId:String;
	@:id(3) var connectorName:String;
}

@:wire typedef MemberConnectorRecord = {
	@:id(1) var instanceId:String;
	@:id(2) var name:String;
	@:id(3) var frame:AssemblyFrame;
}

@:wire typedef ConnectorReference = {
	@:id(1) var instanceId:String;
	@:id(2) var connectorName:String;
}

@:wire typedef BomExtraRecord = {
	@:id(1) var item:SavedBomItem;
	@:id(2) var quantity:Int;
	@:id(3) var mass:SavedBomMass;
}

@:wire typedef SavedBomItem = {
	@:id(1) var partNumber:String;
	@:id(2) var description:String;
	@:id(3) var quantity:Int;
	@:id(4) var material:Null<String>;
	@:id(5) @:optional var typeId:String;
	@:id(6) @:optional var valuesKey:String;
}

@:wire enum SavedBomMass {
	@:id(1) Unknown;
	@:id(2) Point(kg:Float, x:Float, y:Float, z:Float);
	@:id(3) Attached(kg:Float, instanceId:String, x:Float, y:Float, z:Float);
}

/** Source of a derived coupling; the follower is zero at `leaderZero`. */
@:wire typedef TransmissionRecord = {
	@:id(1) var coupling:String;
	@:id(2) var source:Transmission;
	@:id(3) var sense:Sense;
	@:id(4) var leaderZero:Float;
	/** Stated stiffness at the leader, N per leader unit. */
	@:id(5) @:optional var stiffness:Null<Float>;
	/** Stated lost motion on reversal, in leader units. */
	@:id(6) @:optional var backlash:Null<Float>;
	/** Stated drag torque at the follower, N m. */
	@:id(7) @:optional var drag:Null<Float>;
	@:id(8) @:optional var near:Null<machinekit.motion.ScrewSupport>;
	@:id(9) @:optional var far:Null<machinekit.motion.ScrewSupport>;
	/** Longest unsupported stretch in mm; absent means the whole screw. */
	@:id(10) @:optional var unsupported:Null<Float>;
}

/**
 * Motor and driver members driving a joint, its actuator given `margin` of the motor's holding
 * torque (see `MachineAssembly.addMotor`). Voltage and current come from the driver.
 */
@:wire typedef MotorRecord = {
	@:id(1) var actuator:String;
	@:id(2) var joint:String;
	@:id(3) var motor:String;
	@:id(4) var driver:String;
	@:id(5) var margin:Float;
	/** A gearbox between the motor and the joint (see `Gearbox`); absent for a direct drive. */
	@:id(6) @:optional var gearbox:Null<String>;
	/** A process speed/direction pair for a CNC spindle; both channels are analog. */
	@:id(7) @:optional var processVelocity:Null<materia.assembly.AssemblyDefinition.AssemblyProcessVelocityDrive>;
}

/**
 * A pneumatic cylinder driving a prismatic joint through a directional valve. Only the parts are
 * kept; pressure, force and direction are worked out from them and the air wiring.
 */
@:wire typedef CylinderRecord = {
	@:id(1) var actuator:String;
	@:id(2) var joint:String;
	@:id(3) var cylinder:String;
	@:id(4) var valve:String;
}

/**
 * An encoder part reading a joint (see `MachineAssembly.addEncoder`), and the motor's actuator it
 * reads when it is that motor's feedback.
 */
@:wire typedef EncoderRecord = {
	@:id(1) var encoder:String;
	@:id(2) var joint:String;
	@:id(3) var part:String;
	@:id(4) @:optional var actuator:Null<String>;
}

/** Physical clamp and pulley centres, in belt path order. The clamp span follows its geometry. */
@:wire typedef BeltPathRecord = {
	@:id(1) var belt:String;
	@:id(2) @:optional var clamp:Null<ConnectorReference>;
	@:id(3) var wraps:Array<ConnectorReference>;
}

/**
 * The MachineKit facts of one assembly level. Every member reference is a path below that level, so
 * a record may name a member of a subassembly (`arm/link1`).
 */
@:wire typedef MachineLevelRecord = {
	@:id(1) var members:Array<MemberRecord>;
	@:id(2) var memberConnectors:Array<MemberConnectorRecord>;
	@:id(3) var connectorExposures:Array<ConnectorExposureRecord>;
	@:id(4) var portConnections:Array<PortConnectionRecord>;
	@:id(5) var portExposures:Array<PortExposureRecord>;
	@:id(6) var transmissions:Array<TransmissionRecord>;
	@:id(7) var beltPaths:Array<BeltPathRecord>;
	@:id(8) var motors:Array<MotorRecord>;
	@:id(9) var encoders:Array<EncoderRecord>;
	@:id(10) var bomExtras:Array<BomExtraRecord>;
	@:id(11) var cylinders:Array<CylinderRecord>;
	/** Switches, at-speed and presence sensors. */
	@:id(12) var sensors:Array<materia.assembly.AssemblyDefinition.AssemblySensor>;
}

/** One subassembly: its path from the root, its entry in the mechanical table, and its facts. */
@:wire typedef SubassemblyRecord = {
	@:id(1) var path:String;
	@:id(2) var definition:String;
	@:id(3) var machine:MachineLevelRecord;
}

/**
 * A machine assembly as portable data: the nested ProjectKit definition, with each subassembly a
 * table entry, and the MachineKit facts of the root and of each subassembly.
 */
@:wire typedef MachineAssemblyDescription = {
	@:id(1) var schemaVersion:Int;
	@:id(2) var mechanical:AssemblyDefinition;
	@:id(3) var machine:MachineLevelRecord;
	@:id(4) var subassemblies:Array<SubassemblyRecord>;
}

@:wire typedef DescriptionVersion = {
	@:id(1) @:optional var schemaVersion:Int;
}
