package machinekit.assembly;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import haxe.ds.ReadOnlyArray;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.PortInterface;

/** Wire-compatible read-only view of the mechanical schema. */
@:wire typedef FrozenFrame = {
	@:id(1) final x:Float;
	@:id(2) final y:Float;
	@:id(3) final z:Float;
	@:id(4) final qx:Float;
	@:id(5) final qy:Float;
	@:id(6) final qz:Float;
	@:id(7) final qw:Float;
}

@:wire typedef FrozenConnector = {
	@:id(1) final name:String;
	@:id(2) final frame:FrozenFrame;
}

@:wire typedef FrozenComponentDefinition = {
	@:id(1) final id:String;
	@:id(2) final connectors:ReadOnlyArray<FrozenConnector>;
}

@:wire typedef FrozenOccurrence = {
	@:id(1) final id:String;
	@:id(2) final definition:String;
	@:id(3) final initialPose:FrozenFrame;
	@:id(4) @:optional final assembly:String;
}

@:wire typedef FrozenVector = {
	@:id(1) final x:Float;
	@:id(2) final y:Float;
	@:id(3) final z:Float;
}

@:wire typedef FrozenJointLimits = {
	@:id(1) final lower:Null<Float>;
	@:id(2) final upper:Null<Float>;
	@:id(3) final velocity:Null<Float>;
	@:id(4) final effort:Null<Float>;
	@:id(5) @:optional final overtravel:Null<Float>;
	@:id(6) @:optional final acceleration:Null<Float>;
}

@:wire typedef FrozenJoint = {
	@:id(1) final id:String;
	@:id(2) final type:materia.assembly.AssemblyDefinition.AssemblyJointType;
	@:id(3) final role:materia.assembly.AssemblyDefinition.AssemblyJointRole;
	@:id(4) final parent:String;
	@:id(5) final parentConnector:String;
	@:id(6) final child:String;
	@:id(7) final childConnector:String;
	@:id(8) final axis:FrozenVector;
	@:id(9) final limits:FrozenJointLimits;
	@:id(10) final defaultValue:Float;
	@:id(11) @:optional final closureTolerance:Float;
}

@:wire typedef FrozenCoupling = {
	@:id(1) final id:String;
	@:id(2) final source:String;
	@:id(3) final target:String;
	@:id(4) final ratio:Float;
	@:id(5) final offset:Float;
}

@:wire typedef FrozenExposedConnector = {
	@:id(1) final name:String;
	@:id(2) final occurrence:String;
	@:id(3) final connector:String;
}

@:wire typedef FrozenAssemblySubdefinition = {
	@:id(1) final id:String;
	@:id(2) final definitions:ReadOnlyArray<FrozenComponentDefinition>;
	@:id(3) final occurrences:ReadOnlyArray<FrozenOccurrence>;
	@:id(4) final joints:ReadOnlyArray<FrozenJoint>;
	@:id(5) @:optional final couplings:ReadOnlyArray<FrozenCoupling>;
	@:id(6) @:optional final exposedConnectors:ReadOnlyArray<FrozenExposedConnector>;
}

@:wire typedef FrozenAssemblyDefinition = {
	@:id(1) final schemaVersion:Int;
	@:id(2) final id:String;
	@:id(3) @:optional final lengthUnit:String;
	@:id(4) final definitions:ReadOnlyArray<FrozenComponentDefinition>;
	@:id(5) final occurrences:ReadOnlyArray<FrozenOccurrence>;
	@:id(6) final joints:ReadOnlyArray<FrozenJoint>;
	@:id(7) @:optional final couplings:ReadOnlyArray<FrozenCoupling>;
	@:id(8) @:optional final assemblies:ReadOnlyArray<FrozenAssemblySubdefinition>;
	@:id(9) @:optional final exposedConnectors:ReadOnlyArray<FrozenExposedConnector>;
}

/** Serializable recipe inputs. The constructor IDs are part of the wire schema. */
@:wire enum MemberSource {
	@:id(1) Typed(typeId:String, values:ReadOnlyArray<NamedValue>);
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

@:wire typedef PortRecord = {
	@:id(1) var occurrence:String;
	@:id(2) var name:String;
	@:id(3) var kind:PortKind;
	@:id(4) var role:PortRole;
	@:id(5) var iface:PortInterface;
	@:id(6) var required:Bool;
	@:id(7) @:optional var connector:String;
}

@:wire typedef ServiceLinkRecord = {
	@:id(1) var occurrence:String;
	@:id(2) var fromPort:String;
	@:id(3) var toPort:String;
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

@:wire typedef EndEffectorRecord = {
	@:id(1) var mount:Null<ConnectorReference>;
	@:id(2) var frames:ReadOnlyArray<ConnectorExposureRecord>;
	@:id(3) var primaryFrame:Null<String>;
	@:id(4) var collisionExclusions:ReadOnlyArray<String>;
}

@:wire typedef ChangerPortRecord = {
	@:id(1) var robot:String;
	@:id(2) var tool:String;
}

@:wire typedef ChangerRecord = {
	@:id(1) var name:String;
	@:id(2) var instanceId:String;
	@:id(3) var connectorName:String;
	@:id(4) var ports:ReadOnlyArray<ChangerPortRecord>;
}

@:wire typedef ToolRecord = {
	@:id(1) var id:String;
	@:id(2) var mechanical:FrozenAssemblyDefinition;
	@:id(3) var machine:ToolSideRecord;
}

@:wire typedef IncludedRecord = {
	@:id(1) var id:String;
	@:id(2) var pose:AssemblyFrame;
	@:id(3) var mechanical:FrozenAssemblyDefinition;
}

/** A tool has its own EOAT data, with no recursive changer table. */
@:wire typedef ToolSideRecord = {
	@:id(1) var members:ReadOnlyArray<MemberRecord>;
	@:id(2) var portConnections:ReadOnlyArray<PortConnectionRecord>;
	@:id(3) var portExposures:ReadOnlyArray<PortExposureRecord>;
	@:id(4) var bomExtras:ReadOnlyArray<BomExtraRecord>;
	@:id(5) var connectorExposures:ReadOnlyArray<ConnectorExposureRecord>;
	@:id(6) var memberConnectors:ReadOnlyArray<MemberConnectorRecord>;
	@:id(7) var endEffector:EndEffectorRecord;
	@:id(8) var ports:ReadOnlyArray<PortRecord>;
	@:id(9) var included:ReadOnlyArray<IncludedRecord>;
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

/**
 * A coupling whose ratio its parts set (see `Drive`): `kind` is "lead-screw", "gear-mesh",
 * "rack-and-pinion" or "belt", `members` the parts in the order the drive names them, and the
 * follower sits at zero where the leader is at `leaderZero`.
 */
@:wire typedef DriveRecord = {
	@:id(1) var coupling:String;
	@:id(2) var kind:String;
	@:id(3) var members:ReadOnlyArray<String>;
	@:id(4) var alignment:Float;
	@:id(5) var leaderZero:Float;
}

@:wire typedef AssemblySideRecord = {
	@:id(1) var members:ReadOnlyArray<MemberRecord>;
	@:id(2) var portConnections:ReadOnlyArray<PortConnectionRecord>;
	@:id(3) var portExposures:ReadOnlyArray<PortExposureRecord>;
	@:id(4) var bomExtras:ReadOnlyArray<BomExtraRecord>;
	@:id(5) var connectorExposures:ReadOnlyArray<ConnectorExposureRecord>;
	@:id(6) var memberConnectors:ReadOnlyArray<MemberConnectorRecord>;
	@:id(7) @:optional var endEffector:EndEffectorRecord;
	@:id(8) @:optional var changer:ChangerRecord;
	@:id(9) @:optional var tools:ReadOnlyArray<ToolRecord>;
	@:id(10) var ports:ReadOnlyArray<PortRecord>;
	@:id(11) var included:ReadOnlyArray<IncludedRecord>;
	@:id(12) @:optional var drives:ReadOnlyArray<DriveRecord>;
}

/** Mechanical definition plus the MachineKit facts keyed by occurrence ID. */
@:wire typedef MachineAssemblyDescription = {
	@:id(1) var mechanical:FrozenAssemblyDefinition;
	@:id(2) var machine:AssemblySideRecord;
	@:id(3) @:optional var schemaVersion:Int;
}
