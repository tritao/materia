package machinekit.assembly;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import haxe.ds.ReadOnlyArray;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.PortInterface;

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
	@:id(2) var mechanical:AssemblyDefinition;
	@:id(3) var machine:ToolSideRecord;
}

@:wire typedef IncludedRecord = {
	@:id(1) var id:String;
	@:id(2) var pose:AssemblyFrame;
	@:id(3) var mechanical:AssemblyDefinition;
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
	@:id(10) @:optional var portBridges:ReadOnlyArray<ServiceLinkRecord>;
	@:id(11) @:optional var portConversions:ReadOnlyArray<ServiceLinkRecord>;
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
	@:id(12) @:optional var portBridges:ReadOnlyArray<ServiceLinkRecord>;
	@:id(13) @:optional var portConversions:ReadOnlyArray<ServiceLinkRecord>;
}

/** Mechanical definition plus the MachineKit facts keyed by occurrence ID. */
@:wire typedef MachineAssemblyDescription = {
	@:id(1) var mechanical:AssemblyDefinition;
	@:id(2) var machine:AssemblySideRecord;
}
