package machinekit.robotics;

import machinekit.assembly.MachineAssemblyDescription;
import machinekit.assembly.MachineAssemblyDescription.ConnectorExposureRecord;
import machinekit.assembly.MachineAssemblyDescription.ConnectorReference;

@:wire typedef EndEffectorRecord = {
	@:id(1) var mount:Null<ConnectorReference>;
	@:id(2) var frames:Array<ConnectorExposureRecord>;
	@:id(3) var primaryFrame:Null<String>;
	@:id(4) var collisionExclusions:Array<String>;
}

/** An end effector as portable data: its assembly, and its mount and working frames. */
@:wire typedef EndEffectorDescription = {
	@:id(1) var assembly:MachineAssemblyDescription;
	@:id(2) var endEffector:EndEffectorRecord;
}

@:wire typedef ChangerPortRecord = {
	@:id(1) var robot:String;
	@:id(2) var tool:String;
}

@:wire typedef ChangerRecord = {
	@:id(1) var name:String;
	@:id(2) var instanceId:String;
	@:id(3) var connectorName:String;
	@:id(4) var ports:Array<ChangerPortRecord>;
}

@:wire typedef ChangerToolRecord = {
	@:id(1) var id:String;
	@:id(2) var tool:EndEffectorDescription;
}

/** A robot-side end effector with a tool changer, and the tools it can couple. */
@:wire typedef EndEffectorSetDescription = {
	@:id(1) var base:EndEffectorDescription;
	@:id(2) var changer:ChangerRecord;
	@:id(3) var tools:Array<ChangerToolRecord>;
}
