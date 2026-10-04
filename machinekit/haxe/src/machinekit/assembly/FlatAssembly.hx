package machinekit.assembly;

import machinekit.component.BomItem;
import machinekit.component.MachineComponent;
import machinekit.assembly.MachineAssembly.AssemblyBomMass;
import machinekit.assembly.MachineAssembly.MachineAssemblyComponent;
import machinekit.assembly.MachineAssemblyDescription.BeltPathRecord;
import machinekit.assembly.MachineAssemblyDescription.EncoderRecord;
import machinekit.assembly.MachineAssemblyDescription.MotorRecord;
import machinekit.assembly.MachineAssemblyDescription.PortConnectionRecord;
import machinekit.assembly.MachineAssemblyDescription.PortExposureRecord;
import machinekit.assembly.MachineAssemblyDescription.TransmissionRecord;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef BomEntry = {var item:BomItem; var quantity:Int; var mass:AssemblyBomMass;}

/**
 * An assembly and all of its subassemblies in one namespace of member paths. Checks, transmissions,
 * motors, mass and every export work on this view; `MachineAssembly` builds it and drops it on edit.
 */
class FlatAssembly {
	public final members:Array<MachineAssemblyComponent> = [];
	/** Occurrences, connectors, joints and couplings with path ids; derived couplings once `derived`. */
	public final definition:AssemblyDefinition;
	public final connections:Array<PortConnectionRecord> = [];
	/** The ports the assembly itself exposes; a subassembly's own exposures are internal here. */
	public final portExposures:Array<PortExposureRecord> = [];
	public final transmissions:Array<TransmissionRecord> = [];
	public final beltPaths:Array<BeltPathRecord> = [];
	public final motors:Array<MotorRecord> = [];
	public final encoders:Array<EncoderRecord> = [];
	public final cylinders:Array<machinekit.assembly.MachineAssemblyDescription.CylinderRecord> = [];
	public final sensors:Array<materia.assembly.AssemblyDefinition.AssemblySensor> = [];
	public final bomItems:Array<BomEntry> = [];
	/** Findings from working the transmission couplings out from their parts. */
	public final diagnostics:Diagnostics = new Diagnostics();
	/** Whether `DriveSystem.derive` has written the transmissions into `definition`. */
	public var derived:Bool = false;
	final memberById:Map<String, MachineComponent> = [];
	final definitionById:Map<String, AssemblyComponentDefinition> = [];

	public function new() {
		definition = {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "assembly", lengthUnit: "mm",
			definitions: [], occurrences: [], joints: [], couplings: []};
	}

	public function addMember(id:String, component:MachineComponent, pose:AssemblyFrame):Void {
		members.push({id: id, component: component});
		memberById.set(id, component);
		var entry:AssemblyComponentDefinition = {id: id, connectors: [for (connector in component.connectors())
			{name: connector.name, frame: MachineAssembly.copyFrame(connector.frame)}]};
		definition.definitions.push(entry);
		definitionById.set(id, entry);
		definition.occurrences.push({id: id, definition: id, initialPose: pose});
	}

	public function addConnector(id:String, name:String, frame:AssemblyFrame):Void
		requireDefinition(id).connectors.push({name: name, frame: MachineAssembly.copyFrame(frame)});

	public function hasMember(id:String):Bool return memberById.exists(id);

	public function member(id:String):MachineComponent {
		var component = memberById.get(id);
		if (component == null) throw 'Unknown assembly member "$id"';
		return component;
	}

	public function requireDefinition(id:String):AssemblyComponentDefinition {
		var entry = definitionById.get(id);
		if (entry == null) throw 'Missing mechanical definition "$id"';
		return entry;
	}

	public function occurrencePose(id:String):AssemblyFrame {
		for (occurrence in definition.occurrences) if (occurrence.id == id) return occurrence.initialPose;
		throw 'Missing mechanical occurrence "$id"';
	}
}
