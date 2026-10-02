package machinekit.assembly;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.project.MaterialLibrary;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.SceneArtifact.SceneArtifactPart;
import materia.project.SceneArtifact.SceneArtifactRobotTool;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorControls;
import machinekit.welding.WeldingEquipment.WeldingEquipmentData;
import machinekit.welding.WeldingTorch;
import materia.units.LengthUnit;

/**
 * Builds Materia scene artifacts from MachineKit assemblies. Occurrences whose components share a
 * designation share one geometry part, so a scene definition may stand for several occurrences;
 * `shareDefinitions` keeps the connectors of all of them on it.
 */
class AssemblyPreview {
	/** Scene of `assembly` in millimetres, with its joints and initial pose; `assemblyId` names the definition. */
	public static function scene(assembly:MachineAssembly, assemblyId:String):SceneArtifactData {
		assembly.validate();
		var model = new AssemblyModel("mm");
		assembly.addTo(model, "");
		var parts:Array<SceneArtifactPart> = [];
		var definitionByOccurrence = new Map<String, String>();
		var definitionByDesignation = new Map<String, String>();
		for (entry in assembly.components()) {
			var definitionId = definitionByDesignation.get(entry.component.designation);
			if (definitionId == null) {
				definitionId = entry.id;
				definitionByDesignation.set(entry.component.designation, definitionId);
				parts.push(part(definitionId, entry.component.designation,
					entry.component.geometry(ComponentDetail.Preview), entry.component.materialId));
			}
			definitionByOccurrence.set(entry.id, definitionId);
		}
		var definition = model.definition(assemblyId);
		var state = model.initialState(assemblyId).record();
		shareDefinitions(definition, definitionByOccurrence);
		return {lengthUnit: "mm", metresPerUnit: LengthUnit.metresPerUnit("mm"), parts: parts,
			assembly: model.record(), assemblyDefinition: definition, assemblyState: state};
	}

	/**
	 * Points every occurrence at its shared definition (`definitionByOccurrence`; an occurrence missing
	 * from it keeps its own) and drops the definitions nothing points at any more. A kept definition
	 * takes the connectors of every occurrence that shares it; two occurrences may name the same
	 * connector only with the same frame, since the scene holds one frame per name.
	 */
	/**
	 * A robot's tools as its end effector `tool` declares them, its members' occurrences under
	 * `prefix`: each suction cup's contact, worked by the effector's vacuum control channel and
	 * reporting on its pressure sensor when it has one.
	 */
	public static function robotTools(tool:EndEffector, prefix:String, ?welding:WeldingEquipmentData):Array<SceneArtifactRobotTool> {
		var controls = EndEffectorControls.derive(tool, prefix);
		if (controls.arcs.length > 0) return torchTools(controls, prefix, welding);
		var channel = controls.vacuumChannel();
		if (channel == null) throw "The suction tool has no vacuum control";
		return [for (suction in controls.suctions) {
			var entry:SceneArtifactRobotTool = {kind: "suction",
				contact: {occurrence: prefix + "/" + suction.member, connector: suction.connector}, channel: channel};
			if (controls.vacuumSensor != null) entry.sensor = controls.vacuumSensor;
			entry;
		}];
	}

	/**
	 * Each arc torch as a `torch` tool: its wire tip as the contact, its three channels and weld sensor, and the
	 * welder behind it (`welding`: the supply's limits, the wire, and the work the circuit returns through, found by
	 * `WeldingEquipment.of`).
	 */
	static function torchTools(controls:EndEffectorControls, prefix:String, welding:Null<WeldingEquipmentData>):Array<SceneArtifactRobotTool> {
		if (welding == null) throw "A torch needs the welding equipment of its cell: pass WeldingEquipment.of(...)";
		return [for (arc in controls.arcs) {
			kind: "torch",
			contact: {occurrence: prefix + "/" + arc.member, connector: arc.tcpConnector},
			channel: arc.channel,
			sensor: arc.sensor,
			torch: {wireSpeedChannel: arc.wireSpeedChannel, voltageChannel: arc.voltageChannel,
				groundedWork: welding.groundedWork.copy(), maxCurrentA: welding.maxCurrentA, efficiency: welding.efficiency,
				wireDiameterMm: welding.wireDiameterMm, stickoutMm: WeldingTorch.STICKOUT,
				maxWireSpeedMPerMin: welding.maxWireSpeedMPerMin, depositionEfficiency: welding.depositionEfficiency}
		}];
	}

	public static function shareDefinitions(definition:AssemblyDefinition, definitionByOccurrence:Map<String, String>):Void {
		function sharedId(id:String):String {
			var shared = definitionByOccurrence.get(id);
			return shared == null ? id : shared;
		}
		var kept = new Map<String, AssemblyComponentDefinition>();
		for (component in definition.definitions) if (sharedId(component.id) == component.id) kept.set(component.id, component);
		for (component in definition.definitions) {
			var owner = kept.get(sharedId(component.id));
			if (owner == null) throw 'Shared definition "${sharedId(component.id)}" is not in the assembly';
			if (owner == component) continue;
			for (connector in component.connectors) {
				var existing = [for (candidate in owner.connectors) if (candidate.name == connector.name) candidate];
				if (existing.length == 0) owner.connectors.push(connector);
				else if (!sameFrame(existing[0].frame, connector.frame))
					throw 'Occurrences sharing "${owner.id}" give connector "${connector.name}" different frames';
			}
		}
		definition.definitions = [for (component in definition.definitions) if (kept.exists(component.id)) component];
		for (occurrence in definition.occurrences) occurrence.definition = sharedId(occurrence.id);
	}

	/**
	 * One scene part: tessellated mesh, material colour and mass properties. Closes `part`. The mesh
	 * deviates from the solid by a small fraction of its size, or by at most `deflection` (in the part's
	 * units) when given, for a mesh that is measured against rather than only looked at.
	 */
	public static function part(id:String, name:String, part:Part, materialId:String, ?deflection:Float):SceneArtifactPart {
		var material = MaterialLibrary.require(materialId);
		var color = material.visual.baseColor;
		try {
			var physical = part.massProperties();
			var mesh = deflection == null ? part.shape.tessellateRelative() : part.shape.tessellate(deflection);
			var result:SceneArtifactPart = {
				id: id, name: name, red: color[0], green: color[1], blue: color[2],
				appearance: MaterialLibrary.appearance(materialId), materialId: materialId,
				vertexCount: mesh.vertexCount, indexCount: mesh.indexCount,
				volume: physical.volume,
				centerOfMass: [physical.centerOfMass.x, physical.centerOfMass.y, physical.centerOfMass.z],
				vertices: mesh.vertices, normals: mesh.normals, indices: mesh.indices,
				edgeSegments: mesh.edgeSegments, edgeIds: mesh.edgeIds,
				faceDescriptors: cadkit.parametric.GeometricConnectors.describeFaces(part.shape),
				faceRanges: [for (range in mesh.faceRanges) {
					faceIndex: range.faceIndex, firstIndex: range.firstIndex, indexCount: range.indexCount
				}]
			};
			part.close();
			return result;
		} catch (error:Dynamic) {
			part.close();
			throw error;
		}
	}

	static function sameFrame(a:materia.assembly.AssemblyRecord.AssemblyFrame, b:materia.assembly.AssemblyRecord.AssemblyFrame):Bool {
		var tolerance = 1e-9;
		return Math.abs(a.x - b.x) <= tolerance && Math.abs(a.y - b.y) <= tolerance && Math.abs(a.z - b.z) <= tolerance &&
			Math.abs(a.qx - b.qx) <= tolerance && Math.abs(a.qy - b.qy) <= tolerance && Math.abs(a.qz - b.qz) <= tolerance &&
			Math.abs(a.qw - b.qw) <= tolerance;
	}
}
