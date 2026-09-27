import haxe.io.Bytes;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;
import materia.project.MaterialLibrary;
import materia.units.LengthUnit;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import cadkit.modeling.Part;

/** Materia project entrypoint for the MachineKit motor/shaft/bearing assembly. */
class MotorShaftBearingsPreview {
	public static function preview():Bytes {
		var example = new MotorShaftBearings();
		var model = example.assembly();
		var parts:Array<SceneArtifactPart> = [];
		var definitionByOccurrence = new Map<String, String>();
		var definitionByIdentity = new Map<String, String>();
		for (entry in example.components()) {
			var recipe = entry.component.type;
			var identity = recipe == null ? entry.component.designation : recipe.key(entry.component.values());
			var definitionId = definitionByIdentity.get(identity);
			if (definitionId == null) {
				definitionId = entry.id;
				definitionByIdentity.set(identity, definitionId);
				var part = entry.component.geometry(ComponentDetail.Preview);
				addPart(parts, definitionId, entry.component.designation, part, entry.component.materialId);
			}
			definitionByOccurrence.set(entry.id, definitionId);
		}
		for (bearingId in ["bearingA", "bearingB"]) {
			var shieldId = bearingId + "-shields";
			model.add(shieldId);
			model.connector(shieldId, "front", Solids.axial(0, 0, 0));
			model.mate(shieldId + "-seat", "fixed", bearingId, "front", shieldId, "front");
			definitionByOccurrence.set(shieldId, "bearingA-shields");
			if (bearingId == "bearingA") addPart(parts, shieldId,
				example.bearing.designation + " shields", example.bearing.shieldGeometry(), "black-oxide");
		}
		var definition = model.definition("motor-shaft-bearings");
		var state = model.initialState("motor-shaft-bearings").record();
		definition.definitions = [for (component in definition.definitions)
			if (definitionByOccurrence.get(component.id) == component.id) component];
		for (occurrence in definition.occurrences)
			occurrence.definition = definitionByOccurrence.get(occurrence.id);

		return SceneArtifact.encode({lengthUnit: "mm",
			metresPerUnit: LengthUnit.metresPerUnit("mm"), parts: parts,
			assembly: model.record(), assemblyDefinition: definition,
			assemblyState: state});
	}

	static function addPart(parts:Array<SceneArtifactPart>, id:String, name:String, part:Part,
			materialId:String):Void {
		var material = MaterialLibrary.require(materialId);
		var color = material.visual.baseColor;
		try {
			var physical = part.massProperties();
			var mesh = part.shape.tessellateRelative();
			parts.push({
				id: id, name: name, red: color[0], green: color[1], blue: color[2],
				appearance: MaterialLibrary.appearance(materialId), materialId: materialId, vertexCount: mesh.vertexCount, indexCount: mesh.indexCount,
				volume: physical.volume,
				centerOfMass: [physical.centerOfMass.x, physical.centerOfMass.y, physical.centerOfMass.z],
				vertices: mesh.vertices, normals: mesh.normals, indices: mesh.indices,
				edgeSegments: mesh.edgeSegments, edgeIds: mesh.edgeIds,
				faceRanges: [for (range in mesh.faceRanges) {
					faceIndex: range.faceIndex, firstIndex: range.firstIndex, indexCount: range.indexCount
				}]
			});
		} catch (error:Dynamic) {
			part.close();
			throw error;
		}
		part.close();
	}
}

function main():Void {
	var scene = SceneArtifact.decode(MotorShaftBearingsPreview.preview());
	if (scene.parts.length != 8 || scene.assemblyDefinition == null ||
		scene.assemblyDefinition.occurrences.length != 13)
		throw "Bearing preview has missing render regions";
	for (part in scene.parts) if (part.materialId == null || part.materialDensity == null ||
		part.materialDensity <= 0 || part.materialSpec == null || part.volume == null ||
		part.volume <= 0 || part.centerOfMass == null || part.inertia == null || part.inertia.length != 9)
		throw 'Bearing preview has no resolved material for "${part.id}"';
	var shield:Null<SceneArtifactPart> = null;
	for (part in scene.parts) if (part.id == "bearingA-shields") shield = part;
	if (shield == null || shield.appearance == null || shield.appearance.finish != "black-oxide")
		throw "Bearing preview has no shared shield finish";
	for (bearingId in ["bearingA", "bearingB"]) {
		var shieldId = bearingId + "-shields";
		var attached = false, shared = false;
		for (joint in scene.assemblyDefinition.joints)
			if (joint.parent == bearingId && joint.child == shieldId) attached = true;
		for (occurrence in scene.assemblyDefinition.occurrences)
			if (occurrence.id == shieldId && occurrence.definition == "bearingA-shields") shared = true;
		if (!attached || !shared) throw 'Bearing shield "$shieldId" does not share its geometry';
	}
	Sys.println('Generated ${scene.parts.length} preview regions');
}
