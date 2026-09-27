import haxe.io.Bytes;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;
import materia.project.MaterialLibrary;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import cadkit.modeling.Part;

/** Materia project entrypoint for the MachineKit motor/shaft/bearing assembly. */
class MotorShaftBearingsPreview {
	public static function preview():Bytes {
		var example = new MotorShaftBearings();
		var model = example.assembly();
		var parts:Array<SceneArtifactPart> = [];
		for (entry in example.components()) {
			var part = entry.component.geometry(ComponentDetail.Preview);
			addPart(parts, entry.id, entry.component.designation, part, entry.component.materialId);
		}
		for (bearingId in ["bearingA", "bearingB"]) {
			var shieldId = bearingId + "-shields";
			model.add(shieldId);
			model.connector(shieldId, "front", Solids.axial(0, 0, 0));
			model.mate(shieldId + "-seat", "fixed", bearingId, "front", shieldId, "front");
			addPart(parts, shieldId, example.bearing.designation + " shields",
				example.bearing.shieldGeometry(), "black-oxide");
		}
		return SceneArtifact.encode({metresPerUnit: 0.001, parts: parts,
			assembly: model.record(), assemblyDefinition: model.definition("motor-shaft-bearings"),
			assemblyState: model.initialState("motor-shaft-bearings").record()});
	}

	static function addPart(parts:Array<SceneArtifactPart>, id:String, name:String, part:Part,
			materialId:String):Void {
		var material = MaterialLibrary.require(materialId);
		var color = material.visual.baseColor;
		try {
			var mesh = part.shape.tessellate(0.5, 0.5);
			parts.push({
				id: id, name: name, red: color[0], green: color[1], blue: color[2],
				appearance: MaterialLibrary.appearance(materialId), materialId: materialId, vertexCount: mesh.vertexCount, indexCount: mesh.indexCount,
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
	if (scene.parts.length != 13 || scene.assemblyDefinition == null)
		throw "Bearing preview has missing render regions";
	for (part in scene.parts) if (part.materialId == null || part.materialDensity == null ||
		part.materialDensity <= 0 || part.materialSpec == null)
		throw 'Bearing preview has no resolved material for "${part.id}"';
	for (bearingId in ["bearingA", "bearingB"]) {
		var shieldId = bearingId + "-shields";
		var shield:Null<SceneArtifactPart> = null;
		for (part in scene.parts) if (part.id == shieldId) shield = part;
		if (shield == null || shield.appearance == null || shield.appearance.finish != "black-oxide")
			throw 'Bearing preview has no separate shield finish for "$bearingId"';
		var attached = false;
		for (joint in scene.assemblyDefinition.joints)
			if (joint.parent == bearingId && joint.child == shieldId) attached = true;
		if (!attached) throw 'Bearing shield "$shieldId" does not follow its bearing';
	}
	Sys.println('Generated ${scene.parts.length} preview regions');
}
