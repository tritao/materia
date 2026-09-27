import haxe.io.Bytes;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;
import materia.project.Appearance.Appearances;
import materia.project.Appearance;
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
			var finish = switch (entry.id) {
					case "motor": Appearances.painted();
					case "plate": Appearances.aluminium();
					case "shaft", "key", "ring": Appearances.machinedSteel();
					case "bearingA", "bearingB": Appearances.bearingSteel();
					default: Appearances.blackOxide();
				};
			var color = switch (entry.id) {
					case "motor": [0.24, 0.34, 0.43];
					case "plate": [0.68, 0.70, 0.72];
					case "bearingA", "bearingB": [0.58, 0.61, 0.64];
					case "shaft", "key", "ring": [0.62, 0.64, 0.67];
					default: [0.18, 0.19, 0.20];
				};
			addPart(parts, entry.id, entry.component.designation, part, color, finish);
		}
		for (bearingId in ["bearingA", "bearingB"]) {
			var shieldId = bearingId + "-shields";
			model.add(shieldId);
			model.connector(shieldId, "front", Solids.axial(0, 0, 0));
			model.mate(shieldId + "-seat", "fixed", bearingId, "front", shieldId, "front");
			addPart(parts, shieldId, example.bearing.designation + " shields",
				example.bearing.shieldGeometry(), [0.20, 0.23, 0.26], Appearances.blackOxide());
		}
		return SceneArtifact.encode({metresPerUnit: 0.001, parts: parts,
			assembly: model.record(), assemblyDefinition: model.definition("motor-shaft-bearings"),
			assemblyState: model.initialState("motor-shaft-bearings").record()});
	}

	static function addPart(parts:Array<SceneArtifactPart>, id:String, name:String, part:Part,
			color:Array<Float>, finish:Appearance):Void {
		try {
			var mesh = part.shape.tessellate(0.5, 0.5);
			parts.push({
				id: id, name: name, red: color[0], green: color[1], blue: color[2],
				appearance: finish, vertexCount: mesh.vertexCount, indexCount: mesh.indexCount,
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
