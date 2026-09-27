import haxe.io.Bytes;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;
import materia.project.Appearance.Appearances;
import machinekit.component.ComponentDetail;

/** Materia project entrypoint for the MachineKit motor/shaft/bearing assembly. */
class MotorShaftBearingsPreview {
	public static function preview():Bytes {
		var example = new MotorShaftBearings();
		var model = example.assembly();
		var parts:Array<SceneArtifactPart> = [];
		for (entry in example.components()) {
			var part = entry.component.geometry(ComponentDetail.Preview);
			try {
				var mesh = part.shape.tessellate(0.5, 0.5);
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
				parts.push({
					id: entry.id, name: entry.component.designation,
					red: color[0], green: color[1], blue: color[2],
					appearance: finish,
					vertexCount: mesh.vertexCount, indexCount: mesh.indexCount,
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
		return SceneArtifact.encode({metresPerUnit: 0.001, parts: parts,
			assembly: model.record(), assemblyDefinition: model.definition("motor-shaft-bearings"),
			assemblyState: model.initialState("motor-shaft-bearings").record()});
	}
}

function main():Void {}
