import haxe.io.Bytes;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;

/** Project entrypoint that exposes code-authored CAD parts as a scene artifact. */
class ProceduralExcavatorPreview {
	public static function preview():Bytes {
		var components = ProceduralExcavator.build();
		var parts:Array<SceneArtifactPart> = [];
		try {
			var colors = [[0.77, 0.53, 0.26], [0.72, 0.31, 0.19], [0.35, 0.48, 0.63],
				[0.72, 0.66, 0.48], [0.28, 0.34, 0.40], [0.84, 0.76, 0.57]];
			for (index in 0...components.length) {
				var component = components[index];
				var mesh = component.part.shape.tessellate(1.0, 0.7);
				var color = colors[index % colors.length];
				parts.push({
					id: "excavator/" + component.name,
					name: component.name,
					red: color[0], green: color[1], blue: color[2],
					vertexCount: mesh.vertexCount, indexCount: mesh.indexCount,
					vertices: mesh.vertices, normals: mesh.normals, indices: mesh.indices,
					faceRanges: [for (range in mesh.faceRanges) {
						faceIndex: range.faceIndex, firstIndex: range.firstIndex, indexCount: range.indexCount
					}]
				});
			}
			var result = SceneArtifact.encode({metresPerUnit: 0.001, parts: parts});
			for (component in components) component.close();
			return result;
		} catch (error:Dynamic) {
			for (component in components) component.close();
			throw error;
		}
	}
}

// The Haxeon compiler also emits a module entry function.
function main():Void {}
