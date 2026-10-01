import cadkit.Geometry;
import cadkit.Shape;
import cadkit.parametric.GeometricConnectors;
import haxe.io.Bytes;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;
import materia.units.LengthUnit;

/**
	An editor test fixture for mates: a grounded 60 × 40 × 10 mm plate with a radius-5 bore through it at
	(30, 20), and a radius-5, 30 mm pin placed off to the side, with no joints between them. The artifact
	carries face descriptors, so the editor can mate the pin's faces to the plate's.
*/
class PinPlatePreview {
	public static function preview():Bytes {
		var block = Shape.box(60, 40, 10), tool = Shape.cylinder(5, 12);
		var placed = tool.translate(Geometry.vec3(30, 20, -1));
		var plate = block.cut(placed), pin = Shape.cylinder(5, 30);
		for (shape in [block, tool, placed]) shape.close();
		try {
			var parts = [part("plate", "Plate", plate), part("pin", "Pin", pin)];
			var result = SceneArtifact.encode({lengthUnit: "mm", metresPerUnit: LengthUnit.metresPerUnit("mm"), parts: parts,
				assemblyDefinition: {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "pin-plate", lengthUnit: "mm",
					definitions: [{id: "plate", connectors: []}, {id: "pin", connectors: []}],
					occurrences: [{id: "plate", definition: "plate", initialPose: AssemblyFrames.identity(), grounded: true},
						{id: "pin", definition: "pin", initialPose: AssemblyFrames.translation(120, -40, 75)}],
					joints: []}});
			plate.close();
			pin.close();
			return result;
		} catch (error:Dynamic) {
			plate.close();
			pin.close();
			throw error;
		}
	}

	static function part(id:String, name:String, shape:Shape):SceneArtifactPart {
		var mesh = shape.tessellateRelative();
		return {id: id, name: name, red: 0.6, green: 0.6, blue: 0.65, materialId: "machined-steel",
			vertexCount: mesh.vertexCount, indexCount: mesh.indexCount, vertices: mesh.vertices, normals: mesh.normals,
			indices: mesh.indices, edgeSegments: mesh.edgeSegments, edgeIds: mesh.edgeIds,
			faceDescriptors: GeometricConnectors.describeFaces(shape),
			faceRanges: [for (range in mesh.faceRanges) {faceIndex: range.faceIndex, firstIndex: range.firstIndex, indexCount: range.indexCount}]};
	}
}

// The Haxeon compiler also emits a module entry function.
function main():Void {}
