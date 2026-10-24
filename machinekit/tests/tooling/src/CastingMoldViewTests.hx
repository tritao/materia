import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyDefinition;
import materia.presentation.PresentationEvaluator;
import materia.presentation.PresentationGeometry;
import materia.presentation.Presentations;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;
import machinekit.tooling.CastingMoldView;

class CastingMoldViewTests {
	static var assertions = 0;
	static function check(value:Bool, message:String):Void {
		assertions++;
		if (!value) throw message;
	}
	static function mesh(id:String):SceneArtifactPart {
		var vertices = haxe.io.Bytes.alloc(96), normals = haxe.io.Bytes.alloc(96);
		var indices = haxe.io.Bytes.alloc(48);
		vertices.setDouble(24, 1); vertices.setDouble(56, 1); vertices.setDouble(88, 1);
		var triangles = [0,2,1,0,1,3,0,3,2,1,2,3];
		for (i in 0...4) normals.setDouble(i * 24 + 16, 1);
		for (i in 0...triangles.length) indices.setInt32(i * 4, triangles[i]);
		return {id:id, name:id, red:0.5, green:0.5, blue:0.5, vertexCount:4, indexCount:12,
			vertices:vertices, normals:normals, indices:indices, faceRanges:[]};
	}
	static function main():Void {
		var ids = ["bed", "insert-seat", "anchor-bolt"];
		var definition:AssemblyDefinition = {schemaVersion:materia.assembly.AssemblyDefinitionCodec.VERSION,
			id:"mold", definitions:[for (id in ids) {id:id, connectors:[]}],
			occurrences:[for (id in ids) {id:id, definition:id, initialPose:F.identity()}], joints:[]};
		var scene:materia.project.SceneArtifact.SceneArtifactData = {
			metresPerUnit:0.001, parts:[for (id in ids) mesh(id)], assemblyDefinition:definition};
		var original = scene.parts[0].vertices.sub(0, scene.parts[0].vertices.length);
		CastingMoldView.geometry(scene, mesh("casting"));
		var fingerprint = PresentationGeometry.fingerprint(scene);
		var presentation = Presentations.assembled(definition);
		scene.presentation = presentation;
		CastingMoldView.presentation(scene, ["insert-seat", "anchor-bolt"]);
		check(presentation.views.length == 2, "Finished casting is an additional dropdown view");
		check(presentation.defaultView == "assembled", "Mold view remains the default");
		check(PresentationGeometry.fingerprint(scene) == fingerprint, "View generation preserves motion geometry fingerprints");
		check(scene.parts[0].vertices.compare(original) == 0, "Existing tooling meshes remain intact");
		check(scene.parts[3].physicalMassKg == 0, "Reference casting adds no simulation mass");
		var hulls = definition.definitions[3].collisionHulls;
		check(hulls != null && hulls.length == 0, "Reference casting adds no collision hull");
		var solved = new Map<String,materia.assembly.AssemblyRecord.AssemblyFrame>();
		for (part in definition.occurrences) solved.set(part.id, part.initialPose);
		var cursor = PresentationEvaluator.reset(presentation);
		var frame = PresentationEvaluator.evaluate(definition, solved, presentation, cursor, fingerprint);
		check(frame.visibility.get(CastingMoldView.REFERENCE_ID) == Hidden, "Mold view hides the casting reference");
		check(frame.visibility.get("bed") == Shown, "Mold view still shows its tooling");
		cursor.view = CastingMoldView.REFERENCE_ID;
		frame = PresentationEvaluator.evaluate(definition, solved, presentation, cursor, fingerprint);
		check(frame.visibility.get("bed") == Hidden, "Finished view hides mold tooling");
		for (id in [CastingMoldView.REFERENCE_ID, "insert-seat", "anchor-bolt"])
			check(frame.visibility.get(id) == Shown, "Finished view retains " + id);
		var restored = SceneArtifact.decode(SceneArtifact.encode(scene));
		var restoredPresentation = restored.presentation;
		if (restoredPresentation == null) throw "Missing restored presentation";
		check(restoredPresentation.views[1].label == "Finished casting", "Finished view survives artifact encoding");
		check(PresentationGeometry.fingerprint(restored) == fingerprint, "Artifact round trip retains the complete geometry fingerprint");
		Sys.println('CastingMoldViewTests: $assertions assertions passed');
	}
}
