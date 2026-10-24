package machinekit.tooling;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.presentation.AssemblyPresentation;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.SceneArtifact.SceneArtifactPart;

/** A reference rendering of the casting, separate from physical mold members. */
class CastingMoldView {
	public static inline var REFERENCE_ID = "finished-casting";

	/** Add before generating presentations so their evidence fingerprints include
	 * the complete scene. This reference contributes no extra mass or collision.
	 */
	public static function geometry(scene:SceneArtifactData, body:SceneArtifactPart):Void {
		var definition = scene.assemblyDefinition;
		if (definition == null || scene.parts.filter(part -> part.id == REFERENCE_ID).length > 0)
			throw "Finished casting requires a unique assembly reference";
		body.id = REFERENCE_ID;
		body.name = "Finished casting";
		body.physicalMassKg = 0;
		body.physicalMassDeclared = true;
		body.physicalInertiaKgM2 = [for (index in 0...9) 0.0];
		scene.parts.push(body);
		definition.definitions.push({id:REFERENCE_ID, connectors:[], collisionHulls:[]});
		definition.occurrences.push({id:REFERENCE_ID, definition:REFERENCE_ID,
			initialPose:F.identity(), grounded:true});
	}

	public static function presentation(scene:SceneArtifactData, embedded:Array<String>):Void {
		var presentation = scene.presentation;
		if (presentation == null || scene.assemblyDefinition == null)
			throw "Finished casting requires the mold presentation";
		var flat = AssemblyDefinitionFlattener.flattenView(scene.assemblyDefinition);
		var shown = [REFERENCE_ID].concat(embedded);
		for (id in shown)
			if (flat.occurrences.filter(part -> part.id == id).length != 1)
				throw "Unknown finished casting member: " + id;
		for (view in presentation.views)
			view.visibility.push({select:{occurrence:REFERENCE_ID, subtree:false}, mode:Hidden});
		presentation.views.push({id:REFERENCE_ID, label:"Finished casting",
			camera:Frame([for (id in shown) {occurrence:id, subtree:false}], -1.1, 0.35, 1.15),
			visibility:[for (part in flat.occurrences) {
				select:{occurrence:part.id, subtree:false}, mode:shown.indexOf(part.id) >= 0 ? Shown : Hidden
			}], sections:[], explode:0, source:Generated("casting-mold")});
		materia.presentation.PresentationCodec.validate(scene.assemblyDefinition, presentation);
	}
}
