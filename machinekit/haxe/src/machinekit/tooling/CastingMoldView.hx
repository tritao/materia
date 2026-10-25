package machinekit.tooling;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.presentation.AssemblyPresentation;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.SceneArtifact.SceneArtifactPart;

/** A reference rendering of the casting, separate from physical mold members. */
class CastingMoldView {
	public static inline var REFERENCE_ID = "finished-casting";
	public static inline var AS_POURED_ID = "finished-casting-as-poured";
	public static inline var LIFTING_ID = "finished-casting-lifting-points";

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

	public static function presentation(scene:SceneArtifactData, embedded:Array<String>,
			moldToService:materia.assembly.AssemblyRecord.AssemblyFrame, liftingIds:Array<String>):Void {
		materia.assembly.AssemblyCodec.validateFrame(moldToService);
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
		presentation.views.push({id:REFERENCE_ID, label:"Finished casting — installed",
			camera:Frame([for (id in shown) {occurrence:id, subtree:false}], -1.1, 0.35, 1.15),
			visibility:[for (part in flat.occurrences) {
				select:{occurrence:part.id, subtree:false}, mode:shown.indexOf(part.id) >= 0 ? Shown : Hidden
			}], sections:[], explode:0, source:Generated("casting-mold")});
		var installed = presentation.views[presentation.views.length - 1];
		installed.displayTransform = moldToService;
		presentation.views.push({id:AS_POURED_ID, label:"Finished casting — as poured",
			camera:installed.camera, visibility:[for (rule in installed.visibility) {
				select:{occurrence:rule.select.occurrence, subtree:rule.select.subtree}, mode:rule.mode
			}], sections:[], explode:0,
			source:Generated("casting-mold")});
		if (liftingIds.length > 0) {
			for (id in liftingIds) {
				if (embedded.indexOf(id) < 0) throw "Lifting point is not a source insert: " + id;
				for (part in scene.parts) if (part.id == id)
					part.name = "Lifting point · " + id + " (capacity pending)";
			}
			presentation.views.push({id:LIFTING_ID, label:"Lifting points — capacity pending",
				camera:installed.camera, displayTransform:moldToService,
				visibility:[for (part in flat.occurrences) {
					select:{occurrence:part.id, subtree:false},
					mode:part.id == REFERENCE_ID ? Transparent : liftingIds.indexOf(part.id) >= 0 ? Shown : Hidden
				}], sections:[], explode:0, source:Generated("casting-mold")});
			presentation.procedures.push({id:"locate-casting-lifting-points", label:"Locate lifting points (unqualified)",
				kind:Service, start:Assembled, source:Generated("casting-mold"), steps:[{
					id:"identify-lifting-connections", title:"Source lifting connections",
					instruction:"Identification only. Anchorage capacity, rated lifting hardware, rigging and the lift/flip procedure remain Pending.",
					view:LIFTING_ID, highlight:[for (id in liftingIds) {occurrence:id, subtree:false}], action:Note
				}]});
		}
		materia.presentation.PresentationCodec.validate(scene.assemblyDefinition, presentation);
	}
}
