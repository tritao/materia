package app;

import cadkit.parametric.GeometricConnectors;
import materia.assembly.AssemblyDefinition.AssemblyMateKind;

/**
	The two picks of a face mate (plan C4.5c): the viewport hands it each
	clicked face (a scene id and face index); after the second, on another
	part, it adds the mate to the session. A face whose feature cannot take
	the mate (a cylinder for a planar mate, say) is refused at once, so the
	user can pick again. `message` says what to do next or what went wrong.
*/
class MatePickTool {
	public final kind:AssemblyMateKind;
	public var message(default, null):String;
	/** The mate is added: the tool has nothing left to do. */
	public var finished(default, null):Bool = false;
	/** The id of the mate added, once `finished`. */
	public var mateId(default, null):Null<String> = null;
	final session:ProjectDocumentSession;
	var firstId:Null<String> = null;
	var firstFace:Int = -1;

	public function new(session:ProjectDocumentSession, kind:AssemblyMateKind) {
		this.session = session;
		this.kind = kind;
		message = 'Pick the first face for a $kind mate (Esc cancels)';
	}

	/** Offers a clicked face (`sceneId` null or `faceIndex` < 0 for a click on nothing usable). */
	public function pick(sceneId:Null<String>, faceIndex:Int):Void {
		if (finished) return;
		if (sceneId == null || faceIndex < 0 || !StringTools.startsWith(sceneId, "project:")) {
			message = "Pick a face of an assembly part";
			return;
		}
		var feature = session.assemblyFaceFeature(sceneId, faceIndex);
		if (feature == null) {
			message = "That face offers nothing to mate; pick a flat or round face";
			return;
		}
		if (!GeometricConnectors.compatible(feature, kind)) {
			message = 'That face is a $feature; a $kind mate cannot use it';
			return;
		}
		var first = firstId;
		if (first == null) {
			firstId = sceneId;
			firstFace = faceIndex;
			message = 'Pick a face of another part for the $kind mate';
			return;
		}
		if (first == sceneId) {
			message = "Pick a face of another part";
			return;
		}
		try {
			mateId = session.addAssemblyFaceMate(kind, first, firstFace, sceneId, faceIndex);
			finished = true;
			var status = session.assemblyMateStatus();
			message = status == null ? "Mate added" : status;
		} catch (error:Dynamic) {
			message = "Could not add the mate: " + Std.string(error);
		}
	}
}
