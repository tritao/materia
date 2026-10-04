import CadKit;
import cadkit.ElementNames;
import machinekit.component.MachineKitComponents;

/**
	Every MachineKit component's default geometry names its faces strongly and uniquely
	(cadkit/plans/TOPOLOGICAL_NAMING.md, TN8): a face with a weak or repeated name can only be found again by its shape,
	so a mate picked on it does not survive edits. Components listed in `UNNAMED` are known not to yet.
*/
class MachineKitNamingAudit {
	/**
		Components with faces that only their geometry can tell apart: shrink this list, never grow it.
		- pillow-block: the base splits the barrel's side into two mirror-image halves with the same neighbours;
		- shaft-coupling: each set-screw hole's end is split in two the same way.
		Such pieces get weak ordinals by position (`~0`, `~1`), which resolve only when the geometry confirms them.
	*/
	static var UNNAMED:Array<String> = ["machinekit.motion.pillow-block", "machinekit.motion.shaft-coupling"];

	public static function run():Void {
		var failures:Array<String> = [];
		var report = Sys.getEnv("MACHINEKIT_NAMING_AUDIT") == "print";
		for (type in MachineKitComponents.defaultRegistry().all()) {
			var bad = weakFaces(type.id);
			if (bad == null)
				continue;
			if (report && bad.length > 0)
				Sys.println('naming audit ${type.id}: ${bad.length} weak or repeated: ${bad.slice(0, 6).join(", ")}');
			var listed = UNNAMED.indexOf(type.id) >= 0;
			if (bad.length > 0 && !listed)
				failures.push('${type.id}: ${bad.slice(0, 6).join(", ")}');
			if (bad.length == 0 && listed)
				failures.push('${type.id} names every face now: remove it from UNNAMED');
		}
		if (!report && failures.length > 0)
			throw "MachineKitNamingAudit:\n  " + failures.join("\n  ");
	}

	/** The weak or repeated face names of `id`'s default geometry, or null when it has none. */
	static function weakFaces(id:String):Null<Array<String>> {
		var component = MachineKitComponents.defaultRegistry().byId(id).create();
		if (!component.hasGeometry())
			return null;
		var part = component.geometry();
		var names = part.shape.elementNames(CadKit.ShapeKind.Face);
		part.close();
		var seen:Map<String, Bool> = [];
		var bad:Array<String> = [];
		for (name in names) {
			if (!ElementNames.isStrong(name) || seen.exists(name))
				bad.push(name);
			seen.set(name, true);
		}
		return bad;
	}
}
