import CadKit;
import cadkit.ElementNames;
import machinekit.component.MachineKitComponents;

/**
	Every MachineKit component's default geometry names its faces strongly and uniquely: a face with a weak or repeated
	name can only be found again by its shape, so a mate picked on it does not survive edits. Components listed in
	`UNNAMED` are known not to yet. A supplier's model is its STEP as shipped, with no feature history to name faces by.
*/
class MachineKitNamingAudit {
  /**
		Components with faces that only their geometry can tell apart: shrink this list, never grow it.
		- shaft-coupling: each set-screw hole's planar end is split in two halves with the same neighbours.
		(Split faces of revolution, such as a barrel's side halved by a base, are named by their angular sector.)
		Such pieces get weak ordinals by position (`~0`, `~1`), which resolve only when the geometry confirms them.
	*/
  static var UNNAMED:Array<String> = ["machinekit.motion.shaft-coupling"];

  /**
   * Recipes with no default part: each is built from the identity of a source part, so `create()` without it is
   * rejected. Their faces are named where the sources build them.
   */
  static var NEEDS_SOURCE:Array<String> = [
    "machinekit.structural.cast-in-steel-insert",
    "machinekit.structural.linear-mount-nut-cassette",
    "machinekit.tooling.source-casting"
  ];

  public static function run():Void {
    var failures:Array<String> = [];
    var report = Sys.getEnv("MACHINEKIT_NAMING_AUDIT") == "print";
    for (type in MachineKitComponents.defaultRegistry().all()) {
      if (NEEDS_SOURCE.indexOf(type.id) >= 0) continue;
      var bad = weakFaces(type.id);
      if (bad == null) continue;
      if (report && bad.length
        > 0) Sys.println('naming audit ${type.id}: ${bad.length} weak or repeated: ${bad.slice(0, 6).join(", ")}');
      var listed = UNNAMED.indexOf(type.id) >= 0;
      if (bad.length > 0 && !listed) failures.push('${type.id}: ${bad.slice(0, 6).join(", ")}');
      if (bad.length == 0 && listed) failures.push('${type.id} names every face now: remove it from UNNAMED');
    }
    if (!report && failures.length > 0) throw "MachineKitNamingAudit:\n  " + failures.join("\n  ");
  }

  /** The weak or repeated face names of `id`'s default geometry, or null when it has none. */
  static function weakFaces(id:String):Null<Array<String>> {
    var component = MachineKitComponents.defaultRegistry().byId(id).create();
    if (!component.hasGeometry() || component.geometryFidelity() == machinekit.component.GeometryFidelity.Supplier) return null;
    var part = component.geometry();
    var names = part.shape.elementNames(CadKit.ShapeKind.Face);
    part.close();
    var seen:Map<String, Bool> = [];
    var bad:Array<String> = [];
    for (name in names) {
      if (!ElementNames.isStrong(name) || seen.exists(name)) bad.push(name);
      seen.set(name, true);
    }
    return bad;
  }
}
