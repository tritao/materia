package machinekit.pendant;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.presentation.AssemblyPresentation;
import materia.presentation.PresentationCodec;
import materia.presentation.Presentations;

/**
 * The pendant's views: assembled, and exploded along the panel's z axis in the order it comes apart. The case screws lift
 * out, then the plate with every control on it, then the board screws, the board, and last the inserts out of their
 * posts and standoffs. Offsets are presentation only; the assembly itself is untouched.
 */
class PendantPresentation {
  public static function build(definition:AssemblyDefinition):AssemblyPresentation {
    var result = Presentations.assembled(definition, -1.1, 0.35, "pendant");
    var ids = [for (part in AssemblyDefinitionFlattener.flattenView(definition).occurrences) part.id];
    var all:Array<PartSelector> = [for (id in ids) {
      occurrence:id, subtree:false
    }
    ];
    result.views.push({
      id: "exploded",
      label: "Exploded",
      camera: Frame(all, -1.1, 0.35, 1.15),
      visibility: [],
      sections: [],
      explode: 1.0,
      source: Generated("pendant")
    });
    for (id in ids) {
      var offset = lift(id);
      if (offset == null) continue;
      result.explode.push({
        select: {occurrence: id, subtree: false},
        direction: {x: 0, y: 0, z: 1},
        distance: offset.distance,
        stage: offset.stage,
        source: Generated("pendant")
      });
    }
    PresentationCodec.validate(definition, result);
    return result;
  }

  static function lift(id:String):Null<{distance:Float, stage:Int}> {
    if (StringTools.startsWith(id, "fasteners/screw-")) return {distance: 70.0, stage: 1};
    if (StringTools.startsWith(id, "fasteners/board-screw-")) return {distance: 60.0, stage: 3};
    if (StringTools.startsWith(id, "fasteners/board-insert-")) return {distance: 10.0, stage: 5};
    if (StringTools.startsWith(id, "fasteners/insert-")) return {distance: 14.0, stage: 5};
    if (id == "case/board") return {distance: 30.0, stage: 4};
    if (id == "case/shell") return null;
    // The plate and everything mounted on it (the controls, in their groups) leave together.
    return {distance: 40.0, stage: 2};
  }
}
