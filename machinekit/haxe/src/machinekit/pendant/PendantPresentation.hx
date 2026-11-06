package machinekit.pendant;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.presentation.AssemblyPresentation;
import materia.presentation.PresentationCodec;
import materia.presentation.Presentations;

/**
 * The pendant's views: assembled, and exploded along the panel's z axis in the order it comes apart. The case screws lift
 * out, then the plate with every control, the standoffs and the display's frame on it, then the board screws drop out and the
 * board lifts from the shell, and last the case inserts leave their posts. Offsets are presentation only; the assembly itself is untouched.
 */
class PendantPresentation {
  public static function build(definition:AssemblyDefinition, prefix:String = ""):AssemblyPresentation {
    var result = Presentations.assembled(definition, -1.1, 0.35, "pendant");
    // Only the pendant comes apart; a machine it is attached to stays as it is.
    var ids = [for (part in AssemblyDefinitionFlattener.flattenView(definition).occurrences) if (StringTools.startsWith(part.id, prefix)) part.id];
    var all:Array<PartSelector> = [for (id in ids) {
      occurrence:id, subtree:false
    }
    ];
    result.views.push(new PresentationView(
      "exploded",
      "Exploded",
      Frame(all, -1.1, 0.35, 1.15),
      [],
      [],
      1.0,
      Generated("pendant")
    ));
    for (id in ids) {
      var offset = lift(id.substr(prefix.length));
      if (offset == null) continue;
      result.explode.push({
        select: {occurrence: id, subtree: false},
        direction: {x: 0, y: 0, z: offset.up ? 1 : -1},
        distance : offset.distance,
        stage : offset.stage,
        source : Generated("pendant")
      });
    }
    PresentationCodec.validate(definition, result);
    return result;
  }

  static function lift(id:String):Null<{distance:Float, stage:Int, up:Bool}> {
    if (StringTools.startsWith(id, "fasteners/screw-")) return {distance: 70.0, stage: 1, up: true};
    // The board hangs from the plate, so once the plate is off its screws drop out of the standoffs and it lifts out of the shell.
    if (StringTools.startsWith(id, "fasteners/board-screw-")) return {distance: 15.0, stage: 3, up: false};
    if (id == "case/board") return {distance: 30.0, stage: 4, up: true};
    if (StringTools.startsWith(id, "fasteners/insert-")) return {distance: 14.0, stage: 5, up: true};
    if (id == "case/shell") return null;
    // The plate and everything on it (the controls, the bosses' inserts and the standoffs, the display's frame) leave together.
    return {distance: 40.0, stage: 2, up: true};
  }
}
