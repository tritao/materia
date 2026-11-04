import materia.assembly.AssemblyDefinition.AssemblyJointCoordinate;
import materia.presentation.AssemblyPresentation;

/**
 * Assembly procedures for the hobber's axes (FRAME_PLAN.md H7): how the screw supports are aligned to the nut and pinned, and how
 * rails are set against an edge. Steps are notes and joint positions; they highlight the members the step works on. The
 * seat holes the pinning needs are `HobberDrives.seatHoles`; the nut block's bolts are `HobberDrives.nutBlockHoles`.
 */
class HobberAssemblyProcedures {
  static function select(id:String):PartSelector return {occurrence: id, subtree: false};

  static function note(id:String, title:String, instruction:String, members:Array<String>):ProcedureStep return {
    id: id,
    title: title,
    instruction: instruction,
    highlight: [for (member in members) select(member)],
    action: Note
  };

  /** Fit the supports loose, run the carriage to both ends so the screw centres on the nut, tighten, ream and pin. */
  public static function alignScrewSupports(axis:String, travel:{lower:Float, upper:Float}):Procedure {
    var fixedPad = axis + "driveFixed", floatingPad = axis + "driveFloating";
    var members = [axis + "fixed", axis + "floating", axis + "screw", axis + "nut", axis + "nutBlock"];
    return {
      id: axis.toLowerCase() + "-align-supports",
      label: axis + ": align the screw supports and pin them",
      kind: Assembly,
      start: Assembled,
      steps: [
        note("seat", "Fit the supports loose",
          "Bolt the motor mount to its pad and the BF12 to the floating pad with the tapped bolts only snug (no dowels). The pads are drilled and tapped, "
          + "not reamed: the supports can still move about 0.5 mm.", [fixedPad, floatingPad, axis + "fixed", axis + "floating"]),
        note("nut", "Bolt the nut block to the carriage",
          "Fit the nut, its block and the block's two carriage bolts, then thread the screw through the nut into the mount and the BF12.",
          [axis + "nut", axis + "nutBlock", axis + "screw"]),
        {id: "low", title: "Run the carriage to the lower end", instruction: "Turn the screw by hand until the carriage rests at the lower end of its travel; the nut must turn freely.",
          highlight: [select(axis + "nut")], action: SetJoints([{joint: axis, value: travel.lower}])},
        note("centre-low", "Centre the screw on the nut",
          "With the nut at this end, move the floating support (the end the screw sits loose in) until the screw spins without binding at the nut; snug its bolts.", [axis + "floating"]),
        {id: "high", title: "Run the carriage to the upper end", instruction: "Turn the screw by hand to the other end; the nut must still turn freely.",
          highlight: [select(axis + "nut")], action: SetJoints([{joint: axis, value: travel.upper}])},
        note("centre-high", "Check the screw is centred at both ends",
          "Repeat the free-turning check at both ends. If it binds, loosen the supports and shift them by half the difference.", members),
        note("tighten", "Tighten, then ream and pin",
          "Tighten all support bolts. Ream the two dowel holes in each support foot through the pad at "
          + "5 mm and drive the dowels. The pins keep the alignment, so the pad needs no precision machining.", [axis + "fixed", axis + "floating", fixedPad, floatingPad])
      ],
      source: Generated("hobber")
    };
  }

  /** Rails against an edge: the datum edge of the seat sets the rail's line, the push screws hold it while the bolts are tightened. */
  public static function setRails(axis:String, rails:Array<String>):Procedure {
    return {
      id: axis.toLowerCase() + "-set-rails",
      label: axis + ": set the rails against their edge",
      kind: Assembly,
      start: Assembled,
      steps: [
        note("clean", "Clean the seat and the rail", "Stone the seat face and the rail underside, and wipe the datum edge.", rails),
        note("edge", "Lay the rails against the edge",
          "Seat each rail against its datum edge and bolt it down finger-tight; push screws or side bars hold it against the edge along its whole length.", rails),
        note("bolts", "Tighten from the datum end",
          "Tighten the rail bolts from the datum end along the rail, keeping the edge contact. Check straightness with an indicator against the rail side.", rails),
        note("second", "Set the second rail parallel",
          "Tighten the second rail the same way, then check both rails' parallelism and the carriage's free running.", rails)
      ],
      source: Generated("hobber")
    };
  }

  public static function build(machine:FlexsplineHobber):Array<Procedure> {
    var result:Array<Procedure> = [];
    for (travel in machine.travel) result.push(alignScrewSupports(travel.id, {lower: travel.lower, upper: travel.upper}));
    result.push(setRails("X", ["railXLeft", "railXRight"]));
    result.push(setRails("Z", ["railZLeft", "railZRight"]));
    result.push(setRails("Y", ["railYLeft", "railYRight"]));
    return result;
  }
}
