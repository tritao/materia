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
    // X and Z have one ground bar under both supports; Y has a bracket under each.
    var fixedPad = axis == "Y" ? "YdriveFixed" : axis + "drivePad", floatingPad = axis == "Y" ? "YdriveFloating" : axis + "drivePad";
    var members = [axis + "fixed", axis + "floating", axis + "screw", axis + "nut", axis + "nutBlock"];
    return {
      id: axis.toLowerCase() + "-align-supports",
      label: axis + ": align the screw supports and pin them",
      kind: Assembly,
      start: Assembled,
      steps: [
        note("seat", "Fit the supports loose",
          "Bolt the motor mount to its pad and the BF12 to the floating pad with the tapped bolts only snug (no dowels). The pads are drilled and tapped, "
          + "not reamed: the supports can still move about 0.5 mm.", (fixedPad == floatingPad ? [fixedPad] : [fixedPad, floatingPad]).concat([axis + "fixed", axis + "floating"])),
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
          + "5 mm and drive the dowels. The pins keep the alignment, so the pad needs no precision machining.", (fixedPad == floatingPad ? [fixedPad] : [fixedPad, floatingPad]).concat([axis + "fixed", axis + "floating"]))
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

  /**
   * Commissioning: the guides' pitch and yaw over the travel, measured with an autocollimator or laser angular optics, go into the
   * controller's correction table, so the angular error times the offset to the tool (the Abbe error, see `HobberFrameChecks.scales`) is
   * compensated instead of left in the part. Roll about the travel is not measured here (it needs a level or the optics' roll option).
   */
  public static function compensateTilt(axis:String, travel:{lower:Float, upper:Float}, rails:Array<String>):Procedure {
    var scale = axis + "scale";
    return {
      id: axis.toLowerCase() + "-compensate-tilt",
      label: axis + ": measure pitch and yaw and load the correction table",
      kind: Service,
      start: Assembled,
      steps: [
        note("setup", "Set up the angular optics",
          "Put the autocollimator or laser angular reflector on the carriage (or the tool seat) and the optics on a stand beside the machine, aligned to the " + axis
          + " travel. Let the machine reach its working temperature first.", rails),
        {id: "low", title: "Carriage at the lower end", instruction: "Move " + axis + " to the lower end of its travel and zero the optics.",
          highlight: [select(scale)], action: SetJoints([{joint: axis, value: travel.lower}])},
        note("sweep", "Step through the travel",
          "Move in 20 mm steps to the upper end and back, stopping at each point to read pitch and yaw (the two angles about the axes across the travel) "
          + "from the scale reading. Repeat once in the other direction to see the hysteresis.", [scale]),
        {id: "high", title: "Carriage at the upper end", instruction: "Read the last point at the upper end of the travel.",
          highlight: [select(scale)], action: SetJoints([{joint: axis, value: travel.upper}])},
        note("table", "Load the correction table",
          "Enter pitch and yaw against scale position in the controller's tilt correction table for " + axis + ", with the Abbe offsets to the hob centre from the "
          + "machine's scale report. Re-run the sweep with the table on and keep the residual as the acceptance record.", [scale])
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
    var rails = ["X" => ["railXLeft", "railXRight"], "Y" => ["railYLeft", "railYRight"], "Z" => ["railZLeft", "railZRight"]];
    for (travel in machine.travel) if (rails.exists(travel.id)) result.push(compensateTilt(travel.id, {lower: travel.lower, upper: travel.upper}, rails.get(travel.id)));
    return result;
  }
}
