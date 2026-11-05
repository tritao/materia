import machinekit.assembly.JointFasteners;

/** The joint-derived fasteners of the hobber (BOM_PLAN F1): what the derivation finds and the screws by size. */
class HobberFastenerChecks {
  /** The derivation's findings, one line each, and then every joint. */
  public static function report(machine:FlexsplineHobber):Array<String> {
    var derivation = machine.derivedFasteners();
    if (derivation == null) throw "The hobber derived no fasteners";
    var lines:Array<String> = [];
    for (item in derivation.diagnostics.items) lines.push(item.code + " " + item.message);
    var sizes = new Map<String, Int>();
    for (line in derivation.lines()) sizes.set(line.item.description + " [" + line.item.partNumber + "]", line.quantity);
    for (name => count in sizes) lines.push('line $count x $name');
    for (joint in derivation.joints) lines.push('joint ${joint.id}: ${joint.count} x ${joint.size}x${joint.length}${joint.nutted ? " bolt" : ""}, grip ${joint.grip}, engagement ${joint.engagement}');
    for (joint in derivation.dowels) lines.push('dowel ${joint.id}: ${joint.count} x ISO 8734 ${joint.size}x${joint.length} ${joint.fit}, engagement ${joint.engagementA}/${joint.engagementB}');
    for (slip in derivation.slips) lines.push("slip " + machinekit.assembly.JointSlip.describe(slip));
    lines.push('totals: ${derivation.joints.length} joints, ${derivation.count()} screws, ${derivation.dowels.length} dowelled joints, ${derivation.dowelCount()} dowels');
    return lines;
  }
}
