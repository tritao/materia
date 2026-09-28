package toolpathkit.motion;

import toolpathkit.path.Provenance;

/** Maps MotionOp indices and path distance in metres to source locations. */
class ToolpathSourceMap {
  public final entries:Array<ToolpathSourceMapEntry> = [];

  public function new() {}

  public function add(opIndex:Int, startDistance:Float, endDistance:Float,
      provenance:Provenance):Void {
    var entry = new ToolpathSourceMapEntry(opIndex, startDistance,
      endDistance, provenance);
    entries.push(entry);
  }

  public function entriesFor(provenance:Provenance):Array<ToolpathSourceMapEntry> {
    return [for (entry in entries) if (entry.provenance == provenance) entry];
  }

  public function provenanceAt(opIndex:Int, distance:Float):Null<Provenance> {
    if (!Math.isFinite(distance) || distance < 0.0) return null;
    var last:Null<ToolpathSourceMapEntry> = null;
    for (entry in entries) if (entry.opIndex == opIndex) {
      if (entry.startDistance <= distance && distance < entry.endDistance)
        return entry.provenance;
      if (entry.startDistance == 0.0 && entry.endDistance == 0.0 &&
          distance == 0.0) return entry.provenance;
      last = entry;
    }
    return last != null && Math.abs(distance - last.endDistance) <= 1e-12
      ? last.provenance : null;
  }
}

class ToolpathSourceMapEntry {
  public final opIndex:Int;
  public final startDistance:Float;
  public final endDistance:Float;
  public final provenance:Provenance;

  public function new(opIndex:Int, startDistance:Float, endDistance:Float,
      provenance:Provenance) {
    this.opIndex = opIndex;
    this.startDistance = startDistance;
    this.endDistance = endDistance;
    this.provenance = provenance;
  }
}
