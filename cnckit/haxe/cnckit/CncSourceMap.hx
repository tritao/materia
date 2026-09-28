package cnckit;

import toolpathkit.path.Provenance;

/** Maps MotionOp indices and path distance in metres to source locations. */
class CncSourceMap {
  public final entries:Array<CncSourceMapEntry> = [];

  public function new() {}

  public function add(opIndex:Int, startDistance:Float, endDistance:Float,
      span:Provenance):Void
    entries.push(new CncSourceMapEntry(opIndex, startDistance, endDistance, span));

  public function spanAt(opIndex:Int, distance:Float):Null<Provenance> {
    if (!Math.isFinite(distance) || distance < 0.0) return null;
    var last:Null<CncSourceMapEntry> = null;
    for (entry in entries) if (entry.opIndex == opIndex) {
      if (entry.startDistance <= distance && distance < entry.endDistance)
        return entry.span;
      if (entry.startDistance == 0.0 && entry.endDistance == 0.0 &&
          distance == 0.0) return entry.span;
      last = entry;
    }
    return last != null && Math.abs(distance - last.endDistance) <= 1e-12
      ? last.span : null;
  }
}

class CncSourceMapEntry {
  public final opIndex:Int;
  public final startDistance:Float;
  public final endDistance:Float;
  public final span:Provenance;

  public function new(opIndex:Int, startDistance:Float, endDistance:Float,
      span:Provenance) {
    this.opIndex = opIndex;
    this.startDistance = startDistance;
    this.endDistance = endDistance;
    this.span = span;
  }
}
