package toolpathkit.motion;

import toolpathkit.path.Provenance;

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
