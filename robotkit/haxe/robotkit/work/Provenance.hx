package robotkit.work;

/** Traces a WorkSurface back to the design element (and derivation kind) it came from. */
class Provenance {
  public final designElementId:String;
  public final sourceKind:SourceKind;

  public function new(designElementId:String, sourceKind:SourceKind) {
    if (designElementId == null) throw "Provenance requires a design element id";
    if (sourceKind == null) throw "Provenance requires a source kind";
    this.designElementId = designElementId;
    this.sourceKind = sourceKind;
  }
}
