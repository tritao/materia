package toolpathkit.path;

/** Origin of a path operation. G-code locations are one-based. */
class Provenance {
  public final kind:ProvenanceKind;
  public final line:Int;
  public final column:Int;
  public final length:Int;
  public final operationIndex:Null<Int>;

  public function new(line:Int, column:Int, length:Int,
      kind:ProvenanceKind = GCode, operationIndex:Null<Int> = null) {
    this.kind = kind;
    this.line = line;
    this.column = column;
    this.length = length;
    this.operationIndex = operationIndex;
  }

  public static function cam(operationIndex:Int):Provenance
    return new Provenance(operationIndex, 1, 0, Cam, operationIndex);
}

enum ProvenanceKind {
  GCode;
  Cam;
}
