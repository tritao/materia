package machinekit.component;

import cadkit.Shape;
import cadkit.modeling.Part;

/** Local, hash-pinned supplier geometry. Has no inferred mounting connectors or mechanical ratings.
 * Prefer an existing parametric recipe when one describes the required part.
 * Caller owns this import and must close it; each geometry() result is independently owned.
 */
class SupplierCadComponent extends MachineComponent {
  final source:Shape;
  var closed = false;
  public final sha256:String;

  public function new(
    path:String,
    expectedSha256:String,
    partNumber:String,
    description:String,
    material:String = "unspecified"
  ) {
    super(partNumber, description, material, true);
    sha256 = expectedSha256;
    source = imported(path, expectedSha256).cloneShape();
  }

  /**
   * Checked imports by file and pinned digest. A pinned asset never changes, and supplier parts are drawn many times
   * per process (each `geometry()` of a support or mount reads its asset), so each file is hashed and parsed once.
   * Kept shapes are copied out.
   */
  static var imports:Null<Map<String, Shape>>;
  static var importCount = 0;
  static inline final IMPORT_LIMIT = 32;

  static function imported(path:String, expectedSha256:String):Shape {
    var key = path + "|" + expectedSha256, memo:Map<String, Shape> = imports == null ? [] : imports;
    imports = memo;
    var known = memo.get(key);
    if (known != null) return known;
    var bytes = sys.io.File.getBytes(path), digest = haxe.crypto.Sha256.make(bytes), hex = new StringBuf();
    for (i in 0...digest.length) hex.add(StringTools.hex(digest.get(i), 2).toLowerCase());
    if (hex.toString() != expectedSha256) throw "Supplier STEP hash differs from pinned asset";
    var shape = Shape.importStep(path);
    if (shape.subshapeCount(CadKit.ShapeKind.Solid) == 0 || CadKit.shapeValidChecked(shape.borrowHandle()) == 0) {
      shape.close();
      throw "Supplier CAD requires valid solids";
    }
    if (importCount >= IMPORT_LIMIT) {
      for (kept in memo) kept.close();
      memo.clear();
      importCount = 0;
    }
    memo.set(key, shape);
    importCount++;
    return shape;
  }

  override public function hasGeometry():Bool return !closed;
  override public function geometryFidelity():GeometryFidelity return GeometryFidelity.Supplier;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    if (closed) throw "Supplier CAD component is closed";
    return new Part(source.cloneShape());
  }

  public function close():Void {
    if (!closed) {
      closed = true;
      source.close();
    }
  }
}
