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

    public function new(path:String, expectedSha256:String, partNumber:String, description:String, material:String = "unspecified") {
        var bytes = sys.io.File.getBytes(path), digest = haxe.crypto.Sha256.make(bytes), hex = new StringBuf();
        for (i in 0...digest.length) hex.add(StringTools.hex(digest.get(i),2).toLowerCase());
        if (hex.toString() != expectedSha256) throw "Supplier STEP hash differs from pinned asset";
        super(partNumber,description,material,true);
        sha256 = expectedSha256;
        source = Shape.importStep(path);
        if (source.subshapeCount(CadKit.ShapeKind.Solid) == 0 || CadKit.shapeValidChecked(source.borrowHandle()) == 0) {
            source.close();
            throw "Supplier CAD requires valid solids";
        }
    }

    override public function hasGeometry():Bool return !closed;
    override public function geometry(detail:ComponentDetail = Preview):Part {
        if (closed) throw "Supplier CAD component is closed";
        return new Part(source.cloneShape());
    }

    public function close():Void {if (!closed) {closed=true;source.close();}}
}
