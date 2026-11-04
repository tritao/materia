import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.BoltedFace;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** A box between two corners that states its bolted faces, optionally with a vertical bore; for tests of covers and removals. */
class TestSlab extends MachineComponent {
  final min:Array<Float>;
  final max:Array<Float>;
  final faces:Array<BoltedFace>;
  final bore:Null<{x:Float, y:Float, diameter:Float}>;

  public function new(name:String, min:Array<Float>, max:Array<Float>, ?faces:Array<BoltedFace>, ?bore:{x:Float, y:Float, diameter:Float}) {
    super("TEST-SLAB-" + name, "Test slab " + name, "steel", true);
    this.min = min;
    this.max = max;
    this.faces = faces == null ? [] : faces;
    this.bore = bore;
  }

  override public function fasteningFaces():Array<BoltedFace> return faces;
  override public function machinable():Bool return true;
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var box = Part.box(max[0] - min[0], max[1] - min[1], max[2] - min[2]), moved = box.translated(new Vector((min[0] + max[0]) / 2, (min[1] + max[1]) / 2, min[2]));
    box.close();
    var body = Solids.named(moved, "body");
    if (bore != null) body = Solids.cut(body, [Part.cylinderSpan(bore.diameter / 2, min[2] - 1, max[2] + 1, bore.x, bore.y)]);
    return machined(body);
  }
}
