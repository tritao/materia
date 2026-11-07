import app.editor.SelectionReference;
import app.editor.SelectionReference.PickedFace;

/** The status bar's selection text and the text clicking it copies. */
class SelectionReferenceTests {
  static var checks = 0;

  static function check(ok:Bool, message:String):Void {
    checks++;
    if (!ok) throw message;
  }

  static function near(a:Float, b:Float, message:String):Void check(Math.abs(a - b) < 1e-9, message + ": " + a + " vs " + b);

  public static function main():Int {
    var reference = "Scene: Hobber\nPath: X axis / Xbridge\nKey: project:Xbridge";
    // No picked face: the status line and the copy text are the object's alone.
    check(SelectionReference.statusText("Xbridge selected", null) == "Xbridge selected", "no face leaves the label");
    check(SelectionReference.copyText(reference, null) == reference, "no face copies the reference as is");
    // A picked scene-part face.
    var face = new PickedFace(12, [1.5, -2.0, 260.0]);
    check(SelectionReference.statusText("Xbridge selected", face) == "Xbridge selected · face 12", "status names the face");
    var text = SelectionReference.copyText(reference, face);
    check(text == reference + "\nFace: 12\nPoint: 1.5 -2 260 (part frame, mm)", "object, face and point lines: " + text);
    // A CAD-document face also carries its persistent name, on one line.
    var named = SelectionReference.copyText(reference, new PickedFace(3, null, "Face:a\nb"));
    check(named == reference + "\nFace: 3\nFace name: Face:a\\nb", "name line, no point line: " + named);
    check(SelectionReference.copyText(reference, new PickedFace(0)).split("\n").length == 4, "face 0 is a face");
    check(SelectionReference.copyText(reference, new PickedFace(5, [1.0, 2.0])).indexOf("Point") < 0, "a malformed point is left out");
    // Number formatting.
    check(new PickedFace(1, [1.0, 2.0, 3.0]).same(new PickedFace(1, [1.0, 2.0, 3.0])) && !new PickedFace(1).same(new PickedFace(2)) && !new PickedFace(1).same(null), "faces compare by value");
    check(SelectionReference.number(0.0004) == "0", "rounds to a millimetre thousandth");
    check(SelectionReference.number(-0.0001) == "0", "no negative zero");
    check(SelectionReference.number(12.3456) == "12.346", "three decimals");
    check(SelectionReference.number(260.0) == "260", "whole numbers stay whole");
    // World metres to the part frame in mm: a node turned 90 degrees about z and moved to (1, 2, 3) m.
    var turned = [0.0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1];
    var local = SelectionReference.localPointMm(turned, 1.0, 2.5, 3.25);
    check(local != null, "a rigid matrix inverts");
    near(local[0], 500.0, "x: the part's x axis points along world +y");
    near(local[1], 0.0, "y");
    near(local[2], 250.0, "z");
    var moved = SelectionReference.localPointMm([1.0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0.5, 0, 0, 1], 0.5, 0.001, 0.0);
    near(moved[1], 1.0, "a translated node: 1 mm in y");
    check(SelectionReference.localPointMm([1.0, 0], 0, 0, 0) == null, "a short matrix gives no point");
    Sys.println("selection reference tests: " + checks + " checks passed");
    return 0;
  }
}
