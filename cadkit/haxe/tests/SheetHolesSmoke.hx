import cadkit.modeling.Vector;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.HoleSpec;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.sheetmetal.FlatPattern;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetFeature;
import materia.assembly.AssemblyFrames as F;

/** Holes cut in the developed blank: the folded solid as a solid cut would make it, the flat pattern with the holes, saved. */
class SheetHolesSmoke {
  static var checks = 0;

  static function check(value:Bool, label:String):Void {
    checks++;
    if (!value) throw "SheetHolesSmoke: " + label;
  }

  static function near(a:Float, b:Float, label:String):Void check(Math.abs(a - b) < 1e-6 * Math.max(1, Math.abs(b)), '$label: $a vs $b');

  /** A 100 x 60 x 2 plate (material z -2..0) with a 10 mm flange up along y = 0 (its wall at y -4..-2, z 2..12). */
  static function plate(b:DocumentBuilder):SheetFeature {
    var base = b.sheetBase(b.polygonSketch([new Vector(0, 0), new Vector(100, 0), new Vector(100, 60), new Vector(0, 60)]),
      SheetDemoParts.materialA(), b.length("thickness", 2));
    return b.sheetFlange(base, [{wall: 1, identity: "sketch:e0"}], 10, Math.PI / 2, 2);
  }

  static function holes():Array<HoleSpec> {
    // Down through the plate from its top face, and into the flange from its outer face (+Y into the material).
    var top = F.fromRotationMatrix(0, 0, 0, [1.0, 0, 0, 0, -1, 0, 0, 0, -1]);
    var outer = F.fromRotationMatrix(0, -4, 0, [1.0, 0, 0, 0, 0, 1, 0, -1, 0]);
    return [
      {frame: top, x: 30, y: -30, diameter: 6, depth: 2, through: true, kind: "clearance", name: "plate.h1"},
      {frame: outer, x: 50, y: -7, diameter: 4, depth: 3, kind: "clearance", name: "flange.h1"}
    ];
  }

  static function area(d:Float):Float return Math.PI * d * d / 4;

  public static function run():Int {
    checks = 0;
    var b = new DocumentBuilder();
    var sheet = plate(b), drilled = b.sheetHoles(sheet, holes());
    b.output(drilled);
    var document = b.finish();
    var folded = sheet.currentShape().volume(), flat = FlatPattern.of(sheet.sheet()).area();
    near(drilled.currentShape().volume(), folded - 2 * (area(6) + area(4)), "both holes cut through their walls");
    near(FlatPattern.of(drilled.sheet()).area(), flat - area(6) - area(4), "the flat pattern carries the holes");

    var text = DocumentCodec.encode(document);
    check(text.indexOf('"type":"sheet-holes"') >= 0 && text.indexOf("flange.h1") >= 0, "holes saved with their names");
    var restored = DocumentCodec.decode(text);
    check(DocumentCodec.encode(restored) == text, "save after restore is identical");
    restored.parameter("thickness").set(3);
    restored.recompute();
    check(restored.outputFeature().currentShape().volume() > 0, "holes follow a thickness edit");
    restored.close();
    document.close();

    var failures = 0;
    var blind:HoleSpec = {frame: F.fromRotationMatrix(0, 0, 0, [1.0, 0, 0, 0, -1, 0, 0, 0, -1]), x: 30, y: -30, diameter: 6, depth: 1};
    var misses:HoleSpec = {frame: F.identity(), x: 500, y: 500, diameter: 6, depth: 2, through: true};
    for (hole in [blind, misses]) {
      var bad = new DocumentBuilder();
      bad.output(bad.sheetHoles(plate(bad), [hole]));
      try bad.finish().close() catch (_:Dynamic) failures++;
    }
    check(failures == 2, "a blind hole and a hole off the sheet are errors");
    Sys.println('SheetHolesSmoke: $checks checks');
    return checks;
  }

  public static function main():Void run();
}
