import CadKit;
import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.HoleSpec;
import cadkit.parametric.features.HolesFeature;
import cadkit.parametric.recording.DocumentBuilder;
import materia.assembly.AssemblyFrames as F;

/** Holes described by `HoleSpec` and cut by one `holes` feature: geometry, face names and the saved form. */
class HolesFeatureSmoke {
  static function check(value:Bool, message:String):Void {
    if (!value) throw "HolesFeatureSmoke: " + message;
  }

  static function near(a:Float, b:Float, what:String):Void check(Math.abs(a - b) < 1e-6 * Math.max(1, Math.abs(b)), '$what: $a vs $b');

  static function specs():Array<HoleSpec> {
    // The plate's top face, +Z into the material (pointing down), as a joint's face frame is.
    // Its +Y is world -Y, so holes at frame y 10..30 lie in the plate (world y -60..0).
    var top = F.fromRotationMatrix(0, 0, 20, [1.0, 0, 0, 0, -1, 0, 0, 0, -1]);
    return [
      {frame: top, x: 20, y: 10, diameter: 6.8, depth: 13, kind: "tapped", thread: "M8", name: "J1.h1.tap", nominal: 8},
      {frame: top, x: 50, y: 10, diameter: 9, depth: 20, through: true, kind: "clearance", thread: "M8", name: "J2.h1.clearance",
        recess: {diameter: 15, depth: 11.5, name: "J2.h1.counterbore"}},
      {frame: top, x: 80, y: 30, diameter: 6, depth: 20, kind: "dowel", fit: "H7", through: true}
    ];
  }

  /** 100 x 60 x 20 plate less: a 13 deep tap drill, a through clearance hole counterbored 11.5 from the far face, a through dowel. */
  static function expectedVolume():Float {
    var area = (d:Float) -> Math.PI * d * d / 4;
    return 100 * 60 * 20 - area(6.8) * 13 - (area(9) * (20 - 11.5) + area(15) * 11.5) - area(6) * 20;
  }

  public static function run():Void {
    var document = DocumentBuilder.build(function(b) {
      var plate = b.boxBetween(0, 100, -60, 0, 0, 20);
      b.output(b.holes(plate, specs()));
    });
    var shape = document.outputFeature().currentShape();
    near(shape.volume(), expectedVolume(), "plate with holes");
    var names = shape.elementNames(CadKit.ShapeKind.Face).join(" ");
    for (name in ["J1.h1.tap", "J2.h1.clearance", "J2.h1.counterbore", "cut1"])
      check(names.indexOf(name) >= 0, 'faces named under $name: $names');

    // The same tools as MachineComponent.machined cuts (it calls HolesFeature.tools): identical solid.
    var raw = Part.box(100, 60, 20), body = raw.translated(new cadkit.modeling.Vector(50, -30, 0));
    raw.close();
    var tools = HolesFeature.tools(specs());
    var direct = body.subtractAll(tools);
    near(direct.volume(), shape.volume(), "same volume as the direct cut");
    var directNames = direct.shape.elementNames(CadKit.ShapeKind.Face).join(" ");
    // The document prefixes each name with the feature that made it (f1:, f2:); the rest is the same name.
    var unprefixed = [for (name in names.split(" ")) ~/^f[0-9]+:/.replace(name, "")].join(" ");
    check(unprefixed == directNames, 'same face names as the direct cut: $unprefixed vs $directNames');
    direct.close();
    body.close();
    for (tool in tools) tool.close();

    var text = DocumentCodec.encode(document);
    check(text.indexOf('"type":"holes"') >= 0 && text.indexOf('"J2.h1.counterbore"') >= 0, "holes saved with their names");
    document.close();
    var restored = DocumentCodec.decode(text);
    check(DocumentCodec.encode(restored) == text, "save after restore is identical");
    near(restored.outputFeature().currentShape().volume(), expectedVolume(), "restored plate");
    restored.close();
    Sys.println("HolesFeatureSmoke: hole specs cut in one step, named, saved and identical to the direct cut");
  }

  public static function main():Void run();
}
