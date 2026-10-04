import machinekit.welding.WeldingRecipe;

class WeldingRecipeTests {
  static var checks:Int = 0;
  static function check(value:Bool, message:String):Void {
    checks++;
    if (!value) throw message;
  }
  static function near(actual:Float, expected:Float, message:String):Void
    check(Math.abs(actual - expected) < 1e-8, '$message: expected $expected, got $actual');

  static function main():Void {
    var wire = {diameterMm: 1.2, depositionEfficiency: 0.95, maxSpeedMPerMin: 16.0};
    var ordinary = WeldingRecipe.passes(5, wire);
    check(ordinary.length == 1, "5 mm remains one pass");
    near(ordinary[0].weaveAmplitude, 0, "ordinary weld has no weave");
    near(ordinary[0].faceA + ordinary[0].faceB, 0, "ordinary tip remains on the CAD seam");
    var woven = WeldingRecipe.passes(7, wire);
    check(woven.length == 1, "7 mm remains one pass");
    near(woven[0].weaveAmplitude, 1.4, "weave follows leg size");
    near(woven[0].weaveFrequency, 0.1, "10 mm weave period");
    near(woven[0].faceA, 1.75, "woven tip opens toward first face");
    near(woven[0].faceB, 1.75, "woven tip opens toward second face");
    var passes = WeldingRecipe.passes(10, wire, 0.5);
    check(passes.length == 3, "10 mm has root, fill and cap");
    var area = 0.0;
    for (index in 0...passes.length) {
      var pass = passes[index];
      check(pass.name == ["root", "fill", "cap"][index], "ordered pass names");
      near(pass.recipe.legSize * pass.recipe.legSize / 2, [12.5, 18.75, 18.75][index], "pass area");
      var wireArea = Math.PI * wire.diameterMm * wire.diameterMm / 4;
      near(pass.recipe.travelSpeed * pass.recipe.legSize * pass.recipe.legSize / 2,
        pass.recipe.wireSpeed * 1000 / 60 * wireArea * wire.depositionEfficiency, "wire volume matches seam area");
      near(pass.interpassDwell, index == 0 ? 0 : 0.5, "cooling only between passes");
      if (index > 0) near(pass.faceA + pass.faceB, Math.sqrt(2 * area), "tip targets previously deposited surface");
      area += pass.recipe.legSize * pass.recipe.legSize / 2;
    }
    near(Math.sqrt(2 * area), 10, "final leg from total area");
    check(passes[1].faceA > passes[1].faceB && passes[2].faceB > passes[2].faceA, "fill and cap target alternate faces");
    var rejected = false;
    try WeldingRecipe.fillet(10, wire) catch (_:Dynamic) rejected = true;
    check(rejected, "oversized single-pass recipe is rejected");
    rejected = false;
    try WeldingRecipe.passes(13, wire) catch (_:Dynamic) rejected = true;
    check(rejected, "unsupported target leg is rejected");
    rejected = false;
    try WeldingRecipe.passes(7, {diameterMm: 1.2, depositionEfficiency: 0.95, maxSpeedMPerMin: 3.0}) catch (_:Dynamic) rejected = true;
    check(rejected, "CAD feeder limit is enforced for each pass");
    Sys.println('Welding recipe tests passed: $checks assertions');
  }
}
