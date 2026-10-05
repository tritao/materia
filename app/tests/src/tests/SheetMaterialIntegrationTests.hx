package tests;

import app.EditorScene;
import app.SceneCodec;
import app.editor.SheetMetalAuthoring;
import cadkit.sheetmetal.BendDimensions;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.SheetDiagnosis;
import haxeon.wire.JsonWire;
import materia.project.MaterialDef;
import materia.project.MaterialLibrary;

/** Library-to-stock ownership and current scene persistence retain typed sheet-grade data. */
class SheetMaterialIntegrationTests {
  static function check(value:Bool, label:String):Void { if (!value) throw label; }
  static function near(actual:Float, expected:Float, label:String):Void {
    check(Math.isFinite(actual) && Math.abs(actual - expected) < 1e-9, label);
  }
  static function diagnosis(code:String, action:Void->Void):Void {
    try { action(); }
    catch (failure:SheetDiagnosis) { if (failure.code == code) return; throw failure; }
    throw "Expected " + code;
  }
  public static function run():Void {
    var definition = MaterialLibrary.require("mild-steel");
    var stock = SheetMaterial.fromDefinition(definition, 1.5, Math.PI / 3);
    diagnosis("sheet.no-grain-direction", function() { var invalid = SheetMaterial.fromDefinition(definition, 1.5); });
    diagnosis("sheet.not-sheet-material", function() { var invalid = SheetMaterial.fromDefinition(MaterialLibrary.require("painted"), 1.5); });
    near(stock.defaultRadius(), 1.5, "library radius scales with stock thickness");
    near(BendDimensions.allowance(stock, 1.5, Math.PI / 2), Math.PI / 2 * 2.07, "library K table drives development");
    var mechanical = stock.mechanical; if (mechanical == null) throw "missing stock properties";
    near(mechanical.uts, 400, "library strength copied to stock");
    var defaults = definition.sheet; if (defaults == null) throw "missing sheet defaults";
    defaults.kFactorTable[1].k = 0;
    var libraryMechanical = definition.mechanical; if (libraryMechanical == null) throw "missing library properties";
    definition.mechanical = {uts: 500, yield: libraryMechanical.yield, youngs: libraryMechanical.youngs};
    near(mechanical.uts, 400, "stock isolates mutable library properties");
    near(BendDimensions.allowance(stock, 1.5, Math.PI / 2), Math.PI / 2 * 2.07, "stock isolates mutable library K rows");
    var form = new SheetMetalAuthoring();
    form.materialId = "aluminium-5052-h32"; form.thickness = "1"; form.grainDegrees = "30";
    var aluminium = form.stock();
    var aluminiumMechanical = aluminium.mechanical; if (aluminiumMechanical == null) throw "missing aluminium properties";
    near(aluminiumMechanical.youngs, 70000, "editor grade supplies mechanical properties");
    var grain = aluminium.grainAngle; if (grain == null) throw "missing authored grain";
    near(grain, Math.PI / 6, "editor rolling direction uses root-frame radians");
    near(BendDimensions.allowance(aluminium, 1, 1), 1.4, "blank K field uses selected grade");
    form.kFactor = "0.35";
    near(BendDimensions.allowance(form.stock(), 1, 1), 1.35, "explicit K override keeps material properties");
    var custom:MaterialDef = JsonWire.decode(JsonWire.encode(MaterialLibrary.require("mild-steel")));
    custom.id = "custom-sheet-lot"; custom.name = "Custom sheet lot";
    var scene = new EditorScene([]);
    try {
      var encoded = SceneCodec.encode(scene, null, null, null, null, null, [custom]);
      var restored = SceneCodec.decodeCustomMaterialsRoot(SceneCodec.parse(encoded));
      check(restored.length == 1, "scene preserves custom material identity");
      form.materialId = custom.id; form.kFactor = ""; form.thickness = "1.5";
      var restoredStock = form.stock(restored);
      var restoredMechanical = restoredStock.mechanical; if (restoredMechanical == null) throw "missing persisted properties";
      near(restoredMechanical.youngs, 205000, "scene preserves mechanical data");
      near(restoredStock.defaultRadius(), 1.5, "scene preserves sheet defaults");
      near(BendDimensions.allowance(restoredStock, 1.5, 1), 2.07, "scene preserves K rows");
      custom.sheet = null; form.kFactor = "0.35";
      var explicitStock = form.stock([custom]);
      var explicitMechanical = explicitStock.mechanical; if (explicitMechanical == null) throw "missing explicit stock properties";
      near(explicitMechanical.uts, 400, "explicit K stock retains known library strengths without sheet defaults");
      var rejected = false;
      try { var old = SceneCodec.parse('{"format":"materia.scene","version":1,"objects":[]}'); }
      catch (_:Dynamic) rejected = true;
      check(rejected, "old scene format is rejected");
    } catch (error:Dynamic) { scene.dispose(); throw error; }
    scene.dispose();
  }
  static function main():Void { run(); Sys.println("Sheet material integration tests passed"); }
}
