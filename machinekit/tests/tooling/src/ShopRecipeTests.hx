import materia.tooling.ToolingCodec;
import materia.tooling.panel.PanelJoints;

/** Regression against the delivered mill shop records before their extraction. */
class ShopRecipeTests {
 static var assertions = 0;
 static function same(actual:Dynamic, expected:Dynamic, label:String):Void {
  assertions++;
  if (ToolingCodec.canonical(actual) != ToolingCodec.canonical(expected)) {
   for (key in Reflect.fields(expected))
    if (ToolingCodec.canonical(Reflect.field(actual,key)) != ToolingCodec.canonical(Reflect.field(expected,key)))
     throw "Shop extraction changed " + label + "." + key + ": " + ToolingCodec.canonical(Reflect.field(actual,key))
      + " expected " + ToolingCodec.canonical(Reflect.field(expected,key));
   throw "Shop extraction changed " + label + ": " + ToolingCodec.canonical(actual) + " expected " + ToolingCodec.canonical(expected);
  }
 }
 static function main():Void {
  if (Sys.args().length != 1) throw "Supply machinekit/tests/tooling/shop-recipe-reference.json";
  var reference:Dynamic = haxe.Json.parse(sys.io.File.getContent(Sys.args()[0]));
  var base = CastingShopRecipe.tooling("mill-base-panel-eps-review");
  for (name in ["base", "column"]) {
   var expected:Dynamic = Reflect.field(reference, name + "Recipe");
   var recipe = CastingShopRecipe.tooling(expected.id);
   // These allocation/axis choices belong to the casting, rather than the shop.
   var rules:Array<Dynamic> = expected.featureRules;
   for (rule in recipe.featureRules) for (source in rules) if (rule.kind == source.kind && rule.opening == source.opening) {
    rule.priority = source.priority;
    rule.preferredAxis = source.preferredAxis;
   }
   recipe.coreWireAxis = expected.coreWireAxis;
   same(ToolingCodec.recipeRecord(ToolingCodec.copyRecipe(recipe)), expected, name + " recipe");
   if (name == "base") base = recipe;
  }
  same(CastingShopRecipe.fabrication(), reference.fabrication, "fabrication inputs");
  same(CastingShopRecipe.compaction(), reference.compaction, "compaction");
  same(CastingShopRecipe.process(base,"mill-base-process-review"), reference.process, "process and compound");
  var aggregate = CastingShopRecipe.process(base,"aggregate-design-target").castability;
  var dg:Float = cast aggregate.maximumAggregateSizeMm;
  same(dg,5,"declared aggregate design target");
  same(dg * aggregate.minimumWallAggregateRatio,25,"wall and rib minimum");
  same(dg * (cast aggregate.minimumPassageAggregateRatio:Float),15,"flow passage minimum");
  same(dg + aggregate.steelAggregateMarginMm,10,"steel cover minimum");
  same(Math.max(dg + aggregate.steelAggregateMarginMm,dg * aggregate.steelSpacingAggregateRatio),10,
   "separate steel spacing minimum");
  var expectedCam:Dynamic = reference.cam;
  var diameters:Array<Float> = reference.panelToolDiametersMm;
  var panels = [for (index in 0...diameters.length) {
   var panel = PanelJoints.blank("shop-test-" + index,100,100,base.panel);
   panel.operations[0].toolDiameter = diameters[index];
   panel;
  }];
  var settings = CastingShopRecipe.cam(panels);
  for (key in Reflect.fields(expectedCam)) {
   if (key == "lengthUnit" || key == "speedUnit" || key == "studyOnly" || key == "tools" || key == "travel") continue;
   same(Reflect.field(settings,key), Reflect.field(expectedCam,key), "CAM " + key);
  }
  same([for (tool in settings.tools) {number:tool.number,diameter:tool.diameter,nominalDiameter:tool.nominalDiameter,
   fluteLength:tool.fluteLength,stickOut:tool.stickOut,length:tool.length}], expectedCam.tools, "CAM tools");
  same({lower:settings.travel.lower,upper:settings.travel.upper}, expectedCam.travel, "CAM travel");
  var other = CastingShopRecipe.tooling("independent-casting");
  base.panel.materialSpec = "changed only in this test";
  same(other.panel.materialSpec,"coated particleboard","independent recipe stock");
  Sys.println("Shared shop recipe: " + assertions + " assertions passed");
 }
}
