import machinekit.enclosure.LaminatedPanel;
import machinekit.enclosure.PanelCatalogue;
import materia.project.MaterialLibrary;

class PanelCatalogueChecks {
  static var checks = 0;
  static function check(value:Bool, message:String):Void {
    checks++;
    if (!value) throw message;
  }
  static function rejected(build:Void->Void, fragment:String):Void {
    var message = "";
    try build() catch (error:Dynamic) message = Std.string(error);
    check(message.indexOf(fragment) >= 0, 'Expected rejection containing "$fragment", got "$message"');
  }
  public static function main():Void {
    EnclosureRuleChecks.run();
    var seen = new Map<String,Bool>();
    for (panel in PanelCatalogue.all()) {
      panel.validate();
      check(!seen.exists(panel.id), "Duplicate construction");
      seen.set(panel.id, true);
      check(panel.thicknessMm() > 0 && panel.arealMassKgM2() > 0, "Invalid planning dimensions/mass");
      check(panel.qualifications.impactRecord == null && panel.qualifications.coolantRecord == null
        && panel.qualifications.heatLimitC == null && panel.qualifications.fireClass == null,
        "Unqualified catalogue entry gained a protective rating");
      var bom = panel.bomProperties();
      check(bom.constructionId == panel.id && bom.layers.length == panel.layers.length,
        "BOM lost construction layers");
    }
    var composite = PanelCatalogue.require("stainless-faced-acm");
    check(Math.abs(composite.thicknessMm() - 4.7) < 1e-9, "Composite thickness omitted adhesive/facing");
    check(Math.abs(composite.arealMassKgM2() - (7.2 + 0.24 + 3.95)) < 1e-9,
      "Composite mass must include core, adhesive and facing in kg/m2");
    check(MaterialLibrary.require("fr-acm").physical.propertiesAssumed == true, "Planning density lost provenance");
    rejected(() -> {new LaminatedPanel("bad-coat", [{materialId:"fr-acm", thicknessMm:4}], RouterCut, "trim", PowderCoat);}, "bad-coat");
    rejected(() -> {new LaminatedPanel("bad-layer", [{materialId:"pp", thicknessMm:-1}], RouterCut, "trim");}, "thickness");
    rejected(() -> {new LaminatedPanel("no-layers", [], RouterCut, "trim");}, "layers");
    rejected(() -> {new LaminatedPanel("bad-fire", [{materialId:"pp", thicknessMm:6}], RouterCut, "trim", None,
      {coolantRecord:null, heatLimitC:null, fireClass:"claimed", fireRecord:null, impactRecord:null});}, "source record");
    rejected(() -> {new LaminatedPanel("bad-property", [{materialId:"pp", thicknessMm:6}], RouterCut, "trim", None,
      null, {value:1e9, basis:machinekit.transmission.ValueBasis.Measured, source:""});}, "missing source");
    rejected(() -> {PanelCatalogue.require("missing");}, "Unknown panel");
    Sys.println('Panel catalogue: $checks checks passed');
  }
}
