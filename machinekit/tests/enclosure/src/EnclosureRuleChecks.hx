import machinekit.enclosure.EnclosureRules;
import machinekit.enclosure.LaminatedPanel;
import machinekit.enclosure.PanelCatalogue;
import machinekit.transmission.ValueBasis;

class EnclosureRuleChecks {
  public static function run():Void {
    var needs:EnclosureRequirements = {process:WetMachining, maximumTemperatureC:60, oilMist:false,
      extractionProvided:true, drainageProvided:true, chamberControlProvided:false, boundaryGuardProvided:false};
    var use:PanelUse = {panelId:"door", construction:PanelCatalogue.require("fr-acm-4"),
      zones:[Splash, Impact], edgesSealed:false, seamsSealed:false};
    var failures = EnclosureRules.check([use], needs);
    for (rule in ["heat-qualification", "coolant-qualification", "sealed-seams", "sealed-acm-edges", "acm-facing", "impact-qualification"]) {
      var found = false;
      for (failure in failures) if (failure.rule == rule && failure.panelId == "door") found = true;
      if (!found) throw 'Missing named rule $rule';
    }
    use.zones = [WetChip];
    var blocked = false;
    for (failure in EnclosureRules.check([use], needs)) if (failure.rule == "acm-zone" && failure.zone == "WetChip") blocked = true;
    if (!blocked) throw "Plain ACM admitted to direct chips";
    use.construction = PanelCatalogue.require("polycarbonate-6"); use.zones = [Window];
    blocked = false;
    for (failure in EnclosureRules.check([use], needs)) if (failure.rule == "window-qualification") blocked = true;
    if (!blocked) throw "Unrated polycarbonate admitted as protective window";
    needs.oilMist = true;
    blocked = false;
    for (failure in EnclosureRules.check([use], needs)) if (failure.rule == "fire-qualification") blocked = true;
    if (!blocked) throw "Oil mist admitted without a fire record";
    needs.oilMist = false; use.zones = [Splash, Impact];
    // Explicit test fixture: the records below are synthetic, not catalogue qualifications.
    use.construction = new LaminatedPanel("qualified-fixture", [{materialId:"stainless-sheet", thicknessMm:2}],
      PressBrakeFold, "welded", None, {coolantRecord:"fixture: coolant-A", impactRecord:"fixture: demand-A/mount-A",
        heatLimitC:{value:80, basis:Catalog, source:"fixture"}, fireClass:"fixture", fireRecord:"fixture"});
    use.edgesSealed = true; use.seamsSealed = true;
    if (EnclosureRules.check([use], needs).length != 0) throw "Qualified fixture should satisfy declared wet/impact rules";
    needs.maximumTemperatureC = 100;
    if (EnclosureRules.check([use], needs)[0].rule != "heat-limit") throw "Heat demand was ignored";
    needs.maximumTemperatureC = 60;
    use.zones = [Window];
    var windowFailures = EnclosureRules.check([use], needs);
    if (windowFailures.length != 1 || windowFailures[0].rule != "window-material") throw "Steel is not viewing glazing";
    use.zones = [];
    if (EnclosureRules.check([use], needs)[0].rule != "zone-required") throw "Unassigned panel accepted";
    needs.process = DryDust; needs.extractionProvided = false;
    if (EnclosureRules.check([], needs)[0].rule != "dust-extraction") throw "Dust extraction omitted";
    needs.process = WetMachining;
    if (EnclosureRules.check([], needs).length != 1) throw "Mist extraction omitted";
    needs.drainageProvided = false;
    if (EnclosureRules.check([], needs).length != 2) throw "Drainage omitted";
    needs.process = HeatedChamber;
    if (EnclosureRules.check([], needs)[0].rule != "chamber-control") throw "Chamber control omitted";
    needs.process = Press;
    if (EnclosureRules.check([], needs)[0].rule != "boundary-guard") throw "Press boundary guard omitted";
    Sys.println("Enclosure zone/process rules passed");
  }
}
