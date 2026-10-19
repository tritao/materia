package machinekit.enclosure;

enum EnclosureZone {
  WetChip;
  Splash;
  Chute;
  Impact;
  Window;
  Cladding;
  Thermal;
}

enum EnclosureProcess {
  WetMachining;
  DryDust;
  HeatedChamber;
  Press;
}

typedef PanelUse = {
  var panelId:String;
  var construction:LaminatedPanel;
  /** Zones overlap: a window or wet panel can also face an impact demand. */
  var zones:Array<EnclosureZone>;
  var edgesSealed:Bool;
  var seamsSealed:Bool;
}

typedef EnclosureRequirements = {
  var process:EnclosureProcess;
  var maximumTemperatureC:Float;
  var oilMist:Bool;
  var extractionProvided:Bool;
  var drainageProvided:Bool;
  var chamberControlProvided:Bool;
  var boundaryGuardProvided:Bool;
}

typedef EnclosureViolation = {
  var panelId:String;
  var zone:String;
  var rule:String;
  var message:String;
}

/** Catalogue validation reports unknown qualifications as unresolved, never as passing protection. */
class EnclosureRules {
  public static function check(panels:Array<PanelUse>, needs:EnclosureRequirements):Array<EnclosureViolation> {
    if (!Math.isFinite(needs.maximumTemperatureC)) throw "Enclosure needs a finite maximum temperature";
    var violations:Array<EnclosureViolation> = [];
    function issue(id:String, zone:String, rule:String, message:String):Void
      violations.push({panelId:id, zone:zone, rule:rule, message:message});
    switch needs.process {
      case WetMachining:
        if (!needs.extractionProvided) issue("enclosure", "process", "mist-extraction", "Provide mist extraction");
        if (!needs.drainageProvided) issue("enclosure", "process", "drainage", "Provide a sealed floor-to-trough path");
      case DryDust:
        if (!needs.extractionProvided) issue("enclosure", "process", "dust-extraction", "Provide dust extraction");
      case HeatedChamber:
        if (!needs.chamberControlProvided) issue("enclosure", "process", "chamber-control", "Provide chamber temperature control");
      case Press:
        if (!needs.boundaryGuardProvided) issue("enclosure", "process", "boundary-guard", "Provide guarded access at the open boundary");
    }
    var ids = new Map<String,Bool>();
    for (use in panels) {
      if (use.panelId == null || StringTools.trim(use.panelId).length == 0 || ids.exists(use.panelId))
        throw "Enclosure panel IDs must be nonempty and unique";
      ids.set(use.panelId, true);
      use.construction.validate();
      var panel = use.construction, q = panel.qualifications;
      if (use.zones.length == 0) issue(use.panelId, "unassigned", "zone-required", "Assign at least one zone");
      var acm = false, facing = false, polycarbonate = false;
      for (layer in panel.layers) {
        if (layer.materialId == "fr-acm") acm = true;
        if (layer.materialId == "grp" || layer.materialId == "stainless-sheet") facing = true;
        if (layer.materialId == "polycarbonate") polycarbonate = true;
      }
      // Only a sourced heat limit is acceptance evidence. An assumed value remains planning data.
      if (q.heatLimitC == null || q.heatLimitC.basis == machinekit.transmission.ValueBasis.Assumed)
        issue(use.panelId, "all", "heat-qualification", "Supply a sourced construction heat limit");
      else if (q.heatLimitC.value < needs.maximumTemperatureC)
        issue(use.panelId, "all", "heat-limit", "Construction heat limit is below the process maximum");
      if ((needs.oilMist || needs.process == HeatedChamber) && (q.fireClass == null || q.fireRecord == null))
        issue(use.panelId, "all", "fire-qualification", "Supply an applicable construction fire classification");
      for (zone in use.zones) {
        var name = Std.string(zone);
        if (acm && (zone == WetChip || zone == Chute))
          issue(use.panelId, name, "acm-zone", "ACM construction is restricted to cladding, qualified splash or thermal zones");
        switch zone {
          case WetChip | Splash | Chute:
            if (q.coolantRecord == null) issue(use.panelId, name, "coolant-qualification", "Supply a record for the actual coolant and construction");
            if (!use.seamsSealed) issue(use.panelId, name, "sealed-seams", "Wet seams need sealing and drip control");
            if (acm && !use.edgesSealed) issue(use.panelId, name, "sealed-acm-edges", "Seal ACM edges and fastener penetrations");
            if (acm && !facing) issue(use.panelId, name, "acm-facing", "Paint alone does not qualify ACM for splash");
          case Impact:
            if (q.impactRecord == null) issue(use.panelId, name, "impact-qualification", "Supply a construction rating/test covering demand, thickness, joints and mounting");
          case Window:
            if (!polycarbonate) issue(use.panelId, name, "window-material", "Viewing windows require polycarbonate");
            if (q.impactRecord == null) issue(use.panelId, name, "window-qualification", "Polycarbonate alone does not qualify a viewing assembly");
          case Cladding | Thermal:
        }
      }
    }
    return violations;
  }
}
