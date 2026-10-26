package materia.tooling.process;

import materia.tooling.ToolingCodec;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.FillAnalysis;
import materia.casting.CastingInput;

/** Native exact fill at the same critical levels used by the box path. */
typedef ExactFillBackend =(CastingMoldPlan, FillSourceSettings) -> FillAssessment;

typedef CastingProcessReport = {var sourceFingerprint: String;
var processFingerprint:String;
var spec:CastingProcessSpec;
var fill:Null<FillReport>;
var pour:Null<PourPlanning.PourAssessment>;
var compound:ToolingValidation;
var sections:Null<OrthogonalSections.OrthogonalSectionReport>;
var castability:CastabilityAnalysis.CastabilityReport;
var validation:Array<ToolingValidation>;
var fabrication:materia.tooling.ToolingValidation.FabricationReadiness;
}

/** One source-bound fill, geometric screening and supplier-datasheet pass. */
class CastingProcessAnalysis {
  public static function analyze(
    p:CastingMoldPlan,
    raw:CastingProcessSpec,
    gravityMps2:Float,
    ? fabrication:materia.tooling.export.FabricationSpec,
    ? exactFill:ExactFillBackend
  ):CastingProcessReport {
    CastingMoldPlanner.validate(p);
    if (fabrication != null) fabrication = materia.tooling.export.FabricationSpec.FabricationSpecs.copy(fabrication);
    var spec = ProcessSpecs.copy(raw);
    if (spec.recipeFingerprint
      != ToolingCodec.recipeFingerprint(p.document.recipe)) throw "Stale process recipe fingerprint";
    var sourceFingerprint = CastingMoldPlanner.fingerprint(p);
    var fingerprint = CastingProcessAnalysis.fingerprint(p, spec, gravityMps2, fabrication);
    var projection = FillTooling.project(p);
    var panels:Array<FillPanel> = [for (member in p.panels.members.concat(p.supportPanels)) {
      var bounds =
        materia.casting.CastingProfileGeometry.bounds(materia.tooling.panel.PanelContours.outside(member.panel));
      var bottom = Math.POSITIVE_INFINITY;
      for (x in[bounds.x, bounds.x + bounds.width]) for (y in[bounds.y, bounds.y + bounds.height]) for (z in[
        0.0,
        member.panel.stock.measuredThickness
      ]) bottom = Math.min(
        bottom,
        materia.assembly.AssemblyFrames.transformPoint(
          member.pose,
          x,
          y,
          z
        ).z
      );
      {
        id:member.panel.id, bottomMm:Math.max(p.envelope.z, bottom)
      };
    }
    ];
    var fillSettings:FillSourceSettings = {sourceFingerprint: sourceFingerprint,
      processFingerprint: fingerprint, cores:[for (c in p.cores) {
      id:c.id, geometry:c.cavity
    }
    ], densityKgM3:p.document.recipe.mixDensityKgM3, gravityMps2:gravityMps2,
      pourRateMm3PerSecond:spec.pourRateMm3PerSecond, maximumAirEscapeWidthMm:spec.maximumAirEscapeWidthMm,
      extraLevels:[], fillObstructions:projection.blockers, panels:panels};
    var geometry = materia.tooling.casting.CastingSourceRegions.derive(p.document);
    var pourGeometry = materia.tooling.casting.CastingSourceRegions.derive(p.document);
    var fillAssessment:FillAssessment;
    if (pourGeometry.regions == null) fillAssessment = exactFill == null ? {
      report:null, validation:materia.tooling.casting.CastingSourceRegions.checks(pourGeometry, "process.fill.geometry")
    }
    :exactFill(p, fillSettings);
    else fillAssessment = FillAnalysis.assessRegions(pourGeometry, p.document.setup.fillHeightMm, fillSettings);
    if (projection.unsupportedIds.length > 0) fillAssessment.validation.push(FillAnalysis.check(
      "process.fill.tooling-projection",
      projection.unsupportedIds,
      Unsupported,
      "Exact tooling projection is unavailable for these members.",
      []
    ));
    var fill = fillAssessment.report;
    var sectionAssessment = OrthogonalSections.assessRegions(geometry, sourceFingerprint);
    var sections = sectionAssessment.report;
    var castability = CastabilityAnalysis.analyze(
      p.document,
      spec,
      fill,
      sections,
      sourceFingerprint,
      fingerprint,
      p.validation
    );
    var pour = fill == null ? null : PourPlanning.analyze(p.document.input.id, fill, spec.mixComposition, spec.mixer);
    var compound = compoundCheck(p.document.input.id, spec, sections, pour, fabrication);
    if (compound.status == Unsupported && sections == null) {
      var ids = compound.affectedIds.copy();
      for (finding in sectionAssessment.validation) if (finding.status == Unsupported) for (id in finding.affectedIds) {
        var features = p.document.input.features.filter(feature -> feature.id == id);
        var owner = features.length == 1 && features[0].derivedFrom != null ? features[0].derivedFrom : id;
        if (ids.indexOf(owner) < 0) ids.push(owner);
      }
      ids.sort(Reflect.compare);
      compound.affectedIds = ids;
    }
    var checks = p
      .validation.concat(fillAssessment.validation).concat(sectionAssessment.validation).concat(castability.validation);
    if (pour != null) checks = checks.concat(pour.validation);
    checks.push(compound);
    var unique:Map < String, ToolingValidation >=[];
    for (check in checks) {
      var previous = unique.get(check.rule);
      if (previous != null && ToolingCodec.canonical(previous)
        != ToolingCodec.canonical(check)) throw "Conflicting process validation: " + check.rule;
      unique.set(check.rule, check);
    }
    checks = [for (check in unique) check];
    checks.sort((a, b) -> Reflect.compare(a.rule, b.rule));
    return {
      sourceFingerprint: sourceFingerprint,
      processFingerprint: fingerprint,
      spec: spec,
      fill: fill,
      pour: pour,
      compound: compound,
      sections: sections,
      castability: castability,
      validation: checks,
      fabrication: materia.tooling.ToolingValidation.ToolingReadiness.evaluate(checks)
    };
  }

  public static function fingerprint(
    p:CastingMoldPlan,
    raw:CastingProcessSpec,
    gravityMps2:Float,
    ? fabrication:materia.tooling.export.FabricationSpec
  ):String {
    if (!Math.isFinite(gravityMps2) || gravityMps2 <= 0) throw "Invalid process gravity";
    return haxe.crypto.Sha256.encode(ToolingCodec.canonical({source: CastingMoldPlanner.fingerprint(p),
      spec: ProcessSpecs.copy(raw), gravityMps2: gravityMps2, fabrication: fabrication == null ? null : {
      resinReference : fabrication.resinReference,
      adhesiveContainsEPS : fabrication.adhesiveContainsEPS
    }
    }
    ));
  }

  public static function compoundPending(sourceId:String):ToolingValidation return FillAnalysis.check(
    "process.compound.datasheet",
    [sourceId],
    Pending,
    "Declare the production compound datasheet, its section/layer thickness and mass-per-lift limits, and the pour schedule.",
    []
  );

  public static function compoundCheck(
    sourceId:String,
    raw:CastingProcessSpec,
    sections:Null<OrthogonalSections.OrthogonalSectionReport>,
    pour:Null<PourPlanning.PourAssessment>,
    fabrication:Null<materia.tooling.export.FabricationSpec>
  ):ToolingValidation {
    var spec = ProcessSpecs.copy(raw), missing:Array < String >=[], failures:Array < String >=[],
      unsupported:Array < String >=[], notes:Array < String >=[], evidence:Array < String >=[];
    var ids = [sourceId];
    var schedule = pour == null ? null : pour.schedule;
    switch spec.compound.reference {
      case Unresolved(reason):
        missing.push(reason);
      case Known(reference):
        evidence.push(reference);
        if (fabrication == null) missing.push("Fabrication compound identity");
        else if (reference != fabrication
          .resinReference) failures.push("Compound datasheet and fabrication name different resin systems");
        if (spec.mixComposition != null && reference != spec.mixComposition
          .compoundReference) failures.push("Compound datasheet and weighed mix name different resin systems");
    }
    switch spec.compound.thickness {
      case Unresolved(reason):
        missing.push(reason);
      case Section(maximum):
        if (sections == null) unsupported.push("Exact non-box maximum section is unsupported");
        else {
          var bound = sections.junction;
          for (id in bound.witnessSourceIds) if (ids.indexOf(id) < 0) ids.push(id);
          notes.push("Section diameter "
            + bound.lowerDiameterMm + ".." + bound.upperDiameterMm + " mm; datasheet maximum " + maximum + " mm");
          if (bound.lowerDiameterMm > maximum + 1e-7) failures.push("Section exceeds the compound datasheet");
          else if (bound.upperDiameterMm
            > maximum + 1e-7) missing.push("Section interval refinement at the datasheet threshold");
        }
      case Layer(maximum):
        if (schedule == null) missing.push("Mixer and batch level ranges");
        else {
          var layer = schedule.batches[schedule.batches.length - 1].upperHeightMm - schedule.batches[0].lowerHeightMm;
          notes.push("Continuous poured layer "
            + layer + " mm; datasheet maximum " + maximum + " mm. Mixer batches are not separate cured lifts");
          if (layer > maximum + 1e-7) failures.push("Poured layer exceeds the compound datasheet");
        }
    }
    switch spec.compound.maximumMassPerLiftKg {
      case Unresolved(reason):
        missing.push(reason);
      case Known(maximum):
        if (pour == null) missing.push("Exact pour quantity");
        else {
          var mass = pour.totalMixMassKg;
          notes.push("Continuous lift mass " + mass
            + " kg; datasheet maximum " + maximum + " kg. No cooling or cure pause is inferred between mixer batches");
          if (mass > maximum + 1e-7) failures.push("Lift mass exceeds the compound datasheet");
        }
    }
    ids.sort(Reflect.compare);
    var status:ToolingCheckStatus = failures.length
      > 0 ? Fail : unsupported.length > 0 ? Unsupported : missing.length > 0 ? Pending : Pass;
    if (missing.length > 0) notes.push("Missing inputs: " + missing.join("; "));
    return FillAnalysis.check(
      "process.compound.datasheet",
      ids,
      status,
      failures.concat(unsupported).concat(notes).join(". "),
      evidence
    );
  }
}
