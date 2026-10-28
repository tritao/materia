package machinekit.tooling;

import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingValidation;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.CastingProcessAnalysis;
import materia.tooling.process.FillAnalysis;

/** Neutral process results enter the same fabrication manifest/readiness gate. */
class ToolingProcessExport {
  public static function write(
    p:CastingMoldPlan,
    spec:CastingProcessSpec,
    gravityMps2:Float,
    dir:String,
    review:Null<machinekit.tooling.CastingMoldCad.CastingMoldReview>,
    ?fabrication:materia.tooling.export.FabricationSpec,
    ?foundation:machinekit.structural.MineralFoundation,
    ?existing:materia.tooling.process.CastingProcessAnalysis.CastingProcessReport,
    ?interfaceContext:materia.casting.CastingInterfaceContext
  ):Array<ToolingValidation> {
    var report = existing == null ? CastingProcessAnalysis.analyze(
      p,
      spec,
      gravityMps2,
      fabrication,
      CastingFillAnalysis.fromPlan,
      CastingWallReview.fromPlan
    ) : existing;
    if (report.sourceFingerprint != materia.tooling.casting.CastingMoldPlanner.fingerprint(p)
      || report.processFingerprint != CastingProcessAnalysis.fingerprint(
        p,
        spec,
        gravityMps2,
        fabrication
      ) || report.castability.sourceFingerprint != report.sourceFingerprint
      || report.castability.processFingerprint != report.processFingerprint) throw "Stale process export report";
    var checks = report.validation.filter(v -> StringTools.startsWith(v.rule, "process."));
    if (report.fill != null) checks = checks.concat(cadVolume(
      p,
      report.fill,
      report.sourceFingerprint,
      dir,
      review,
      foundation,
      interfaceContext
    ));
    else {
      checks.push(FillAnalysis.check(
        "process.fill.cad-volume-agreement",
        [p.document.input.id],
        Unsupported,
        "No exact fill region is available for comparison against foundation CAD.",
        []
      ));
      sys.io.File.saveContent(dir + "/fill-foundation-cad-check.json", "null\n");
    }
    writeCastability(report.castability, report.sections, report.fill, dir);
    if (report.aggregate != null) writeAggregate(p, report.aggregate, dir);
    sys.io.File.saveContent(dir + "/fill-analysis.json", ToolingCodec.canonical(report.fill) + "\n");
    sys.io.File.saveContent(dir + "/casting-process-spec.json", ToolingCodec.canonical(report.spec) + "\n");
    sys.io.File.saveContent(dir + "/compound-datasheet-check.json", ToolingCodec.canonical(report.compound) + "\n");
    sys.io.File.saveContent(
      dir + "/junction-thickness.json",
      ToolingCodec.canonical(report.sections == null ? null : report.sections.junction) + "\n"
    );
    sys.io.File.saveContent(dir + "/section-geometry.json", ToolingCodec.canonical(report.sections) + "\n");
    writePour(report, dir);
    return checks;
  }

  public static function writeAggregate(
    p:CastingMoldPlan,
    report:materia.tooling.process.AggregateGate.AggregateReport,
    dir:String
  ): {
    groups: Array<Array<String>>,
    witnesses: Array<Array<String>>
  } {
    sys.io.File.saveContent(dir + "/aggregate-wall-review.json", ToolingCodec.canonical(report) + "\n");
    var groups:Array<Array<String>> = [[
      "casting",
      "rule_class",
      "feature_a",
      "feature_b",
      "min_measured_mm",
      "required_mm",
      "witness_count",
      "status",
      "source_min_x_mm",
      "source_min_y_mm",
      "source_min_z_mm",
      "source_max_x_mm",
      "source_max_y_mm",
      "source_max_z_mm",
      "criterion"
    ]];
    var witnesses:Array<Array<String>> = [[
      "casting",
      "gap_id",
      "rule_class",
      "feature_a",
      "feature_b",
      "source_ids",
      "measured_mm",
      "required_mm",
      "status",
      "source_min_x_mm",
      "source_min_y_mm",
      "source_min_z_mm",
      "source_max_x_mm",
      "source_max_y_mm",
      "source_max_z_mm",
      "witness_start_x_mm",
      "witness_start_y_mm",
      "witness_start_z_mm",
      "witness_end_x_mm",
      "witness_end_y_mm",
      "witness_end_z_mm",
      "method"
    ]];
    var inverse = materia.assembly.AssemblyFrames.inverse(p.document.setup.sourceToMold);
    function location(b:materia.casting.Box3):Array<String> {
      var region = materia.tooling.casting.CastingMoldPlanner.box(
        b.width,
        b.depth,
        b.height,
        materia.assembly.AssemblyFrames.compose(
          inverse,
          materia.assembly.AssemblyFrames.translation(b.x + b.width / 2, b.y + b.depth / 2, b.z)
        )
      );
      return [for (value in [
        region.x,
        region.y,
        region.z,
        region.x + region.width,
        region.y + region.depth,
        region.z + region.height
      ]) Std.string(value)];
    }
    for (group in report.groups) {
      var row = [
        p.document.input.id,
        (group.kind:String),
        group.featurePair[0],
        group.featurePair[1],
        Std.string(group.minimumMeasuredMm),
        group.requiredMm == null ? "" : Std.string(group.requiredMm),
        Std.string(group.witnessCount),
        (group.status : String)
      ];
      row = row.concat(location(group.region));
      row.push(group.criterion);
      groups.push(row);
    }
    for (gap in report.gaps) {
      var required = materia.tooling.process.AggregateGate.required(gap.kind, report.limits);
      var group = report.groups.filter(group -> group.kind == gap.kind
        && ToolingCodec.canonical(group.featurePair) == ToolingCodec.canonical(gap.featurePair))[0];
      var status = gap.kind == materia.tooling.process.AggregateGate.AggregateGapKind.WithinAssembly
        ? group.status : required == null ? ToolingCheckStatus.Pending
        : gap.measuredMm < required - 1e-7 ? ToolingCheckStatus.Fail : ToolingCheckStatus.Pass;
      var row = [
        p.document.input.id,
        gap.id,
        (gap.kind:String),
        gap.featurePair[0],
        gap.featurePair[1],
        gap.sourceIds.join(" | "),
        Std.string(gap.measuredMm),
        required == null ? "" : Std.string(required),
        (status : String)
      ];
      row = row.concat(location(gap.region));
      var witness = gap.witness;
      if (witness == null) for (_ in 0...6) row.push("");
      else for (point in [witness.first, witness.second]) {
        var source = materia.assembly.AssemblyFrames.transformPoint(inverse, point.x, point.y, point.z);
        for (value in [source.x, source.y, source.z]) row.push(Std.string(value));
      }
      row.push(gap.method);
      witnesses.push(row);
    }
    sys.io.File.saveContent(dir + "/aggregate-groups.csv", materia.tooling.export.ToolingFabrication.csv(groups));
    sys.io.File.saveContent(dir + "/aggregate-witnesses.csv", materia.tooling.export.ToolingFabrication.csv(witnesses));
    var text = [
      "# Aggregate wall, flow and steel clearance review",
      "",
      "Geometry unchanged. Coordinates and bounding boxes in the CSVs are foundation-local millimetres; JSON geometry is in mold coordinates.",
      "One row per source feature pair and rule class: [aggregate-groups.csv](aggregate-groups.csv).",
      "All measured witnesses, including open within-assembly gaps: [aggregate-witnesses.csv](aggregate-witnesses.csv).",
      "An open gap within one insert assembly has no passage-spacing floor; its bulk access uses the existing exact filling topology.",
      "Where a structural wall also carries flow between forming tooling, both rules apply; the 24 mm process rule does not weaken the 40 mm wall rule.",
      ""
    ];
    for (check in report.validation) text.push("- " + check.rule + " (" + check.status + "): " + check.message);
    text.push("\nNo source geometry change is authorised by this report.");
    sys.io.File.saveContent(dir + "/aggregate-wall-review.md", text.join("\n") + "\n");
    return {groups: groups, witnesses: witnesses};
  }

  static function writeCastability(
    castability:materia.tooling.process.CastabilityAnalysis.CastabilityReport,
    sections:Null<materia.tooling.process.OrthogonalSections.OrthogonalSectionReport>,
    fill:Null<materia.tooling.process.FillAnalysis.FillReport>,
    dir:String
  ):Void {
    sys.io.File.saveContent(dir + "/castability-analysis.json", ToolingCodec.canonical(castability) + "\n");
    var text = [
      "# Casting design feedback",
      "",
      "Review suggestions only. Source geometry is unchanged; compound limits, CAD and physical release checks remain independent.",
      "",
      "All dimensions/points are in mold coordinates (mm).",
      ""
    ];
    if (sections != null) {
      var junction = sections.junction;
      text.push("Largest inscribed diameter: " + junction.lowerDiameterMm + ".." + junction.upperDiameterMm
        + " mm; numerical tolerance reached: " + junction.converged + ". Witness: " + ToolingCodec
        .canonical(junction.witness) + "; nearby source faces: " + junction.witnessSourceIds.join(", ") + ".");
    } else text
      .push("Orthogonal section and junction measurements are Unsupported; available source, fill and release findings follow.\n");
    if (fill == null) text.push("Exact fill geometry is Unsupported; no air-pocket or vent locations are inferred.\n");
    for (v in castability.validation) text.push("- `"
      + v.rule + "` (" + v.status + ") [" + v.affectedIds.join(", ") + "]: " + v.message);
    text.push("\n## Suggested actions\n");
    for (proposal in castability.proposals) {
      text.push("- `" + proposal.id + "` — " + proposal.action + " (" + proposal.owner + "), sources: "
        + proposal.sourceIds.join(", ") + "; blocking rule: `" + proposal.rule + "`. " + proposal.instruction);
      if (proposal.measuredMm != null || proposal.targetMm != null) text.push("  Measured: "
        + proposal.measuredMm + " mm; target: " + proposal.targetMm + " mm; axis: " + proposal.axis + ".");
      if (proposal.point != null) text.push("  Candidate point: " + ToolingCodec.canonical(proposal.point) + ".");
    }
    sys.io.File.saveContent(dir + "/castability-review.md", text.join("\n") + "\n");
  }

  static function cadVolume(
    p:CastingMoldPlan,
    fill:FillReport,
    sourceFingerprint:String,
    dir:String,
    review:Null<machinekit.tooling.CastingMoldCad.CastingMoldReview>,
    foundation:Null<machinekit.structural.MineralFoundation>,
    interfaceContext:Null<materia.casting.CastingInterfaceContext>
  ):Array<ToolingValidation> {
    var checks:Array<ToolingValidation> = [];
    if (foundation == null) {
      checks.push(FillAnalysis.check(
        "process.fill.cad-volume-agreement",
        [p.document.input.id],
        ToolingCheckStatus.Pending,
        "No source foundation geometry was supplied for the independent CAD comparison.",
        []
      ));
      sys.io.File.saveContent(dir + "/fill-foundation-cad-check.json", "null\n");
    } else {
      if (review != null && review.fingerprint != sourceFingerprint) throw "Stale process CAD review";
      var actualInput = FoundationToolingAdapter.snapshot(
        foundation,
        p.document.input.id,
        p.document.input.revision,
        interfaceContext
      );
      if (ToolingCodec.inputFingerprint(actualInput)
        != ToolingCodec.inputFingerprint(p.document.input)) throw "Process CAD foundation differs from source revision";
      var resources:Array<cadkit.modeling.Part> = [];
      var difference = 0.0;
      try {
        var local = foundation.body.asCastGeometry();
        resources.push(local);
        var actual = machinekit.assembly.PosedParts.moved(local, p.document.setup.sourceToMold);
        resources.push(actual);
        if (Math.abs(fill.geometricResinVolumeMm3 - actual.volume()) > Math.max(1e-5,
          actual.volume() * 1e-9)) throw "Fill geometric resin volume differs from actual foundation CAD volume";
        var region = CastingMoldCad.source(p);
        resources.push(region);
        var missing = actual.subtractAll([region]);
        resources.push(missing);
        var extra = region.subtractAll([actual]);
        resources.push(extra);
        difference = missing.volume() + extra.volume();
        if (!Math.isFinite(difference) || difference > Math.max(1e-5,
          actual.volume() * 1e-9)) throw "Fill-region/actual foundation symmetric difference: " + difference;
      } catch (error:Dynamic) {
        for (part in resources) part.close();
        throw error;
      }
      for (part in resources) part.close();
      sys.io.File.saveContent(
        dir + "/fill-foundation-cad-check.json",
        ToolingCodec.canonical({
          sourceFingerprint: sourceFingerprint,
          reference: "foundation.body.asCastGeometry() transformed to mold coordinates",
          symmetricDifferenceMm3: difference
        }) + "\n"
      );
      checks.push(FillAnalysis.check(
        "process.fill.cad-volume-agreement",
        [p.document.input.id],
        ToolingCheckStatus.Pass,
        "Symmetric difference against actual foundation.body.asCastGeometry() passed; see fill-foundation-cad-check.json.",
        [sourceFingerprint, "fill-foundation-cad-check.json"]
      ));
    }
    return checks;
  }

  static function writePour(report:materia.tooling.process.CastingProcessAnalysis.CastingProcessReport,
    dir:String):Void {
    var pour = report.pour;
    if (pour == null) {
      sys.io.File.saveContent(dir + "/pour-plan.json", "null\n");
      sys.io.File.saveContent(dir + "/pour-quantities.csv", "ingredient_id,material_reference,mass_kg\n");
      sys.io.File.saveContent(
        dir + "/pour-batches.csv",
        "batch,mass_kg,from_level_mm,to_level_mm,mixing_start_s,pour_start_s,pour_end_s\n"
      );
      sys.io.File.saveContent(dir + "/pour-batch-quantities.csv", "batch,ingredient_id,material_reference,mass_kg\n");
      sys.io.File.saveContent(
        dir + "/pour-plan.md",
        "# Pour plan\n\nUnsupported until the exact non-box fill curve is available. No material quantities or batch schedule are credited.\n"
      );
      return;
    }
    sys.io.File.saveContent(dir + "/pour-plan.json", ToolingCodec.canonical(pour) + "\n");
    var quantities:Array<Array<String>> = [["ingredient_id", "material_reference", "mass_kg"]];
    for (q in pour.quantities) quantities.push([q.ingredientId, q.materialReference, Std.string(q.massKg)]);
    sys.io.File.saveContent(dir + "/pour-quantities.csv", materia.tooling.export.ToolingFabrication.csv(quantities));
    var batches:Array<Array<String>> = [[
      "batch",
      "mass_kg",
      "from_level_mm",
      "to_level_mm",
      "mixing_start_s",
      "pour_start_s",
      "pour_end_s"
    ]];
    var batchQuantities:Array<Array<String>> = [["batch", "ingredient_id", "material_reference", "mass_kg"]];
    var text = [
      "# Pour plan",
      "",
      "Review calculation; process and physical release checks remain in fabrication-validation.json.",
      "",
      "Total wet mix: " + pour.totalMixMassKg + " kg.",
      ""
    ];
    if (report.spec.mixComposition
      == null) text.push("Weighed material quantities are pending the declared mix fractions.");
    else text.push("Weighing quantities: pour-quantities.csv.");
    var schedule = pour.schedule;
    if (schedule == null) text.push("Batch schedule is unresolved; see the validation records below.");
    else {
      for (batch in schedule.batches) {
        batches.push([
          Std.string(batch.index),
          Std.string(batch.massKg),
          Std.string(batch.lowerHeightMm),
          Std.string(batch.upperHeightMm),
          Std.string(batch.mixingStartSeconds),
          Std.string(batch.pourStartSeconds),
          Std.string(batch.pourEndSeconds)
        ]);
        for (q in batch.quantities) batchQuantities.push([
          Std.string(batch.index),
          q.ingredientId,
          q.materialReference,
          Std.string(q.massKg)
        ]);
      }
      text.push("Batch sequence: pour-batches.csv. "
        + schedule.batches.length + " batches; elapsed " + schedule.totalElapsedSeconds
        + " s; minimum declared working-time margin " + schedule.minimumWorkingTimeMarginSeconds + " s.");
      text.push("Individual batch weighings: pour-batch-quantities.csv.");
    }
    text.push("");
    for (check in pour.validation) text.push("- `" + check.rule + "` (" + check.status + "): " + check.message);
    sys.io.File.saveContent(dir + "/pour-batches.csv", materia.tooling.export.ToolingFabrication.csv(batches));
    sys.io.File.saveContent(
      dir + "/pour-batch-quantities.csv",
      materia.tooling.export.ToolingFabrication.csv(batchQuantities)
    );
    sys.io.File.saveContent(dir + "/pour-plan.md", text.join("\n") + "\n");
  }
}
