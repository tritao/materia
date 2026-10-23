import materia.casting.BoxAlgebra;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.CastingProcessSpec.ProcessSpecs;
import materia.tooling.process.FillAnalysis;
import materia.tooling.process.CastingProcessAnalysis;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan.MoldBox;
import materia.tooling.casting.CastingMoldPlan;

class ProcessTests {
  static var assertions = 0;

  static function items(v:Dynamic):Array < Dynamic > return cast v;

  static function check(ok:Bool, label:String):Void {
    assertions++;
    if (!ok) throw label;
  }

  static function near(actual:Null<Float>, expected:Float, tolerance:Float, label:String):Void check(
    actual != null && Math.abs(actual - expected) <= tolerance,
    label + ": " + actual + " vs " + expected
  );

  static function rejects(fn:Void -> Void, expected:String):Void {
    var error = "";
    try fn() catch (e:Dynamic) error = Std.string(e);
    check(error.indexOf(expected) >= 0, 'Expected "$expected", got "$error"');
  }

  static function box(x:Float, z:Float, width:Float, height:Float):MoldBox return {
    x: x,
    y: 0,
    z: z,
    width: width,
    depth: 2,
    height: height
  };

  public static function spec():CastingProcessSpec return {id: "process-test",
    revision: 1, recipeFingerprint: haxe.crypto.Sha256.encode("test-recipe"),
    pourRateMm3PerSecond: null, maximumAirEscapeWidthMm: null, mixComposition: null, mixer: null, castability: {
    maximumAggregateSizeMm: null,
    minimumPassageAggregateRatio: null,
    evidence:[]
  }, compound: {
    reference: materia.casting.CastingInput.CastingTextInput.Unresolved("Production compound and datasheet"),
    thickness: CompoundThicknessLimit.Unresolved("Compound section/layer thickness"),
    maximumMassPerLiftKg: materia.casting.CastingInput.Unresolved("Compound mass per lift")
  }
  };

  /** Independent dense-height reference: explicit face graph, no event-plane
	 * solver or BoxAlgebra.groups. Its first sampled trapped volume approaches
	 * the analytic right-hand limit as the height step shrinks.
	 */
  static function sampledTraps(region:Array<MoldBox>, step:Float):Float {
    var prior:Array<Array < Int>> = [], total = 0.0, index = 0;
    while ((index + 0.37) * step < 10) {
      var height =(index + 0.37) * step;
      var cells:Array < {
        id:Int, box:MoldBox
      }
      > = [];
      for (i in 0...region.length) {
        var b = region[i], z = Math.max(height, b.z), top = b.z + b.height;
        if (top > z) cells.push({id: i, box: {x: b.x, y: b.y, z: z, width: b.width, depth: b.depth, height: top - z}
        }
        );
      } while (cells.length > 0) {
        var group = [cells.shift()], cursor = 0;
        while (cursor < group.length) {
          var a = group[cursor++].box, j = cells.length - 1;
          while (j >= 0) {
            var b = cells[j].box;
            var overlap = [
              Math.min(a.x + a.width, b.x + b.width) - Math.max(a.x, b.x),
              Math.min(
                a.y + a.depth,
                b.y + b.depth
              ) - Math.max(
                a.y,
                b.y
              ),
              Math.min(
                a.z + a.height,
                b.z + b.height
              ) - Math.max(
                a.z,
                b.z
              )
            ];
            if (overlap.filter(v -> v < -1e-8).length == 0
              && overlap.filter(v -> v > 1e-8).length >= 2) group.push(cells.splice(
                j,
                1
              )[0]);
            j--;
          }
        }
        if (group.filter(v -> v.box.z + v.box.height >= 10 - 1e-8).length > 0) continue;
        var known = false;
        for (old in prior) for (cell in group) if (old.indexOf(cell.id) >= 0) known = true;
        if (!known) {
          prior.push([for (cell in group) cell.id]);
          for (cell in group) total += cell.box.width * cell.box.depth * cell.box.height;
        }
      }
      index++;
    }
    return total;
  }

  public static function run(p:CastingMoldPlan):Void {
    var airRecord =
      materia.tooling.ToolingCodec.canonical(materia.tooling.process.FillAnalysis.FillPocketGeometry.CadAirAbove(
        2,
        "region-example"
      ));
    check(
      airRecord.indexOf("cad-air-above") >= 0 && airRecord.indexOf("boxes") < 0,
      "native air references never masquerade as box regions"
    );
    rejects(
      () -> materia.tooling.ToolingCodec.canonical(materia.tooling.process.FillAnalysis.FillPocketGeometry.CadAirAbove(
        Math.NaN,
        "region-example"
      )),
      "Invalid CAD air-region height"
    );
    rejects(
      () -> materia.tooling.ToolingCodec.canonical(materia.tooling.process.FillAnalysis.FillPocketGeometry.CadAirAbove(
        2,
        "region invalid"
      )),
      "Invalid tooling identifier"
    );
    rejects(
      () -> materia.tooling.ToolingCodec.canonical(materia.tooling.process.FillAnalysis.FillOpeningGeometry.CadOpening(
        Math.POSITIVE_INFINITY,
        []
      )),
      "Invalid CAD opening height"
    );
    rejects(
      () -> materia.tooling.ToolingCodec.canonical(materia.tooling.process.FillAnalysis.FillOpeningGeometry.CadOpening(
        2,
        [
          "cover",
          "cover"
        ]
      )),
      "Duplicate CAD opening blocker ID"
    );

    var region = [box(0, 0, 2, 10), box(2, 0, 2, 2), box(4, 0, 4, 6)];
    var boundaries = [{id: "ceiling-feature", box: box(4, 6, 4, 4)}
    ];
    var cores = [{id: "foam", geometry: materia.tooling.casting.MoldRegions.box(box(10, 1, 2, 3))}
    ];
    function fill(cells:Array<MoldBox>, ? extra:Array<Float>):FillReport return FillAnalysis.analyze(
      "body",
      "source",
      "process",
      cells,
      boundaries,
      cores,
      10,
      2400,
      10,
      2,
      null,
      extra
    );
    var trapped = fill(region);
    check(trapped.trappedPockets.length == 1, "downward-ceiling pocket caught");
    var pocket = trapped.trappedPockets[0];
    near(pocket.sealedAtMm, 2, 1e-10, "exact neck closure event");
    near(pocket.volumeMm3, 32, 1e-10, "analytic trapped pocket volume");
    near(pocket.proposedVentPoint.z, 6, 1e-10, "vent proposal at ceiling");
    check(pocket.sourceIds.indexOf("ceiling-feature") >= 0, "pocket has source feature ownership");
    check(trapped.validation.filter(r -> r.status == "fail").length == 1, "trapped pocket blocks readiness");
    var dense = fill(region, [for (i in 0...1001) i / 100.0]);
    for (level in dense.levels) {
      var h = level.heightMm;
      var analytic = 4 * Math.min(h,
        10) + 4 * Math.min(h, 2) + 8 * Math.min(h, 6) - 8 * Math.max(0, Math.min(h, 6) - 2);
      near(level.mixVolumeMm3, analytic, 1e-9, "dense level curve excludes air above its side-neck seal");
    }
    near(trapped.totalMixVolumeMm3, 64, 1e-9, "trapped air is excluded from final mix quantity");
    near(trapped.geometricResinVolumeMm3, 96, 1e-9, "geometric cavity volume remains separately available");
    var multiple = fill(region.concat([box(-6, 0, 4, 8), box(-2, 0, 2, 3)]));
    check(multiple.trappedPockets.length == 2, "two independently side-sealed pockets");
    near(
      multiple.trappedPockets[0].volumeMm3 + multiple.trappedPockets[1].volumeMm3,
      72,
      1e-9,
      "multiple pocket volumes counted once"
    );
    var nested = fill(region.concat([box(8, 0, 2, 8)]));
    check(nested.trappedPockets.length == 1 && switch nested.trappedPockets[0].geometry {
      case Rectilinear(_, faces):
        faces.length >= 2;
      default:
        false;
    }, "one trapped component retains nested ceiling levels");
    near(nested.trappedPockets[0].volumeMm3, 56, 1e-9, "nested ceiling limiting volume");
    near(sampledTraps(region, 0.01), 32, 0.1, "dense independent face graph approaches side-neck limiting volume");
    near(
      sampledTraps(region.concat([box(-6, 0, 4, 8), box(-2, 0, 2, 3)]), 0.01),
      72,
      0.1,
      "dense independent graph agrees on multiple pockets"
    );
    near(
      sampledTraps(region.concat([box(8, 0, 2, 8)]), 0.01),
      56,
      0.1,
      "dense independent graph agrees on nested ceilings"
    );
    var twoTops = [box(0, 0, 2, 10), box(2, 0, 2, 2), box(4, 0, 2, 10)];
    var covered = FillAnalysis.analyze("body", "s", "p", twoTops, [], [], 10, 2400, 10, null, null, [5], [box(
      4,
      0,
      2,
      1
    )], [{
      id: "low-wall",
      bottomMm: 0
    }, {
      id: "high-wall",
      bottomMm: 4
    }
    ]);
    near(covered.openFillAreaMm2, 4, 1e-9, "bridge footprint is subtracted from open fill area");
    check(covered.trappedPockets.length == 1, "tool-covered top cannot vent an isolated pocket");
    near(covered.trappedPockets[0].sealedAtMm, 2, 1e-9, "covered top seals at side neck");
    var halfway = covered.levels.filter(v -> v.heightMm == 5)[0];
    near(halfway.panelHeads.filter(v -> v.id == "low-wall")[0].headPa, 120, 1e-9, "per-level low panel head");
    near(halfway.panelHeads.filter(v -> v.id == "high-wall")[0].headPa, 24, 1e-9, "per-level elevated panel head");
    rejects(
      () -> FillAnalysis.analyze(
        "body",
        "s",
        "p",
        twoTops,
        [],
        [],
        10,
        2400,
        10,
        null,
        null,
        null,
        [box(
          0,
          0,
          6,
          1
        )]
      ),
      "Fill face has zero open area"
    );
    region.reverse();
    check(ToolingCodec.canonical(fill(region)) == ToolingCodec.canonical(trapped), "fill independent of member order");
    var vented = region.concat([box(6, 6, 2, 4)]), ventedReport = fill(vented);
    check(ventedReport.trappedPockets.length == 0, "real open vent path clears trapped pocket");
    check(
      ventedReport.validation.filter(r -> r.rule == "process.fill.connected-air" && r.status == "pass").length == 1,
      "vented geometry passes topology"
    );
    check(
      trapped.validation.filter(r -> r.rule == "process.fill.air-escape-width" && r.status == "pending").length == 1,
      "unknown ceiling width stays pending"
    );
    var widthFail = FillAnalysis.analyze("body", "s", "p", region, boundaries, [], 10, 2400, 10, null, 1);
    check(
      widthFail.validation.filter(r -> r.rule == "process.fill.air-escape-width" && r.status == "fail").length == 1,
      "wide ceiling fails declared limit"
    );
    var widthPass = FillAnalysis.analyze("body", "s", "p", region, boundaries, [], 10, 2400, 10, null, 100);
    check(
      widthPass.validation.filter(r -> r.rule == "process.fill.air-escape-width" && r.status == "pass").length == 1,
      "ceiling passes declared large limit"
    );
    for (level in trapped.levels) {
      near(level.mixMassKg, level.mixVolumeMm3 * 2400e-9, 1e-12, "level mass units");
      near(level.timeSeconds, level.mixVolumeMm3 / 2, 1e-10, "declared pour-rate time");
    }
    var atSubmersion = trapped.levels.filter(r -> r.heightMm == 4)[0],
      finalLevel = trapped.levels[trapped.levels.length - 1];
    near(atSubmersion.cores[0].grossUpliftN, 12 * 2400 * 10e-9, 1e-12, "gross uplift units");
    near(atSubmersion.cores[0].grossUpliftN, finalLevel.cores[0].grossUpliftN, 1e-12, "uplift peaks at submersion");
    near(finalLevel.maximumHeadPa, 240, 1e-10, "final liquid head pressure");
    rejects(() -> fill(region, [11]), "Invalid extra fill height");
    rejects(
      () -> FillAnalysis.analyze("body", "s", "p", region, boundaries, [], 5, 2400, 10, null, null),
      "insufficient fill height"
    );
    var edgeOnly:Array<MoldBox> = [box(0, 0, 2, 10), {x: 2, y: 2, z: 0, width: 2, depth: 2, height: 6}
    ];
    check(fill(edgeOnly).trappedPockets.length == 1, "edge-only contact cannot vent an air region");
    var ceilingFragments:Array<MoldBox> = [{x: 0, y: 0, z: 0, width: 1, depth: 4, height: 10}, {
      x: 1,
      y: 0,
      z: 0,
      width: 1,
      depth: 4,
      height: 6
    }, {
      x: 2,
      y: 0,
      z: 0,
      width: 1,
      depth: 4,
      height: 6
    }, {
      x: 3,
      y: 0,
      z: 0,
      width: 1,
      depth: 4,
      height: 6
    }
    ];
    var combinedCeiling = FillAnalysis.analyze("body", "s", "p", ceilingFragments, [], [], 10, 2400, 10, null, 2.5);
    check(
      combinedCeiling.validation.filter(v -> v.rule == "process.fill.air-escape-width"
      && v.status == "fail").length == 1,
      "artificial cell fragments cannot make a wide connected ceiling pass"
    );
    var overlapCore = FillAnalysis.analyze("body", "s", "p", region, boundaries, [{
      id: "overlap",
      geometry: materia.casting.CastingRegion.Union([
        materia.tooling.casting.MoldRegions.box(box(
          10,
          1,
          2,
          3
        )),
        materia.tooling.casting.MoldRegions.box(box(
          10,
          2,
          2,
          2
        ))
      ])
    }
    ], 10, 2400, 10, null, null);
    near(
      overlapCore.levels[overlapCore.levels.length - 1].cores[0].submergedVolumeMm3,
      12,
      1e-10,
      "overlapping core displacement counted once"
    );
    var s = spec();
    s.recipeFingerprint = ToolingCodec.recipeFingerprint(p.document.recipe);
    check(
      ToolingCodec.canonical(ProcessSpecs.fromRecord(haxe.Json.parse(ToolingCodec.canonical(s))))
      == ToolingCodec.canonical(s),
      "delivered process shop records parse without an intermediate format"
    );
    var tooMany=ProcessSpecs.copy(s);tooMany.castability.evidence=[for (_ in 0...513) "fixture"];
    rejects(() -> ProcessSpecs.copy(tooMany),"Casting process array exceeds limit");
    var highRate:Dynamic=haxe.Json.parse(ToolingCodec.canonical(s));highRate.pourRateMm3PerSecond=1e14;
    near(ProcessSpecs.fromRecord(highRate).pourRateMm3PerSecond,1e14,0,
      "raw shop input retains the previous declared pour-rate range");
    var integrated = CastingProcessAnalysis.analyze(p, s, 9.80665);
    check(integrated.pour != null, "box process includes pour planning");
    if (integrated.pour == null) throw "Missing pour plan";
    var integratedPour:materia.tooling.process.PourPlanning.PourAssessment = cast integrated.pour;
    check(
      integratedPour.sourceFingerprint == integrated.sourceFingerprint
      && integratedPour.fillProcessFingerprint == integrated.processFingerprint,
      "pour consumes the same fingerprinted fill report"
    );
    check(
      integrated.validation.filter(v -> v.rule == "process.pour.batch-schedule" && v.status == "pending").length == 1,
      "missing mixer data remains pending"
    );
    var fabrication = FabricationTests.spec();
    s.compound.reference = materia.casting.CastingInput.CastingTextInput.Known(fabrication.resinReference);
    s.mixComposition = {reference: "synthetic mix", compoundReference: fabrication.resinReference, ingredients:[{
      id: "resin",
      kind: materia.tooling.process.PourPlanning.MixIngredientKind.Resin,
      materialReference: "synthetic resin",
      massFraction: 0.1
    }, {
      id: "aggregate",
      kind: materia.tooling.process.PourPlanning.MixIngredientKind.Aggregate,
      materialReference: "synthetic aggregate",
      massFraction: 0.9
    }
    ]};
    var mass = integratedPour.totalMixMassKg;
    s.mixer = {
      reference: "synthetic mixer",
      capacityKg: mass / 2.5,
      mixingSeconds: 1,
      transferSeconds: 1,
      workingTimeSeconds: 100000,
      workingTimeReference: "fixture"
    };
    s.pourRateMm3PerSecond = 1e9;
    var batched = CastingProcessAnalysis.analyze(p, s, 9.80665, fabrication);
    if (batched.pour == null || batched.sections == null) throw "Missing box schedule/sections";
    var batchedPour:materia.tooling.process.PourPlanning.PourAssessment = cast batched.pour;
    var batchedSections:materia.tooling.process.OrthogonalSections.OrthogonalSectionReport = cast batched.sections;
    if (batchedPour.schedule == null) throw "Missing box schedule";
    var batchPlan:materia.tooling.process.PourPlanning.PourSchedule = cast batchedPour.schedule;
    check(batchPlan.batches.length == 3, "real process includes declared batches");
    for (row in batchedPour.validation) check(
      batched.validation.filter(v -> v.rule == row.rule
      && ToolingCodec.canonical(v) == ToolingCodec.canonical(row)).length == 1,
      "aggregate retains pour rule " + row.rule
    );
    s.compound.thickness = CompoundThicknessLimit.Section(batchedSections.junction.upperDiameterMm);
    s.compound.maximumMassPerLiftKg = materia.casting.CastingInput.Known(mass);
    check(
      CastingProcessAnalysis.analyze(p, s, 9.80665, fabrication).compound.status == "pass",
      "declared datasheet section and lift mass pass"
    );
    s.compound.thickness = CompoundThicknessLimit.Section(0.1);
    var thick = CastingProcessAnalysis.analyze(p, s, 9.80665, fabrication);
    check(thick.compound.status == "fail", "section over datasheet limit fails");
    check(
      thick.castability.proposals.filter(v -> v.rule == "process.compound.datasheet"
      && v.action == materia.tooling.process.CastabilityAnalysis.CastabilityAction.ReduceSection).length > 0,
      "section finding keeps the existing design proposal"
    );
    var maximumLayer = batchPlan.batches[batchPlan.batches.length
      - 1].upperHeightMm - batchPlan.batches[0].lowerHeightMm;
    s.compound.thickness = CompoundThicknessLimit.Layer(maximumLayer);
    check(
      CastingProcessAnalysis.analyze(p, s, 9.80665, fabrication).compound.status == "pass",
      "continuous batch level range satisfies layer limit"
    );
    s.compound.thickness = CompoundThicknessLimit.Layer(maximumLayer / 2);
    check(
      CastingProcessAnalysis.analyze(p, s, 9.80665, fabrication).compound.status == "fail",
      "layer over supplier maximum fails"
    );
    s.compound.thickness = CompoundThicknessLimit.Layer(maximumLayer);
    s.compound.maximumMassPerLiftKg = materia.casting.CastingInput.Known(mass / 2);
    check(
      CastingProcessAnalysis.analyze(p, s, 9.80665, fabrication).compound.status == "fail",
      "splitting mixer batches does not relax the full lift mass limit"
    );
    s.compound.maximumMassPerLiftKg = materia.casting.CastingInput.Known(mass);
    s.compound.reference = materia.casting.CastingInput.CastingTextInput.Known("different compound");
    check(
      CastingProcessAnalysis.analyze(p, s, 9.80665, fabrication).compound.status == "fail",
      "datasheet, weighed mix and fabrication identities agree"
    );
    s.compound.reference = materia.casting.CastingInput.CastingTextInput.Known(fabrication.resinReference);
    s.mixComposition.compoundReference = "different weighed system";
    check(
      CastingProcessAnalysis.analyze(p, s, 9.80665, fabrication).compound.status == "fail",
      "weighed mix identity cannot differ from the declared compound"
    );
    s.mixComposition.compoundReference = fabrication.resinReference;
    s.mixer = null;
    check(
      CastingProcessAnalysis.analyze(p, s, 9.80665, fabrication).compound.status == "pending",
      "missing mixer leaves lift checks pending"
    );
    var raw:Dynamic = haxe.Json.parse(ToolingCodec.canonical(s));
    Reflect.setField(raw, "solver", {}
    );
    rejects(() -> ProcessSpecs.fromRecord(raw), "Unexpected casting record fields");
    s.recipeFingerprint = haxe.crypto.Sha256.encode("stale");
    rejects(() -> CastingProcessAnalysis.analyze(p, s, 9.80665), "Stale process recipe");
    JunctionTests.run();
    CastabilityTests.run(p, spec());
    SectionTests.run(p, spec());
    Sys.println('ProcessTests: $assertions assertions passed');
  }
}
