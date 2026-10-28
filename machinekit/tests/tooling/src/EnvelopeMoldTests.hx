import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import machinekit.tooling.CastingMoldCad;
import machinekit.tooling.ToolingVolume;
import machinekit.structural.MineralFoundation;
import machinekit.component.ComponentValues;

/** The shell bounds size stock; exact source body and filler regions size resin. */
@:access(materia.tooling.casting.CastingMoldPlanner) class EnvelopeMoldTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }

  static function options():CastingMoldOptions return {
    replicationJigs: [],
    foundationJointProcesses: [],
    precision: [],
    compaction: {
      method: Undeclared,
      bedInterfaceReference: null,
      frequencyHz: null,
      amplitudeMm: null,
      durationSeconds: null,
      evidence: []
    },
    serviceOrientation: F.identity(),
    overlapRule: ReservedThenPriority,
    liftingEvidence: []
  };

  static function fixture():Void {
    var values = new ComponentValues().setNumber("width", 100).setNumber("depth", 80).setNumber("height", 120);
    values.setNumber("cornerChamfer", 12);
    var foundation = new MineralFoundation(values, []);
    var document = SinglePocketSetup.document();
    document.input = machinekit.tooling.FoundationToolingAdapter.snapshot(foundation, "chamfered-shell", 1);
    document.setup.sourceFingerprint = ToolingCodec.inputFingerprint(document.input);
    document.setup.sourceToMold = F.identity();

    document.setup.fillHeightMm = 120;
    var boxes = materia.tooling.casting.CastingSourceRegions.derive(document);
    check(
      boxes.regions == null && boxes.validation.filter(row -> row.status == "unsupported"
      && row.affectedIds.indexOf(document.input.id) >= 0).length == 1,
      "Box-only consumers retain an explicit source-envelope Unsupported result"
    );
    var plan = CastingMoldCad.plan(document, options());
    check(plan.formers.length == 4, "Each envelope complement island is an independent former");
    check(
      plan.document.input.envelope.length == document.input.envelope.length,
      "Shell planning preserves source envelope pieces"
    );
    check(
      plan.validation.filter(row -> row.rule == "tooling.feature.box-geometry" && row.status == "pass").length == 1,
      "Native regions qualify envelope tooling without claiming box-only source support"
    );
    var source = foundation.body.geometry(), resin = CastingMoldCad.reconstructed(plan);
    var missing = source.subtractAll([resin]), excess = resin.subtractAll([source]);
    check(missing.volume() + excess.volume() < 1e-5, "Shell and exact fillers reproduce actual foundation CAD");
    missing.close();
    excess.close();
    check(
      Math.abs(plan.handling.castingMassKg - source.volume() * document.recipe.mixDensityKgM3 * 1e-9) < 1e-7,
      "Handling uses actual envelope mass rather than shell bounds"
    );
    check(
      source.volume() < CastingMoldPlanner.volume(plan.cavity),
      "Fixture distinguishes actual resin from shell bounding stock"
    );
    source.close();
    resin.close();
    for (former in plan.formers) {
      var shape = ToolingVolume.geometryOf(former.volume);
      check(shape.solidCount() == 1, "Each filler is one physical connected body");
      shape.close();
    }
    var drawings = machinekit.tooling.ToolingSectionReview.build(plan);
    check(
      drawings.records.length == plan.formers.length
      && drawings.validation.filter(row -> row.status != "pass").length == 0,
      "Every exact envelope filler has line/arc section drawing evidence"
    );
    var review = CastingMoldCad.review(plan);
    check(review.symmetricDifferenceMm3 < 1e-5, "Full review proves exact envelope reconstruction");
    for (name in ["reconstruction", "coverage", "assembly-graph", "demolding-graph"]) check(
      review.validation.filter(row -> row.rule == "tooling.cad." + name && row.status == "pass").length == 1,
      "Envelope mold retains native " + name + " proof"
    );
    check(
      review.demolding.removalOrder.length > plan.formers.length,
      "Complete graph contains walls, fillers and cured casting"
    );
    plan.formers.pop();
    var message = "";
    try CastingMoldCad.review(plan) catch (error:Dynamic) message = Std.string(error);
    check(
      message.indexOf("Casting reconstruction differs by") >= 0,
      "Missing envelope filler fails exact reconstruction: " + message
    );
  }

  static function main():Void {
    fixture();
    Sys.println('EnvelopeMoldTests: $assertions assertions passed');
  }
}
