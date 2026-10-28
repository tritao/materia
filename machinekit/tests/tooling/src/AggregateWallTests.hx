import cadkit.modeling.Part;
import cadkit.modeling.Model;
import cadkit.modeling.Scope;
import cadkit.modeling.Vector;
import materia.casting.CastingPatterns as P;
import materia.casting.CastingFeature.CastingFeatureKind;
import materia.tooling.process.AggregateGate;
import materia.tooling.process.CastingProcessSpec.CastabilityLimits;
import machinekit.tooling.CastingWallReview;
import machinekit.tooling.CastingWallReview.WallObstacle;

class AggregateWallTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
  static function limits():CastabilityLimits return {
    maximumAggregateSizeMm: 8,
    minimumWallAggregateRatio: 5,
    steelAggregateMarginMm: 5,
    steelSpacingAggregateRatio: 4.0 / 3.0,
    minimumPassageAggregateRatio: 3,
    evidence: ["synthetic test rules"]
  };
  static function obstacle(id:String, part:Part, insert:Bool = false, ?owner:String):WallObstacle {
    var bounds = part.shape.bounds(), a = bounds.get_min(), b = bounds.get_max();
    return {
      ids: [id],
      featureIds: [owner == null ? id : owner],
      assemblyIds : insert ? [owner == null ? id : owner] : [],
      tooling : !insert,
      insert : insert,
      part : part,
      box : {
        x : a.get_x(),
        y : a.get_y(),
        z : a.get_z(),
        width : b.get_x() - a.get_x(),
        depth : b.get_y() - a.get_y(),
        height : b.get_z() - a.get_z()
      }
    };
  }
  static function main():Void {
    var scope = new Scope();
    try {
      var slab = scope.own(Part.box(100, 80, 20));
      var slabGaps = CastingWallReview.measure("slab", slab, [], limits(), scope);
      check(
        slabGaps.length == 1 && Math.abs(slabGaps[0].measuredMm - 20) < 1e-6,
        "a solid slab has one real 20 mm opposing-plane wall"
      );
      var assessed = AggregateGate.evaluate("slab", haxe.crypto.Sha256.encode("slab"), limits(), slabGaps);
      check(
        assessed.failures.length == 1 && assessed.failures[0].requiredMm == 40,
        "8 mm aggregate requires the declared 5x wall"
      );
      var supportLocal = scope.own(Part.box(20, 20, 10)), support = scope.own(supportLocal.translated(new Vector(
        0,
        0,
        -10
      )));
      var mixedSlab = scope.own(Part.box(100, 80, 30));
      var mixed = CastingWallReview.measure(
        "mixed-owner",
        mixedSlab,
        [obstacle("local-steel-contact", support, true)],
        limits(),
        scope
      );
      check(
        mixed.filter(gap -> gap.kind == materia.tooling.process.AggregateGate.AggregateGapKind.Wall
        && Math.abs(gap.measuredMm - 30) < 1e-6).length > 0,
        "a local insert contact does not turn the entire coplanar wall into a passage"
      );
      check(
        mixed.filter(gap -> gap.kind == materia.tooling.process.AggregateGate.AggregateGapKind.SteelCover
        && gap.sourceIds.indexOf("local-steel-contact") >= 0 && Math.abs(gap.measuredMm - 30) < 1e-6).length > 0,
        "the actual insert contact footprint retains its own passage attribution"
      );
      var profile = P.box("rounded-bay", Lightening, [P.face("body.z-min")], 40, 40, 42);
      profile.cornerRadius = 3.2;
      var casting = new machinekit.structural.MineralCasting(new machinekit.component.ComponentValues().setNumber(
        "width",
        100
      ).setNumber(
        "depth",
        80
      ).setNumber(
        "height",
        60
      ).setToken(
        "features",
        P.encode([profile])
      ));
      var source = scope.own(casting.geometry()),
        cut = scope.own(machinekit.structural.CastingCavityCad.part(casting.finishedCavities()[0]));
      var gaps = CastingWallReview.measure("rounded", source, [obstacle("rounded-bay", cut)], limits(), scope);
      check(
        gaps.filter(gap -> Math.abs(gap.measuredMm - 18) < 1e-6
        && gap.sourceIds.indexOf("rounded-bay") >= 0).length > 0,
        "finished rounded cavity has the exact 18 mm roof"
      );
      check(
        gaps.filter(gap -> gap.measuredMm < 18 - 1e-6).length == 0,
        "corner arcs do not invent thin material partitions"
      );
      var body = scope.own(Part.box(120, 100, 80)), voids:Array<Part> = [];
      for (item in [{id: "left",
        x: -30.0, w: 40.0}, {id: "bridge", x: 0.0, w: 20.0}, {id: "right", x: 30.0, w: 40.0}]) {
        var local = scope.own(Part.box(item.w, 40, 40));
        voids.push(scope.own(local.translated(new Vector(item.x, 0, 20))));
      }
      var network = scope.own(body.subtractAll(voids));
      var networkGaps = CastingWallReview.measure(
        "network",
        network,
        [for (index in 0...voids.length) obstacle(["left", "bridge", "right"][index], voids[index])],
        limits(),
        scope
      );
      check(
        networkGaps.filter(gap -> gap.region.x >= -10 - 1e-6 && gap.region.x + gap.region.width <= 10 + 1e-6
        && gap.region.y >= -20 - 1e-6 && gap.region.y + gap.region.depth <= 20 + 1e-6
        && gap.region.z >= 20 - 1e-6 && gap.region.z + gap.region.height <= 60 + 1e-6).length == 0,
        "a 20 mm air bridge between two cavities is not reported as a material wall"
      );
      var slabWithBubble = scope.own(Part.box(100, 80, 30)), bubbleLocal = scope.own(Part.box(20, 20, 5));
      var bubble = scope.own(bubbleLocal.translated(new Vector(0,
        0, 12))), bubbleSource = scope.own(slabWithBubble.subtract(bubble));
      var bubbleGaps = CastingWallReview.measure("bubble", bubbleSource, [obstacle("bubble", bubble)], limits(), scope);
      check(
        bubbleGaps.filter(gap -> Math.abs(gap.measuredMm - 12) < 1e-6).length > 0
        && bubbleGaps.filter(gap -> Math.abs(gap.measuredMm - 13) < 1e-6).length > 0,
        "intervening voids retain their real 12 and 13 mm walls without inventing partition thicknesses"
      );
      var insertBody = scope.own(Part.box(100, 80, 60)), local = scope.own(Part.cylinderSpan(4, 40, 60));
      var insert = scope.own(local.translated(new Vector(40, 0, 0))), resin = scope.own(insertBody.subtract(insert));
      var insertGaps = CastingWallReview.measure(
        "insert-case",
        resin,
        [obstacle("source-insert", insert, true)],
        limits(),
        scope
      );
      var narrow = insertGaps.filter(gap -> gap.kind
        == materia.tooling.process.AggregateGate.AggregateGapKind.SteelCover && Math.abs(gap.measuredMm - 6) < 1e-6);
      check(
        narrow.length > 0 && narrow[0].witness != null,
        "circular insert clearance is measured on native surfaces and has a real witness"
      );
      var insertReport = AggregateGate.evaluate(
        "insert-case",
        haxe.crypto.Sha256.encode("insert"),
        limits(),
        insertGaps
      );
      check(
        insertReport.failures.filter(row -> Math.abs(row.gap.measuredMm - 6) < 1e-6 && row.requiredMm == 13).length > 0,
        "steel cover uses dg+5, separately from flow passages and 5x walls"
      );
      for (sample in [
        {
          id: "bubble",
          source: bubbleSource,
          body: slabWithBubble,
          obstacles: [obstacle("bubble", bubble)],
          expected: bubbleGaps
        },
        {
          id: "insert-case",
          source: resin,
          body: insertBody,
          obstacles: [obstacle("source-insert", insert, true)],
          expected: insertGaps
        }
      ]) {
        var localGaps = CastingWallReview.measure(
          sample.id,
          sample.source,
          sample.obstacles,
          limits(),
          scope,
          sample.body
        );
        function signature(rows:Array<materia.tooling.process.AggregateGate.AggregateGap>):String {
          var values = [for (row in rows) row.kind
            + "/" + Math.round(row.measuredMm * 1e6) + "/" + row.sourceIds.join(",")];
          values.sort(Reflect.compare);
          return values.join(";");
        }
        check(
          signature(localGaps) == signature(sample.expected),
          "local distributive Booleans preserve complete-source findings: " + sample.id
        );
      }
      var loftBody = scope.own(Part.box(80, 80, 30));
      var socketLocal =
        scope.own(machinekit.structural.InsertBodies.part(materia.casting.InsertSpec.InsertBody.CrimpedSocket(
          materia.casting.CastingSolid.Cylinder(8, 10),
          {length: 10, width: 10, thickness: 2, wave: 1, blend: 1}
        )));
      var socket = scope.own(socketLocal.translated(new Vector(0,
        0, 5))), loftResin = scope.own(loftBody.subtract(socket));
      var coverage:Array<materia.tooling.ToolingValidation> = [];
      var loftGaps = CastingWallReview.measure(
        "loft-case",
        loftResin,
        [obstacle("crimped-socket", socket, true)],
        limits(),
        scope,
        loftBody,
        coverage
      );
      check(
        coverage.length > 0
        && coverage.filter(row -> row.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported
        && row.affectedIds.indexOf("crimped-socket") >= 0).length == coverage.length,
        "unsupported loft shadows name their owner without stopping independent wall checks"
      );
      check(
        loftGaps.filter(row -> Math.abs(row.measuredMm - 5) < 1e-6).length > 0,
        "known roof and floor gaps survive a separate loft-shadow capability gap"
      );
      var assemblyBody = scope.own(Part.box(100, 80, 60));
      var plateLocal = scope.own(Part.box(40, 40, 5)), headLocal = scope.own(Part.box(20, 20, 3));
      var plate = scope.own(plateLocal.translated(new Vector(0,
        0, 20))), head = scope.own(headLocal.translated(new Vector(
          0,
          0,
          27.2
        )));
      var assemblyResin = scope.own(assemblyBody.subtractAll([plate, head]));
      var same = CastingWallReview.measure(
        "assembly",
        assemblyResin,
        [obstacle("plate", plate, true, "insert-A"), obstacle("head", head, true, "insert-A")],
        limits(),
        scope,
        assemblyBody
      );
      var bodyBox:materia.casting.Box3 = {x: -50, y: -40, z: 0, width: 100, depth: 80, height: 60};
      var cuts = [obstacle("plate", plate).box, obstacle("head", head).box];
      var fill = materia.tooling.process.FillAnalysis.analyze(
        "assembly",
        "source",
        "process",
        materia.casting.BoxAlgebra.subtract([bodyBox], cuts),
        [],
        [],
        60,
        2400,
        9.81,
        null,
        100
      );
      var assessedSame = AggregateGate.evaluate("assembly", "source", limits(), same, fill);
      var own = assessedSame.groups.filter(group -> group.kind
        == materia.tooling.process.AggregateGate.AggregateGapKind.WithinAssembly);
      check(
        own.length == 1
        && Math.abs(own[0].minimumMeasuredMm - 2.2) < 1e-6 && own[0].requiredMm == null && own[0].status == "pass",
        "an open 2.2 mm gap within one insert assembly is not a 24 mm passage failure"
      );
      check(
        own[0].witnessCount > 1 && own[0].featurePair.join("/") == "insert-A/insert-A",
        "multiple head/plate witnesses become one row for the source feature pair and rule"
      );
      var separate = CastingWallReview.measure(
        "assembly",
        assemblyResin,
        [obstacle("plate", plate, true, "insert-A"), obstacle("head", head, true, "insert-B")],
        limits(),
        scope,
        assemblyBody
      );
      var separateReport = AggregateGate.evaluate("assembly", "source", limits(), separate, fill);
      var spacing = separateReport.groups.filter(group -> group.kind
        == materia.tooling.process.AggregateGate.AggregateGapKind.SteelSpacing);
      check(
        spacing.length == 1 && spacing[0].requiredMm == 13 && spacing[0].status == "fail",
        "the same physical gap between separate insert assemblies fails the 13 mm steel spacing rule"
      );
      var touchingHead = scope.own(headLocal.translated(new Vector(0, 0, 25)));
      var touchingResin = scope.own(assemblyBody.subtractAll([plate, touchingHead]));
      var touchingGaps = CastingWallReview.measure(
        "assembly",
        touchingResin,
        [obstacle("plate", plate, true, "insert-A"), obstacle("head", touchingHead, true, "insert-B")],
        limits(),
        scope,
        assemblyBody
      );
      var touchingReport = AggregateGate.evaluate("assembly", "source", limits(), touchingGaps, fill);
      check(
        touchingReport.groups.filter(group -> group.kind
        == materia.tooling.process.AggregateGate.AggregateGapKind.SteelSpacing
        && group.minimumMeasuredMm == 0 && group.status == "fail").length == 1,
        "a zero-width contact between separate inserts cannot disappear from the resin-only spacing check"
      );
      var neck:Array<materia.casting.Box3> = [
        {x: 0, y: 0, z: 0, width: 2, depth: 2, height: 10},
        {x: 2, y: 0, z: 0, width: 2, depth: 2, height: 2},
        {x: 4, y: 0, z: 0, width: 4, depth: 2, height: 6}
      ];
      var trapped = materia.tooling.process.FillAnalysis.analyze(
        "assembly",
        "source",
        "process",
        neck,
        [{id: "insert-A", box: {x: 4, y: 0, z: 6, width: 4, depth: 2, height: 1}}],
        [],
        10,
        2400,
        9.81,
        null,
        100
      );
      var trappedReport = AggregateGate.evaluate("assembly", "source", limits(), same, trapped);
      check(
        trapped.trappedPockets.length > 0 && trappedReport.groups.filter(group -> group.kind
        == materia.tooling.process.AggregateGate.AggregateGapKind.WithinAssembly)[0].status == "fail",
        "an owner-attributed narrow dead-end recess remains a failure under the matrix-fines exemption"
      );
      var left = scope.own(Part.box(10, 10, 10)), right = scope.own(left.translated(new Vector(17, 0, 0)));
      var witness = left.shape.distanceWitness(right.shape);
      check(
        Math.abs(witness.get_distance() - 7) < 1e-9 && witness.get_count() > 0,
        "distance witness preserves the native scalar distance"
      );
      var a = Vector.fromNative(witness.get_first()), b = Vector.fromNative(witness.get_second());
      check(Math.abs(a.subtract(b).length() - 7) < 1e-9, "witness points reproduce the measured distance");
      var message = "";
      try left.shape.distanceWitness(right.shape,
        witness.get_count()) catch (error:Dynamic) message = Std.string(error);
      check(message.indexOf("index out of range") >= 0, "native witness rejects an invalid solution index");
      var contact = left.shape.distanceWitness(left.shape);
      check(contact.get_count() == 0 && contact.get_distance() == 0, "contact has no positive-gap witness");
      scope.close();
    } catch (error:Dynamic) {
      scope.close();
      throw error;
    }
    Sys.println("AggregateWallTests: " + assertions + " assertions passed");
  }
}
