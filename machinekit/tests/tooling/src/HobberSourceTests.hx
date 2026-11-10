import hobber.*;
import hobber.HobberLayout;
import materia.casting.BoxAlgebra;
import materia.casting.CastingCavities;
import materia.casting.CastingPatterns;
import materia.assembly.AssemblyFrames as F;

/** Approved source declarations, independent of artifact export. Native kit
 * review still proves reconstruction, fixture clearance and both motion graphs. */
@:access(hobber.HobberFoundationSizing) class HobberSourceTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }

  public static function bodyIntent():Void {
    for (kind in ["bed", "column"]) {
      var values = HobberLayout.get().foundationBodyValues(kind), cuts = CastingPatterns.decode(values.token("features"));
      var envelope = materia.casting.CastingEnvelope.build(
        values.number("width"),
        values.number("depth"),
        values.number("height")
      );
      var cavities = CastingCavities.derive(envelope, cuts);
      check(cavities.length == cuts.length, "Every declared source cut derives exactly: " + kind);
      var byFeature = new Map<String, Array<materia.casting.Box3>>(), voids:Array<materia.casting.Box3> = [];
      for (cavity in cavities) {
        check(
          CastingCavities.supportsBoxes(cavity),
          "Hobber design cuts retain exact box topology: " + cavity.featureId
        );
        var boxes = CastingCavities.boxes(cavity);
        byFeature.set(cavity.featureId, boxes);
        voids = voids.concat(boxes);
      }
      for (group in BoxAlgebra.groups(BoxAlgebra.subtract(voids, []))) {
        var members = cuts.filter(feature -> {var boxes = byFeature.get(feature.id);
        return BoxAlgebra.volume(BoxAlgebra.subtract(boxes, group)) < BoxAlgebra.volume(boxes) - 1e-7;
        });
        check(
          members.filter(feature -> feature.openings.filter(opening -> opening.kind == BodyFace).length > 0).length > 0,
          "Every source cavity network has a declared body opening: " + [for (feature in members) feature.id].join(", ")
        );
      }
      if (kind == "column") {
        check(
          cuts.length == 3 && cuts.filter(feature -> feature.kind == ServiceOpening && StringTools.startsWith(
            feature.id,
            "head-recess-"
          )).length == 3,
          "The column is solid apart from its declared head recess"
        );
      } else {
        var drive = cuts.filter(feature -> feature.id == "pocket-driveRibRear-leftWebInside-floor")[0];
        check(
          drive.openings.filter(opening -> opening.kind == BodyFace && opening.target == "body.z-min").length == 1,
          "The drive chamber opens directly to the sole"
        );
        var duct = cuts.filter(feature -> feature.id == "cable-route")[0];
        check(
          duct.openings.length == 2 && duct.openings.filter(opening -> opening.kind == BodyFace).length == 2,
          "The cable duct has two real end-face openings"
        );
      }
    }
  }

  static function foundationIntent():Void {
    var selections = Sys.args().indexOf("--column-only") >= 0 ? [HobberMoldSetup.HobberMoldCasting.Column]
      : Sys.args().indexOf("--plinth-only") >= 0 ? [HobberMoldSetup.HobberMoldCasting.Plinth] : [
        HobberMoldSetup.HobberMoldCasting.Plinth,
        HobberMoldSetup.HobberMoldCasting.Column
      ];
    for (kind in selections) {
      var foundation:machinekit.structural.MineralFoundation = kind
        == Plinth ? new hobber.MineralCastBed() : new hobber.MineralCastColumn();
      var document = HobberMoldSetup.document(foundation, kind), input = document.input;
      var axis = kind == Plinth ? F.transformVector(document.setup.sourceToMold, 0, 0, 1) : F.transformVector(
        document.setup.sourceToMold,
        0,
        1,
        0
      );
      check(axis.z < -1 + 1e-8, "Declared mounting faces point toward the mold bed: " + kind);
      var lifting = input.inserts.filter(insert -> insert.role == materia.casting.InsertSpec.InsertRole.LiftingMount);
      var points = [for (point in input.liftingPoints) {
        var insert = input.inserts.filter(insert -> insert.id == point.insertId)[0];
        var hole = insert.holes.filter(hole -> hole.id == point.holeId)[0];
        var position = F.transformPoint(F.compose(insert.pose, hole.frame), 0, 0, 0);
        {
          id:point.id, x:position.x, y:position.y, z:position.z
        };
      }
      ];
      check(
        points.length == 4 && lifting.length == (kind == Plinth ? 4 : 2),
        "Four source lifting threads; the column shares two continuous plates"
      );
      for (point in points.filter(point -> point.id == "liftLeft" || point.id == "liftRight")) {
        var expectedX = kind == Plinth
          ? -150 - hobber.HobberLayout.get().xNominal().bedCentre : (point.id == "liftLeft" ? -160.0 : 160.0);
        var expectedY = kind == Plinth ? (point.id == "liftLeft" ? -310.0 : 310.0) : -40.0;
        check(
          Math.abs(point.x - expectedX) + Math.abs(point.y - expectedY) < 1e-7,
          "Original lifting service X/Y centre does not move: " + point.id
        );
      }
      var additional = points.filter(point -> StringTools.startsWith(point.id, "liftAdditional"));
      check(additional.length == 2, "The source retains exactly two additional lifting points");
      check(
        kind == Plinth ? Math.abs(additional[0].x - additional[1].x) < 1e-7
        && Math.abs(additional[0].y + additional[1].y) < 1e-7 : Math.abs(additional[0].x + additional[1].x) < 1e-7
        && Math.abs(additional[0].y - additional[1].y) < 1e-7,
        "The additional lifting pair remains symmetric"
      );
      var mass = machinekit.structural.FoundationSizing.asCastMass(foundation);
      var relative = F.compose(
        HobberMoldSetup.options(input).serviceOrientation,
        F.inverse(document.setup.sourceToMold)
      );
      var handling:materia.tooling.casting.CastingMoldPlan.HandlingPlan = {
        liftingPoints: input.liftingPoints,
        requiresFlip: F.transformVector(relative, 0, 0, 1).z < 1 - 1e-8,
        moldToService: relative,
        castingMassKg: mass.mass,
        centerOfMassMoldMm: F.transformPoint(
          document.setup.sourceToMold,
          mass.centreOfMass.x,
          mass.centreOfMass.y,
          mass.centreOfMass.z
        )
      };
      var liftingChecks = HobberMoldChecks.liftingReview(document, handling);
      check(
        liftingChecks.filter(row -> row.status
        == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail).length == 0,
        "Delivered lift review resolves each explicit thread, including multiple threads on one plate"
      );
      check(
        liftingChecks.filter(row -> row.rule == "tooling.handling.lift-polygon"
        && row.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass).length == 1,
        "Delivered lifting polygon uses all four source threads"
      );
      var xs = [for (point in points) point.x], ys = [for (point in points) point.y];
      xs.sort(Reflect.compare);
      ys.sort(Reflect.compare);
      var x0 = xs[0], x1 = xs[xs.length - 1], y0 = ys[0], y1 = ys[ys.length - 1], c = mass.centreOfMass;
      check(
        mass.unaccounted.length == 0 && c.x > x0 && c.x < x1 && c.y > y0 && c.y < y1,
        "Actual source CoM lies inside the four-thread lifting polygon"
      );
      var total = 0.0;
      for (point in points) {
        var wx = Math.abs(point.x - x0) < 1e-7 ? (x1 - c.x) / (x1 - x0) : (c.x - x0) / (x1 - x0);
        var wy = Math.abs(point.y - y0) < 1e-7 ? (y1 - c.y) / (y1 - y0) : (c.y - y0) / (y1 - y0);
        var force = mass.mass * 9.80665 * wx * wy;
        total += force;
        check(
          force > 0 && force <= mass.mass * 9.80665 / 2,
          "Nominal vertical point share is positive and below the two-load-path anchor demand: " + point.id
        );
      }
      check(
        Math.abs(total - mass.mass * 9.80665) < 1e-6,
        "Nominal vertical shares balance source weight; hardware and sling qualification remain Pending"
      );
      for (insert in lifting) {
        var count = input.liftingPoints.filter(point -> point.insertId == insert.id).length;
        var rules = switch insert.anchorage {
          case Bolted(rules, _):
            rules;
          default:
            throw "Missing lifting anchors";
        };
        var expected = mass.mass * 9.80665 * Math.min(2, count) / 2;
        check(
          Math.abs(rules.demand.liftingShareN - expected) < 1e-4,
          "A shared plate carries both possible active lifting paths, not only one point's share: " + insert.id
        );
        var size = materia.casting.InsertSpec.InsertBodyCodec.box(insert.body, insert.id);
        Sys.println("Source lifting " + kind + "/" + insert.id + " at "
          + insert.pose.x + ", " + insert.pose.y + "; blank " + size.width + " x " + size.depth + " x " + size.height);
      }
      var minimum = document.recipe.designReview.minimumLiftingPoints;
      check(
        minimum != null && materia.tooling.casting.CastHandling.liftingIds(document).length >= minimum,
        "Source lifting threads satisfy the unchanged shop point-count rule: " + kind
      );
      for (insert in input.inserts) {
        var stock = F.compose(document.setup.sourceToMold, insert.pose);
        var face = materia.tooling.casting.InsertFixturePlanner.mountingFace(
          insert,
          document.setup.sourceToMold,
          document.recipe.hardware
        );
        var holes = materia.tooling.casting.InsertFixturePlanner.availableHoles(insert);
        check(
          holes.filter(hole -> materia.tooling.casting.InsertFixturePlanner.select(
            hole,
            stock,
            face,
            document.recipe.hardware
          ) != null).length > 0,
          "Every source insert has its own usable fixing thread: " + kind + "/" + insert.id
        );
        switch insert.origin {
          case Fabricated(_):
            check(switch insert.anchorage {
              case Bolted(_, anchors):
                anchors.length >= 2;
              default:
                false;
            }, "Made inserts use the shared demand-sized bolted anchor rule: " + kind + "/" + insert.id);
          case Catalogue(_):
        }
      }
      if (kind == Column) {
        var rows = foundation.insertLocations(), site = rows.filter(row -> row.id == "liftRight")[0];
        var neighbours = new machinekit.tooling.SourceSteel(rows.filter(row -> row.id != site.id)),
          resin = new machinekit.structural.MineralCasting(HobberLayout.get().foundationBodyValues("column")).geometry();
        var cover = switch site.part.insertSpec().anchorage {
          case Bolted(rules, _):
            rules.minimumCoverMm;
          default:
            throw "Missing lifting anchors";
        };
        var bad:machinekit.structural.MineralFoundation.CastInsertPlacement = {
          id: site.id,
          purpose:site.purpose,
          part: site.part,
          openings: site.openings,
          pose: F.compose(site.pose, F.translation(0, 1, 0))
        };
        var message = "";
        try machinekit.tooling.InsertAnchorGeometry.checkSourceClearance(
          bad,
          neighbours,
          resin,
          HobberLayout.get().foundationDimensions("column"),
          cover
        ) catch (error:Dynamic) message = Std.string(error);
        try machinekit.tooling.InsertAnchorGeometry.checkSourceClearance(
          site,
          neighbours,
          resin,
          HobberLayout.get().foundationDimensions("column"),
          cover
        ) catch (error:Dynamic) {
          neighbours.close();
          resin.close();
          throw error;
        }
        neighbours.close();
        resin.close();
        check(
          message.indexOf("Lifting blank lacks casting cover to source anchor:") >= 0 && message.indexOf("rail") >= 0,
          "Moving a merged plate 1 mm toward the rails is rejected against their real anchor heads, not only the rail blanks"
        );
        check(
          lifting.length == 2 && points.length == 4,
          "The merged source keeps four independent threads on two clear plates"
        );
      }
      if (Sys.args().indexOf("--mold-check") >= 0) {
        var planned = machinekit.tooling.CastingMoldCad.plan(document, HobberMoldSetup.options(document.input));
        for (fixture in planned.fixtures) check(
          fixture.fixings.length > 0,
          "Every declared source fixture thread has a supported screw chase: " + fixture.insertId
        );
        var review = machinekit.tooling.CastingMoldCad.review(planned);
        HobberMoldChecks.run(planned, review);
        check(review.symmetricDifferenceMm3 < 1e-3, "Exact corrected hobber reconstruction: " + kind);
        for (rule in ["reconstruction", "coverage", "assembly-graph", "demolding-graph"]) check(
          review.validation.filter(row -> row.rule == "tooling.cad." + rule && row.status == "pass").length == 1,
          "Corrected hobber native " + rule + ": " + kind
        );
      }
    }
  }
  static function main():Void {
    bodyIntent();
    Sys.println("Hobber source cavity intent passed");
    foundationIntent();
    Sys.println('HobberSourceTests: $assertions assertions passed');
  }
}
