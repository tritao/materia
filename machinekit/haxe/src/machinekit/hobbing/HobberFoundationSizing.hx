package machinekit.hobbing;

import machinekit.component.ComponentValues;
import machinekit.structural.CastInsert;
import machinekit.structural.FoundationSizing;
import machinekit.structural.InsertHoles;
import machinekit.structural.MineralFoundation;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertAnchorage.InsertAnchorSelection;
import materia.casting.InsertSpec.InsertRole;

typedef HobberSizedFoundation = {
  var body:ComponentValues;
  var placements:Array<CastInsertPlacement>;
  var liftingPoints:Array<materia.casting.CastingLiftingPoint>;
}

/** Resolve source dimensions before realizing the final foundation. Hole limits
 * and demand are design rules; compound capacity remains a workshop input. */
class HobberFoundationSizing {
  static inline final LIFT_THREAD_MM = 12;
  static inline final LIFT_MAXIMUM_WIDTH_MM = 40;
  static inline final LIFT_DEPTH_MM = 100;
  static inline final LIFT_THICKNESS_MM = 30;
  static inline final LIFT_POINT_COUNT = 4;
  static inline final COLUMN_LIFT_PROPOSED_Y_MM = 60;

  public static function resolve(kind:String):HobberSizedFoundation {
    var body = FoundationLayout.bodyValues(kind);
    var rows = HobberMountingHoles.derive(kind, FoundationLayout.placements(kind));
    var rule = InsertAnchorSelection.sourceDemandRule("hobber-" + kind + "-source-anchors");
    var failures:Array<String> = [];
    for (size in rule.allowedNominalDiametersMm) {
      var single = FoundationSizing.singleThreadRule(rule, size);
      single.coverDiameters = Math.max(single.coverDiameters, FoundationLayout.SOURCE_STEEL_COVER_MM / size);
      try {
        var placed = kind == "column" ? columnLifting(body, rows, single) : plinthLifting(body, rows, single);
        // Catalogue sockets retain their retention while their real mass
        // remains in the complete lifting-demand assembly.
        var points:Array<materia.casting.CastingLiftingPoint> = kind == "column" ? [
          {id : "liftLeft", insertId : "liftLeft", holeId : "lifting-primary"},
          {id : "liftRight", insertId : "liftRight", holeId : "lifting-primary"},
          {id : "liftAdditional1", insertId : "liftRight", holeId : "lifting-secondary"},
          {id : "liftAdditional2", insertId : "liftLeft", holeId : "lifting-secondary"}
        ] : [for (id in [
          "liftLeft",
          "liftRight",
          "liftAdditional1",
          "liftAdditional2"
        ]) {
          id:id, insertId:id, holeId:"fixing"
        }
        ];
        return {
          body: body,
          placements: FoundationSizing.anchors(placed, single, body, false, points),
          liftingPoints: points
        };
      } catch (error:Dynamic) {
        if (Std.isOfType(error, machinekit.tooling.InsertAnchorGeometry.InsertAnchorScreenDefect)) throw error;
        failures.push(size + " mm: " + Std.string(error));
      }
    }
    throw "No common source anchor thread fits " + kind + ": " + failures.join("; ");
  }

  static function blank(width:Float, depth:Float):CastInsert {
    var choice = InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(LIFT_THREAD_MM).mating());
    return new CastInsert(CastInsert.precastHoles(
      CastInsert.boxValues(width, depth, LIFT_THICKNESS_MM),
      InsertHoles.tappedBlind(choice, LIFT_THICKNESS_MM)
    ).setToken(
      "role",
      LiftingMount
    ).setToken(
      "label",
      "foundation lifting connection"
    ).setToken(
      "partId",
      "HOBBER-LIFT-ONE"
    ).setToken(
      "finish",
      materia.casting.InsertSpec.InsertFinish.AsSupplied
    ));
  }

  /** A continuous plate occupies each clear side land. Its two service threads
	 * are declared separately; a fixture or rear anchor is never a lift point. */
  static function columnLifting(
    body:ComponentValues,
    rows:Array<CastInsertPlacement>,
    rule:materia.casting.InsertAnchorage.InsertAnchorPatternRule
  ):Array<CastInsertPlacement> {
    var d = FoundationLayout.dimensions("column"), cover = FoundationLayout.SOURCE_STEEL_COVER_MM;
    var choice = InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(LIFT_THREAD_MM).mating());
    var policy = machinekit.design.ThreadedHolePolicy.resolve(choice.thread, C45Bright);
    var x0 = FoundationLayout.COLUMN_HEAD_WIDTH / 2 + cover, x1 = d[0] / 2 - cover;
    var fixed = FoundationSizing.anchors(
      rows.filter(row -> row.part.parameterToken("role") != LiftingMount),
      rule,
      body
    );
    var rails = fixed.filter(row -> row.part.parameterToken("role") == GuideSeat);
    if (rails.length != 2) throw "Column lifting lands require both declared rail seats";
    var topY = Math.POSITIVE_INFINITY;
    for (rail in rails) {
      var box = materia.casting.CastingRegionBounds.box(Placed({solid: Insert(rail.part.bodySpec()), pose: rail.pose}));
      if (box == null) throw "Column rail seat requires exact box geometry: " + rail.id;
      topY = Math.min(topY, box.y - cover);
      // The adjacent rail's rear flange heads also bound the land. Use
      // their placed solids, only where they overlap the lifting plate height.
      var scope = new cadkit.modeling.Scope();
      try {
        for (hardware in machinekit.tooling.InsertAnchorGeometry.components(rail.part.insertSpec())) {
          var local = scope.own(hardware.part.geometry());
          local.shape.setThreadedFaces([]);
          var solid = scope.own(machinekit.assembly.PosedParts.moved(local, F.compose(rail.pose, hardware.pose)));
          var b = machinekit.assembly.PosedParts.boxOf(solid);
          if (b.maxZ > d[2] - LIFT_THICKNESS_MM && b.minZ < d[2]) topY = Math.min(topY, b.minY - cover);
        }
      } catch (error:Dynamic) {
        scope.close();
        throw error;
      }
      scope.close();
    }
    var bottomY = -d[1] / 2 + cover, centreY = (topY + bottomY) / 2;
    var width = x1 - x0, depth = topY - bottomY;
    var secondaryY = Math.min(COLUMN_LIFT_PROPOSED_Y_MM, topY - policy.minWall - choice.thread.majorDiameter / 2);
    var result = rows.filter(row -> row.part.parameterToken("role") != LiftingMount);
    for (original in rows.filter(row -> row.part.parameterToken("role") == LiftingMount)) {
      var sx = original.pose.x < 0 ? -1 : 1, x = sx * (x0 + x1) / 2;
      if (Math.abs(x - original.pose.x) > 1e-7) throw "Merged lifting land would move the original X: " + original.id;
      var holes = [];
      for (site in [{id: "lifting-primary", y: original.pose.y}, {id: "lifting-secondary", y: secondaryY}]) {
        var hole = InsertHoles.records(InsertHoles.tappedBlind(choice, LIFT_THICKNESS_MM))[0];
        hole.id = site.id;
        hole.frame.y = site.y - centreY;
        holes.push(hole);
      }
      var part = new CastInsert(CastInsert.precastHoles(
        CastInsert.boxValues(width, depth, LIFT_THICKNESS_MM),
        InsertHoles.fromNeutral(holes)
      ).setToken(
        "role",
        LiftingMount
      ).setToken(
        "label",
        "two-point column lifting plate"
      ).setToken(
        "partId",
        "HOBBER-COLUMN-LIFT-TWO"
      ).setToken(
        "finish",
        materia.casting.InsertSpec.InsertFinish.AsSupplied
      ));
      result.push({
        id: original.id,
        part: part,
        pose: F.translation(x, centreY, d[2] - LIFT_THICKNESS_MM),
        openings: [materia.casting.CastingPatterns.face("body.z-max")]
      });
    }
    var resin = new machinekit.structural.MineralCasting(body).geometry();
    try {
      for (candidate in result.filter(row -> row.part.parameterToken("role")
        == LiftingMount)) machinekit.tooling.InsertAnchorGeometry.checkSourceClearance(
          candidate,
          fixed,
          resin,
          d,
          cover
        );
    } catch (error:Dynamic) {
      resin.close();
      throw error;
    }
    resin.close();
    return result;
  }

  static function plinthLifting(
    body:ComponentValues,
    rows:Array<CastInsertPlacement>,
    rule:materia.casting.InsertAnchorage.InsertAnchorPatternRule
  ):Array<CastInsertPlacement> {
    var choice = InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(LIFT_THREAD_MM).mating());
    var policy = machinekit.design.ThreadedHolePolicy.resolve(choice.thread, C45Bright);
    var minimumWidth = Std.int(Math.ceil(choice.thread.majorDiameter + 2 * policy.minWall));
    var existing = rows.filter(row -> row.part.parameterToken("role") == LiftingMount);
    var result = rows.filter(row -> row.part.parameterToken("role") != LiftingMount);
    var sites:Array<CastInsertPlacement> = [for (row in existing) {
      id:row.id, part:blank(LIFT_MAXIMUM_WIDTH_MM, LIFT_DEPTH_MM), openings:row.openings, pose:F.compose(
        row.pose,
        F.translation(0, 0, row.part.boxDimensions(row.id).height - LIFT_THICKNESS_MM)
      )
    }
    ];
    var mass = FoundationSizing.asCastMass(new MineralFoundation(body, result.concat(sites)));
    if (mass.mass <= 0 || !Math.isFinite(mass.mass)
      || mass.unaccounted.length > 0) throw "Hobber lifting layout requires known source mass: plinth";
    var d = FoundationLayout.dimensions("bed");
    var proposed = InsertAnchorSelection.liftingPositions(
      {x: -d[0] / 2, y: -d[1] / 2, z: 0, width: d[0], depth: d[1], height: d[2]},
      {x: mass.centreOfMass.x, y: mass.centreOfMass.y, z: mass.centreOfMass.z},
      LIFT_POINT_COUNT
    );
    var centreX = 0.0, centreY = 0.0;
    for (site in sites) {
      centreX += site.pose.x;
      centreY += site.pose.y;
    }
    centreX /= sites.length;
    centreY /= sites.length;
    proposed.sort((a, b) -> {var da = (a.x - centreX) * (a.x - centreX) + (a.y - centreY) * (a.y - centreY);
    var db = (b.x - centreX) * (b.x - centreX) + (b.y - centreY) * (b.y - centreY);
    var distance = Reflect.compare(db, da);
    return distance == 0 ? Reflect.compare(a.y, b.y) : distance;
    });
    var missing = LIFT_POINT_COUNT - sites.length;
    if (missing < 0 || proposed.length < missing) throw "Unexpected source lifting count: plinth";
    for (index in 0...missing) {
      var point = proposed[index];
      point.y = (point.y < 0 ? -1 : 1) * FoundationLayout.LIFT_CENTRE_Y;
      Sys.println("Hobber source lifting proposal bed: " + point.x + ", " + point.y + ", " + point.z);
      sites.push({
        id: "liftAdditional" + (index + 1),
        part: blank(LIFT_MAXIMUM_WIDTH_MM, LIFT_DEPTH_MM),
        pose: F.translation(point.x, point.y, point.z - LIFT_THICKNESS_MM),
        openings: [materia.casting.CastingPatterns.face("body.z-max")]
      });
    }
    var resin = new machinekit.structural.MineralCasting(body).geometry();
    try {
      var neighbours = FoundationSizing.anchors(result, rule, body);
      for (site in sites) {
        var chosen:CastInsertPlacement = null, chosenHardware:CastInsertPlacement = null, failures:Array<String> = [];
        var stock = site.part.boxDimensions(site.id);
        for (width in minimumWidth...Std.int(stock.width) + 1) {
          for (turn in [0, 1]) {
            var pose = turn == 0 ? site.pose : F.compose(
              site.pose,
              F.fromRotationMatrix(0, 0, 0, [0.0, -1, 0, 1, 0, 0, 0, 0, 1])
            );
            var candidate:CastInsertPlacement = {
              id: site.id,
              part: blank(width, stock.depth),
              pose: pose,
              openings: site.openings
            };
            try {
              machinekit.tooling.InsertAnchorGeometry.checkSourceClearance(
                candidate,
                neighbours,
                resin,
                d,
                FoundationLayout.SOURCE_STEEL_COVER_MM
              );
              var anchored = machinekit.tooling.InsertAnchorGeometry.derive(
                candidate.id,
                candidate.part,
                rule,
                Casting(resin, pose),
                mass.mass * rule.gravityMps2 / Math.min(rule.liftingLoadPaths, LIFT_POINT_COUNT)
              );
              chosen = candidate;
              chosenHardware = {id: site.id, part: anchored, pose: pose, openings: site.openings};
              break;
            } catch (error:Dynamic) {
              if (Std.isOfType(error, machinekit.tooling.InsertAnchorGeometry.InsertAnchorScreenDefect)) throw error;
              failures.push(width + " x " + stock.depth + " mm, turn " + turn + ": " + Std.string(error));
            }
          }
          if (chosen != null) break;
        }
        if (chosen == null) throw "No source lifting footprint fits plinth/" + site.id + ": " + failures.join("; ");
        result.push(chosen);
        neighbours.push(chosenHardware);
      }
    } catch (error:Dynamic) {
      resin.close();
      throw error;
    }
    resin.close();
    return result;
  }
}
