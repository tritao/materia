import machinekit.component.ComponentValues;
import machinekit.structural.CastInsert;
import machinekit.structural.MineralFoundation;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import machinekit.tooling.InsertAnchorGeometry;
import machinekit.structural.InsertHoles;
import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingPatterns;

/** Four source connections enclose the foundation's nominal centre of mass.
 * The attachment footprint is a design declaration, never a capacity approval. */
class MillLiftingPattern {

  public static function placements(
    body:ComponentValues,
    existing:Array<CastInsertPlacement>,
    details:MillCastingDetails,
    topFace: {width: Float, depth: Float},
    rule:materia.casting.InsertAnchorage.InsertAnchorPatternRule
  ):Array<CastInsertPlacement> {
    var fixed = machinekit.structural.FoundationSizing.anchors(existing, rule, body);
    var foundation = new MineralFoundation(body.copy().setToken("postCureBonds",
      "[]"), existing), mass = machinekit.structural.FoundationSizing.asCastMass(foundation);
    if (mass.mass <= 0 || !Math.isFinite(mass.mass)
      || mass.unaccounted.length > 0) throw "Lifting layout needs the source foundation mass and centre of mass";
    var servicePolicy = machinekit.design.ThreadedHolePolicy.resolve(details.liftingThread.thread, C45Bright);
    var minimumWidth = Math.ceil(details.liftingThread.thread.majorDiameter + 2 * servicePolicy.minWall);
    var footprints = [for (width in Std.int(minimumWidth)...Std.int(Math.floor(details.liftingPlateWidthMm)) + 1) {
      width:(width:Float), depth:details.liftingPlateDepthMm
    }
    ];
    if (footprints.length == 0) throw "Lifting stock cannot retain the policy thread wall";
    for (footprint in footprints) if (!Math.isFinite(footprint.width + footprint.depth)
      || footprint.width <= 0 || footprint.depth <= 0) throw "Invalid source lifting plate footprint";
    footprints.sort((a, b) -> {var area = Reflect.compare(a.width * a.depth, b.width * b.depth);
    return area != 0 ? area : Reflect.compare(a.width, b.width);
    });
    var share = mass.mass * rule.gravityMps2 / Math.min(rule.liftingLoadPaths, 4);
    if (details.liftingLandStepMm <= 0
      || details.liftingMaximumLandMm < details.liftingLandStepMm) throw "Invalid lifting land search rule";
    var seeds:Array<{x:Float, y:Float, turn:Int}> = [];
    var count = Std.int(Math.floor(details.liftingMaximumLandMm / details.liftingLandStepMm));
    for (ix in 1...count + 1) for (iy in 1...count + 1) for (turn in [0,
      1]) if (ix * details.liftingLandStepMm >= CastingShopRecipe.minimumSteelCoverMm()
      && iy * details.liftingLandStepMm >= CastingShopRecipe.minimumSteelCoverMm()) seeds.push({
        x: ix * details.liftingLandStepMm,
        y: iy * details.liftingLandStepMm,
        turn: turn
      });
    seeds.sort((a, b) -> {var sum = Reflect.compare(a.x + a.y, b.x + b.y);
    if (sum != 0) return sum;
    var x = Reflect.compare(a.x, b.x);
    return x != 0 ? x : Reflect.compare(a.turn, b.turn);
    });
    // The centred service drilling is identical at every site. Reject a
    // thread/blank that cannot carry demand before searching placements.
    var admissible:Array<{width:Float, depth:Float}> = [];
    for (footprint in footprints) {
      var trial = placement(
        "lifting-blank",
        body,
        details,
        footprint.width,
        footprint.depth,
        1,
        1,
        share,
        seeds[0],
        topFace
      );
      try {
        InsertAnchorGeometry.derive(trial.id, trial.part, rule, FootprintOnly, share);
        admissible.push(footprint);
      } catch (error:Dynamic) {
        if (Std.isOfType(error, machinekit.tooling.InsertAnchorGeometry.InsertAnchorScreenDefect)) throw error;
      }
    }
    footprints = admissible;
    if (footprints.length == 0) throw "No declared lifting blank meets source anchor demand";
    var resin = foundation.body.geometry(), result:Array<CastInsertPlacement> = [];
    var neighbours:machinekit.tooling.SourceSteel = null;
    try {
      neighbours = new machinekit.tooling.SourceSteel(fixed);
      for (sx in [-1, 1]) for (sy in [-1, 1]) {
        var id = "lift" + (sx < 0 ? "Left" : "Right") + (sy < 0 ? "Front" : "Rear");
        var failures:Array<String> = [], chosen:CastInsertPlacement = null;
        for (footprint in footprints) {
          for (seed in seeds) {
            try {
              var candidate = placement(
                id,
                body,
                details,
                footprint.width,
                footprint.depth,
                sx,
                sy,
                share,
                seed,
                topFace
              );
              InsertAnchorGeometry.checkSourceClearance(
                candidate,
                neighbours,
                resin,
                [body.number("width"), body.number("depth"), body.number("height")],
                CastingShopRecipe.minimumSteelSpacingMm()
              );
              // Use the source contract to reject a new side breakout as
              // well as volume outside the resin. Only the top is open.
              new MineralFoundation(body.copy().setToken("postCureBonds",
                "[]"), existing.concat([candidate])).body.finishedCavities();
              var blank = machinekit.structural.InsertBodies.part(candidate.part.insertSpec().body);
              var placed:cadkit.modeling.Part = null, outside:cadkit.modeling.Part = null;
              var missing = 0.0;
              try {
                placed = machinekit.assembly.PosedParts.moved(blank, candidate.pose);
                outside = placed.subtract(resin);
                missing = outside.volume();
              } catch (error:Dynamic) {
                if (outside != null) outside.close();
                if (placed != null) placed.close();
                blank.close();
                throw error;
              }
              outside.close();
              placed.close();
              blank.close();
              if (missing
                > 1e-6) throw "blank intersects an existing cavity or leaves the casting: " + missing + " mm3";
              var steel = new machinekit.assembly.MachineAssembly();
              steel.addComponent(id, candidate.part, F.identity());
              var checked = machinekit.design.AssemblyDesignChecks.check(steel);
              if (checked.hasErrors()) throw checked.items.map(item -> item.message).join("; ");
              var anchored = InsertAnchorGeometry.derive(
                id,
                candidate.part,
                rule,
                Casting(resin, candidate.pose),
                share
              );
              neighbours.add({id: candidate.id, part: anchored, pose: candidate.pose, openings: candidate.openings});
              chosen = candidate;
              break;
            } catch (error:Dynamic) {
              if (Std.isOfType(error, machinekit.tooling.InsertAnchorGeometry.InsertAnchorScreenDefect)) throw error;
              failures.push(footprint.width
                + "x" + footprint.depth + " @ " + seed.x + "," + seed.y + ": " + Std.string(error));
            }
          }
          if (chosen != null) break;
        }
        if (chosen == null) throw "No source lifting footprint fits " + id + ": " + failures.join("; ");
        var com = mass.centreOfMass;
        if (Math.abs(com.x) >= Math.abs(chosen.pose.x) || Math.abs(com.y)
          >= Math.abs(chosen.pose.y)) throw "Source lifting footprint cannot enclose the foundation centre of mass";
        result.push(chosen);
      }
    } catch (error:Dynamic) {
      if (neighbours != null) neighbours.close();
      resin.close();
      throw error;
    }
    neighbours.close();
    resin.close();
    return result;
  }

  static function placement(
    id:String,
    body:ComponentValues,
    details:MillCastingDetails,
    width:Float,
    depth:Float,
    sx:Int,
    sy:Int,
    share:Float,
    seed: {x: Float, y: Float, turn: Int},
    topFace: {width: Float, depth: Float}
  ):CastInsertPlacement {
    var height = details.liftingPlateThicknessMm;
    if (!Math.isFinite(height) || height <= 0) throw "Invalid source lifting plate thickness";
    var land = seed;
    var holes = InsertHoles.records(InsertHoles.tappedBlind(details.liftingThread, height, C45Bright));
    holes[0].frame.x = 0;
    holes[0].frame.y = 0;
    var wall = details.liftingThread.thread.majorDiameter / 2 + holes[0].limits.minWall;
    if (Math.abs(holes[0].frame.x) + wall > width / 2 + 1e-7 || Math.abs(holes[0].frame.y) + wall
      > depth / 2 + 1e-7) throw "Lifting source blank cannot retain the service thread's policy wall";
    var part = new CastInsert(CastInsert.precastHoles(
      CastInsert.boxValues(width, depth, height),
      InsertHoles.fromNeutral(holes)
    ).setToken(
      "role",
      materia.casting.InsertSpec.InsertRole.LiftingMount
    ).setToken(
      "label",
      "foundation lifting connection"
    ).setToken(
      "partId",
      "MILL-LIFT-ONE"
    ).setToken(
      "finish",
      materia.casting.InsertSpec.InsertFinish.AsSupplied
    ));
    var footprintWidth = seed.turn == 0 ? width : depth, footprintDepth = seed.turn == 0 ? depth : width;
    var x = topFace.width / 2 - footprintWidth / 2 - land.x, y = topFace.depth / 2 - footprintDepth / 2 - land.y;
    var z = body.number("height") - height;
    if (x <= 0 || y <= 0 || z < 0) throw "Source lifting blank exceeds the foundation envelope";
    return {
      id: id,
      part: part,
      pose: seed.turn == 0 ? F.translation(sx * x, sy * y, z) : F.fromRotationMatrix(
        sx * x,
        sy * y,
        z,
        [0, -1, 0, 1, 0, 0, 0, 0, 1]
      ),
      openings : [CastingPatterns.face("body.z-max")]
    };
  }
}
