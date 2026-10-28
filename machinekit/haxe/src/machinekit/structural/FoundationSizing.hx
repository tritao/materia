package machinekit.structural;

import machinekit.component.ComponentValues;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import materia.casting.InsertSpec.InsertRole;

/** Structural sizing before source realization. The result contains declared
 * insert footprints, holes and anchors; no workshop capacity is assumed. */
class FoundationSizing {
  /** Native nominal mass in the pour state: transferred holes and finishing
	 * removal do not yet exist. Embedded anchor hardware remains real components. */
  public static function asCastMass(foundation:MineralFoundation):machinekit
    .assembly.MachineAssembly.MachineAssemblyMassProperties {
    var raw = new machinekit.assembly.MachineAssembly(), poses = foundation.solvedPoses();
    var members = foundation.castingMemberIds();
    for (member in foundation.components()) {
      if (members.indexOf(member.id) < 0) continue;
      var part = member.component;
      if (Std.isOfType(part, MineralCasting)) {
        part = new MineralCasting(part.values().copy().setToken(
          "buildStage",
          materia.casting.InsertStages.InsertStage.BeforeCasting
        ));
      } else if (Std.isOfType(part, CastInsert)) {
        var steel:CastInsert = cast part;
        part = new CastInsert(steel.values().copy().setToken(
          "buildStage",
          materia.casting.InsertStages.InsertStage.BeforeCasting
        ));
      }
      raw.addComponent(member.id, part, poses.get(member.id));
    }
    return raw.massProperties();
  }

  public static function singleThreadRule(rule:materia.casting.InsertAnchorage.InsertAnchorPatternRule,
    size:Float):materia.casting.InsertAnchorage.InsertAnchorPatternRule return {
      id: rule.id,
      demandFactor: rule.demandFactor,
      maximumCount: rule.maximumCount,
      liftingLoadPaths: rule.liftingLoadPaths,
      gravityMps2: rule.gravityMps2,
      minimumCount: rule.minimumCount,
      maximumPitchInShortWidths: rule.maximumPitchInShortWidths,
      embedmentDiameters: rule.embedmentDiameters,
      coverDiameters: rule.coverDiameters,
      spacingDiameters: rule.spacingDiameters,
      screwLengthsMm: rule.screwLengthsMm.copy(),
      allowedNominalDiametersMm: [size]
    };

  /** Minimum declared stock width admitted by the same demand/steel selector.
	 * The final placement still needs casting cover and whole-body confirmation. */
  public static function anchorPadWidth(
    id:String,
    prototype:CastInsert,
    minimum:Float,
    depth:Float,
    height:Float,
    maximum:Float,
    increment:Float,
    rule:materia.casting.InsertAnchorage.InsertAnchorPatternRule
  ):Float {
    if (increment <= 0 || maximum < minimum) throw "No source pad width remains inside the casting: " + id;
    var count = Std.int(Math.floor((maximum - minimum) / increment));
    var last = "";
    for (index in 0...count + 1) {
      var width = minimum + index * increment;
      var part = new CastInsert(prototype.values().copy().setToken(
        "body",
        materia.casting.InsertSpec.InsertBodyCodec.encode(Primitive(Box(width, depth, height)))
      ));
      try {
        machinekit.tooling.InsertAnchorGeometry.derive(id, part, rule, FootprintOnly, 0);
        return width;
      } catch (error:Dynamic) {
        if (Std.isOfType(error, machinekit.tooling.InsertAnchorGeometry.InsertAnchorScreenDefect)) throw error;
        last = Std.string(error);
      }
    }
    throw "No demand-sized source pad fits " + id + ": " + last;
  }

  public static function anchors(
    rows:Array<CastInsertPlacement>,
    rule:materia.casting.InsertAnchorage.InsertAnchorPatternRule,
    castingValues:ComponentValues,
    commonSize:Bool = false
  ):Array<CastInsertPlacement> {
    if (commonSize) {
      var sizes = rule.allowedNominalDiametersMm.copy(), failures:Array<String> = [];
      sizes.sort(Reflect.compare);
      for (size in sizes) {
        var single = singleThreadRule(rule, size);
        try return anchors(rows, single, castingValues) catch (error:Dynamic) {
          if (Std.isOfType(error, machinekit.tooling.InsertAnchorGeometry.InsertAnchorScreenDefect)) throw error;
          failures.push(Std.string(size) + " mm: " + Std.string(error));
        }
      }
      throw "No shared declared anchor thread fits the casting: " + failures.join("; ");
    }
    var body = new machinekit.structural.MineralCasting(castingValues).geometry();
    try {
      var lifting = rows.filter(row -> row.part.parameterToken("role")
        == materia.casting.InsertSpec.InsertRole.LiftingMount);
      function share(placed:Array<CastInsertPlacement>):Float {
        if (lifting.length == 0) return 0;
        var mass = asCastMass(new MineralFoundation(castingValues.copy().setToken("postCureBonds", "[]"), placed));
        if (mass.unaccounted.length > 0 || !Math.isFinite(mass.mass)
          || mass.mass <= 0) throw "Anchor lifting demand requires the actual source casting mass";
        return mass.mass * rule.gravityMps2 / Math.min(rule.liftingLoadPaths, lifting.length);
      }
      var liftingShare = share(rows);
      // Only lifting demand depends on the resolved casting mass. Other
      // patterns and their exact containment are unchanged across this loop.
      var fixed:Map<String, CastInsert> = [];
      for (row in rows) if (row.part.parameterToken("role") != InsertRole.LiftingMount) {
        var part = switch row.part.insertSpec().origin {
          // Published retention stays unchanged, while its real mass and
          // displacement remain in the complete lifting-demand assembly.
          case Catalogue(_):
            row.part;
          case Fabricated(_):
            machinekit.tooling.InsertAnchorGeometry.derive(row.id, row.part, rule, Casting(body, row.pose), 0);
        };
        fixed.set(row.id, part);
      }
      // Hardware changes the casting mass. Resolve that known design coupling
      // without borrowing workshop strength values or dropping hardware mass.
      for (pass in 0...8) {
        var result:Array<CastInsertPlacement> = [for (row in rows) {
          id:row.id, pose:row.pose, openings:row.openings,
            part:fixed.exists(row.id) ? fixed.get(row.id) : machinekit.tooling.InsertAnchorGeometry.derive(
              row.id,
              row.part,
              rule,
              Casting(body, row.pose),
              liftingShare
            )
        }
        ];
        var resolved = share(result);
        if (Math.abs(resolved - liftingShare) < 1e-6) {
          body.close();
          return result;
        }
        liftingShare = resolved;
      }
      throw "Anchor/lifting mass sizing did not converge for " + [for (row in lifting) row.id].join(", ");
    } catch (error:Dynamic) {
      body.close();
      throw error;
    }
  }

}
