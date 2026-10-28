package materia.tooling.casting;

import materia.casting.CastingFeature;
import materia.casting.CastingCavities;
import materia.casting.CastingFeatureCodec;
import materia.casting.CastingRecordCodec as R;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingDocument;
import materia.tooling.casting.CastingSetup;

enum abstract StrategyOpening(String) to String {
  var BodyOpening = "body-opening";
  var Enclosed = "enclosed";
}

enum abstract WithdrawalAxis(String) to String {
  var X = "x";
  var Y = "y";
  var Z = "z";
}

class WithdrawalAxes {
  public static function values():Array<WithdrawalAxis> return [WithdrawalAxis.X, WithdrawalAxis.Y, WithdrawalAxis.Z];
  public static function tokens():Array<String> return [for (axis in values())(axis:String)];
}

class StrategyOpenings {
  public static function values():Array<StrategyOpening> return [StrategyOpening.BodyOpening, StrategyOpening.Enclosed];
  public static function tokens():Array<String> return [for (opening in values())(opening:String)];
}

typedef CastingStrategyRule = {
  var kind:CastingFeatureKind;
  var opening:StrategyOpening;
  var strategy:CoreStrategy;
  var priority:Int;
  var preferredAxis:WithdrawalAxis;
  /** Deep face-open pockets use EPS when no declared board can hold the source cut. */
  var useEPSAboveBoardThickness:Bool;
}

/** Directions are expressed in mold coordinates, never source coordinates. */
typedef ResolvedFeatureStrategy = {
  var featureId:String;
  var strategy:CoreStrategy;

  /** Higher priority owns overlap first; IDs resolve equal priorities. */
  var priority:Int;

  /** Required for reusable formers. EPS may use null for breakup removal. */
  var withdrawal:Null<{
    var x:Float;
    var y:Float;
    var z:Float;
  }>;
}

/** Source intent and recipe rules choose tooling; source axes select withdrawal. */
class CastingStrategies {
  public static function standardRules():Array<CastingStrategyRule> {
    var rules:Array<CastingStrategyRule> = [];
    var kinds:Array<CastingFeatureKind> = [Lightening, ServiceOpening, InterfaceRecess, Clearance];
    var openings:Array<StrategyOpening> = [BodyOpening, Enclosed];
    for (kind in kinds) {
      for (opening in openings) rules.push({
        kind: kind,
        opening: opening,
        strategy: kind == Lightening || opening == Enclosed ? SacrificialEPS : ReusableFormer,
        priority : 0,
        preferredAxis : kind == ServiceOpening ? X : Z,
        useEPSAboveBoardThickness : false
      });
    }
    return rules;
  }

  public static function snapshotRules(raw:Dynamic):Array<CastingStrategyRule> {
    if (!Std.isOfType(raw, Array)) throw "Invalid casting strategy rules";
    var rows:Array<Dynamic> = cast raw, result:Array<CastingStrategyRule> = [], ids = new Map<String, Bool>();
    for (row in rows) {
      CastingFeatureCodec.fields(
        row,
        ["kind", "opening", "strategy", "priority", "preferredAxis", "useEPSAboveBoardThickness"]
      );
      var limit:Dynamic = Reflect.field(row, "useEPSAboveBoardThickness");
      if (!Std.isOfType(limit, Bool)) throw "Invalid casting strategy board limit";
      var kind = R.string(row, "kind"), opening = R.string(row, "opening"), strategy = R.string(row, "strategy");
      var axis = R.string(row, "preferredAxis"), priority = R.number(row, "priority");
      if ((CastingFeatureKinds.tokens().indexOf(kind) < 0
        || kind == (CastingFeatureKind.InsertRecess:String))) throw "Unknown casting strategy feature kind: " + kind;
      if (StrategyOpenings.tokens().indexOf(opening) < 0) throw "Invalid casting strategy opening condition";
      if (materia.tooling.casting.CastingSetup.CoreStrategies.tokens().indexOf(strategy)
        < 0) throw "Invalid casting rule strategy";
      if (WithdrawalAxes.tokens().indexOf(axis) < 0 || priority != Math.floor(priority)
        || Math.abs(priority) > 1000000) throw "Invalid casting strategy axis or priority";
      var id = kind + "/" + opening;
      if (ids.exists(id)) throw "Duplicate casting strategy rule: " + id;
      ids.set(id, true);
      result.push({
        kind: cast kind,
        opening: cast opening,
        strategy: cast strategy,
        priority: Std.int(priority),
        preferredAxis: cast axis,
        useEPSAboveBoardThickness: cast limit
      });
    }
    result.sort((a, b) -> Reflect.compare((a.kind:String) + "/" + a.opening, (b.kind:String) + "/" + b.opening));
    return result;
  }

  /** Stock height follows an actual declared opening, not the thinnest box
	 * dimension. All clipping stays in CastingCavities. Unavailable non-box
	 * queries retain the strategy for the existing Unsupported geometry gate. */
  static function withdrawalDepth(
    document:ToolingDocument,
    feature:CastingFeature,
    normal: {x: Float, y: Float, z: Float}
  ):Null<Float> {
    var cavity = CastingCavities.derive(document.input.envelope,
      document.input.features).filter(cavity -> cavity.featureId == feature.id)[0];
    return switch cavity.geometry {
      case Prisms(pieces):
        var low = Math.POSITIVE_INFINITY, high = Math.NEGATIVE_INFINITY;
        for (piece in pieces) for (point in piece.profile) for (z in [0.0, piece.height]) {
          var p = F.transformPoint(piece.pose,
            point.x, point.y, z), projection = p.x * normal.x + p.y * normal.y + p.z * normal.z;
          low = Math.min(low, projection);
          high = Math.max(high, projection);
        }
        high - low;
      case SolidIntersection(_, _, _):
        if (CastingMoldPlanner.geometry == null) null;
        else {
          var local = MoldRegions.moved(CastingCavities.region(cavity), F.inverse(F.sectionFrame(normal, feature.id)));
          MoldRegions.exactBounds(local, feature.id).height;
        }
      case UnsupportedGeometry(_):
        null;
    };
  }

  public static function resolve(document:ToolingDocument):Array<ResolvedFeatureStrategy> {
    var result:Array<ResolvedFeatureStrategy> = [];
    for (feature in document.input.features) {
      if (feature.kind == InsertRecess) continue;
      var openings = feature.openings.filter(o -> o.kind == BodyFace);
      var opening:StrategyOpening = openings.length == 0 ? Enclosed : BodyOpening;
      var rules = document.recipe.featureRules.filter(rule -> rule.kind == feature.kind && rule.opening == opening);
      if (rules.length != 1) throw "Missing or duplicate recipe strategy rule for feature: " + feature.id;
      var rule = rules[0], strategy = rule.strategy;
      var direction:Null<{x:Float, y:Float, z:Float}> = null;
      if (rule.strategy == ReusableFormer || strategy == SacrificialEPS && opening == BodyOpening) {
        if (openings.length == 0) throw "Reusable former rule requires a body opening: " + feature.id;
        var normals = [for (o in openings) {
          id:o.target, normal:CastingCavities.bodyFaceNormal(document.input.envelope, o.target)
        }
        ];
        function coordinate(v: {x: Float,
          y: Float, z: Float}):Float return rule.preferredAxis == X ? v.x : rule.preferredAxis == Y ? v.y : v.z;
        var best = 0.0;
        for (candidate in normals) best = Math.max(best, Math.abs(coordinate(candidate.normal)));
        if (strategy == ReusableFormer && rule.useEPSAboveBoardThickness) {
          var thickness = 0.0;
          for (stock in document.recipe.former.stocks) thickness = Math.max(thickness, stock.measuredThickness);
          var allDeep = true;
          for (candidate in normals) if (Math.abs(Math.abs(coordinate(candidate.normal)) - best) <= 1e-9) {
            var depth = withdrawalDepth(document, feature, candidate.normal);
            if (depth == null || depth <= thickness + 1e-7) allDeep = false;
          }
          if (allDeep) strategy = SacrificialEPS;
        }
        // Sacrificial stock can share the open pour face; it is broken
        // out after lifting rather than claiming a rigid side withdrawal.
        var fillPortals = strategy == SacrificialEPS ? normals.filter(candidate -> {var v = candidate.normal;
        return F.transformVector(document.setup.sourceToMold, v.x, v.y, v.z).z > 1 - 1e-9;
        }):[];
        var candidates = fillPortals.length > 0
          ? fillPortals : normals.filter(candidate -> Math.abs(Math.abs(coordinate(candidate.normal)) - best) <= 1e-9);
        // A source-authored axis selects the intended end of a through
        // passage; IDs never choose between opposed openings.
        if (candidates.length > 1) {
          var axis = feature.draft.axis;
          var nominated = candidates.filter(candidate -> {var n = candidate.normal;
          return n.x * axis.x + n.y * axis.y + n.z * axis.z > 1 - 1e-9;
          });
          if (nominated.length == 1) candidates = nominated;
        }
        var normal = candidates[0].normal;
        for (candidate in candidates) {
          var v = candidate.normal;
          if (Math.abs(v.x - normal.x) + Math.abs(v.y - normal.y) + Math.abs(v.z - normal.z)
            > 1e-9) throw "Ambiguous feature withdrawal; declare the source withdrawal axis: " + feature.id;
        }
        direction = F.transformVector(document.setup.sourceToMold, normal.x, normal.y, normal.z);
      }
      result.push({featureId: feature.id, strategy: strategy, priority: rule.priority, withdrawal: direction});
    }
    result.sort((a, b) -> Reflect.compare(a.featureId, b.featureId));
    return result;
  }
}
