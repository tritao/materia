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
	public static function tokens():Array<String> return [for (axis in values()) (axis:String)];
}

class StrategyOpenings {
	public static function values():Array<StrategyOpening> return [StrategyOpening.BodyOpening, StrategyOpening.Enclosed];
	public static function tokens():Array<String> return [for (opening in values()) (opening:String)];
}

typedef CastingStrategyRule = {
	var kind:CastingFeatureKind;
	var opening:StrategyOpening;
	var strategy:CoreStrategy;
	var priority:Int;
	var preferredAxis:WithdrawalAxis;
}

/** Source intent and recipe rules choose defaults; setups name only explicit overrides. */
class CastingStrategies {
	public static function standardRules():Array<CastingStrategyRule> {
		var rules:Array<CastingStrategyRule> = [];
		var kinds:Array<CastingFeatureKind> = [Lightening, ServiceOpening, InterfaceRecess, Clearance];
		var openings:Array<StrategyOpening> = [BodyOpening, Enclosed];
		for (kind in kinds) {
			for (opening in openings)
				rules.push({
					kind: kind,
					opening: opening,
					strategy: kind == Lightening || opening == Enclosed ? SacrificialEPS : ReusableFormer,
					priority: 0,
					preferredAxis: kind == ServiceOpening ? X : Z
				});
		}
		return rules;
	}

	public static function snapshotRules(raw:Dynamic):Array<CastingStrategyRule> {
		if (!Std.isOfType(raw, Array))
			throw "Invalid casting strategy rules";
		var rows:Array<Dynamic> = cast raw, result:Array<CastingStrategyRule> = [], ids = new Map<String, Bool>();
		for (row in rows) {
			CastingFeatureCodec.fields(row, ["kind", "opening", "strategy", "priority", "preferredAxis"]);
			var kind = R.string(row, "kind"), opening = R.string(row, "opening"), strategy = R.string(row, "strategy");
			var axis = R.string(row, "preferredAxis"), priority = R.number(row, "priority");
			if ((CastingFeatureKinds.tokens().indexOf(kind) < 0 || kind == (CastingFeatureKind.InsertRecess:String)))
				throw "Unknown casting strategy feature kind: " + kind;
			if (StrategyOpenings.tokens().indexOf(opening) < 0)
				throw "Invalid casting strategy opening condition";
			if (materia.tooling.casting.CastingSetup.CoreStrategies.tokens().indexOf(strategy) < 0)
				throw "Invalid casting rule strategy";
			if (WithdrawalAxes.tokens().indexOf(axis) < 0 || priority != Math.floor(priority) || Math.abs(priority) > 1000000)
				throw "Invalid casting strategy axis or priority";
			var id = kind + "/" + opening;
			if (ids.exists(id))
				throw "Duplicate casting strategy rule: " + id;
			ids.set(id, true);
			result.push({kind: cast kind, opening: cast opening, strategy: cast strategy,
				priority: Std.int(priority), preferredAxis: cast axis});
		}
		result.sort((a, b) -> Reflect.compare((a.kind : String) + "/" + a.opening, (b.kind : String) + "/" + b.opening));
		return result;
	}

	public static function resolve(document:ToolingDocument):Array<CastingFeatureStrategy> {
		var result:Array<CastingFeatureStrategy> = [];
		for (feature in document.input.features) {
			if (feature.kind == InsertRecess)
				continue;
			var overrides = document.setup.features.filter(s -> s.featureId == feature.id);
			if (overrides.length == 0 && feature.derivedFrom != null)
				overrides = document.setup.features.filter(s -> s.featureId == feature.derivedFrom);
			if (overrides.length == 1) {
				var selected = overrides[0], direction = selected.withdrawal;
				result.push({featureId: feature.id, strategy: selected.strategy, priority: selected.priority,
					withdrawal: direction == null ? null : {x: direction.x, y: direction.y, z: direction.z}});
				continue;
			}
			var openings = feature.openings.filter(o -> o.kind == BodyFace);
			var opening:StrategyOpening = openings.length == 0 ? Enclosed : BodyOpening;
			var rules = document.recipe.featureRules.filter(rule -> rule.kind == feature.kind && rule.opening == opening);
			if (rules.length != 1)
				throw "Missing or duplicate recipe strategy rule for feature: " + feature.id;
			var rule = rules[0];
			var direction:Null<{x:Float, y:Float, z:Float}> = null;
			if (rule.strategy == ReusableFormer) {
				if (openings.length == 0)
					throw "Reusable former rule requires a body opening: " + feature.id;
				var normals = [for (o in openings) {id: o.target, normal: CastingCavities.bodyFaceNormal(document.input.envelope, o.target)}];
				function coordinate(v:{x:Float, y:Float, z:Float}):Float
					return rule.preferredAxis == X ? v.x : rule.preferredAxis == Y ? v.y : v.z;
				var best = 0.0;
				for (candidate in normals) best = Math.max(best, Math.abs(coordinate(candidate.normal)));
				var candidates = normals.filter(candidate -> Math.abs(Math.abs(coordinate(candidate.normal)) - best) <= 1e-9);
				var normal = candidates[0].normal;
				for (candidate in candidates) {
					var v = candidate.normal;
					if (Math.abs(v.x - normal.x) + Math.abs(v.y - normal.y) + Math.abs(v.z - normal.z) > 1e-9)
						throw "Ambiguous former withdrawal; declare an explicit setup override: " + feature.id;
				}
				direction = F.transformVector(document.setup.sourceToMold, normal.x, normal.y, normal.z);
			}
			result.push({featureId: feature.id, strategy: rule.strategy, priority: rule.priority, withdrawal: direction});
		}
		result.sort((a, b) -> Reflect.compare(a.featureId, b.featureId));
		var units = new Map<String, CoreStrategy>();
		for (resolved in result) {
			var feature = document.input.features.filter(source -> source.id == resolved.featureId)[0];
			var parent = feature.derivedFrom == null ? feature.id : feature.derivedFrom;
			if (units.exists(parent) && units.get(parent) != resolved.strategy)
				throw "Core unit strategies differ: " + parent;
			units.set(parent, resolved.strategy);
		}
		return result;
	}
}
