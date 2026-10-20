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
					priority: kind == ServiceOpening ? 30 : kind == Clearance ? 20 : 0,
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
			if (["lightening", "service-opening", "interface-recess", "clearance"].indexOf(kind) < 0)
				throw "Unknown casting strategy feature kind: " + kind;
			if (opening != "body-opening" && opening != "enclosed")
				throw "Invalid casting strategy opening condition";
			if (["sacrificial-eps", "reusable-former", "unsupported"].indexOf(strategy) < 0)
				throw "Invalid casting rule strategy";
			if (["x", "y", "z"].indexOf(axis) < 0 || priority != Math.floor(priority) || Math.abs(priority) > 1000000)
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
				normals.sort((a, b) -> {
					var aa = coordinate(a.normal), bb = coordinate(b.normal);
					if (Math.abs(Math.abs(aa) - Math.abs(bb)) > 1e-9)
						return Reflect.compare(Math.abs(bb), Math.abs(aa));
					if (Math.abs(aa - bb) > 1e-9)
						return Reflect.compare(bb, aa);
					return Reflect.compare(a.id, b.id);
				});
				var normal = normals[0].normal;
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
