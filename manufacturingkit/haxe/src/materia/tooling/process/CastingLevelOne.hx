package materia.tooling.process;

import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;

import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.casting.BoxAlgebra;
import materia.tooling.casting.BoxMoldPlanner;
import materia.tooling.casting.CastingMoldPlanner;
import materia.casting.CastingFeature.CastingFeatureKind;
import materia.tooling.casting.CastingSetup.CoreStrategy;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.process.CastabilityAnalysis;
import materia.assembly.AssemblyFrames as F;

typedef CastingDesignMetrics = {
	var minimumCornerRadiusMm:Null<Float>;
	var minimumInsertEdgeMm:Null<Float>;
	var minimumEmbeddedDepthMm:Null<Float>;
	var minimumFixingAccessLandMm:Null<Float>;
	var liftingPointCount:Int;
	var checkedRemovalDirections:Int;
	var blockedRemovalDirections:Int;
}

typedef CastingDesignReport = {
	var validation:Array<ToolingValidation>;
	var proposals:Array<CastabilityProposal>;
	var metrics:CastingDesignMetrics;
}

/** Source-only screening. No mold planning, CAD, inferred capacities or source mutation. */
class CastingLevelOne {
	public static function analyze(document:ToolingDocument):CastingDesignReport {
		var d = ToolingCodec.snapshot(document),
			limits = d.recipe.designReview;
		var checks:Array<ToolingValidation> = [],
			proposals:Array<CastabilityProposal> = [];
		var metrics:CastingDesignMetrics = {
			minimumCornerRadiusMm: null,
			minimumInsertEdgeMm: null,
			minimumEmbeddedDepthMm: null,
			minimumFixingAccessLandMm: null,
			liftingPointCount: d.input.inserts.filter(i -> i.role == materia.casting.InsertSpec.InsertRole.LiftingMount).length,
			checkedRemovalDirections: 0,
			blockedRemovalDirections: 0
		};
		function finding(rule:String, id:String, measured:Float, threshold:Null<Float>, message:String, ?parameter:String, ?value:Float,
				minimum:Bool = true):Void {
			var status = threshold == null ? Pending : (minimum ? measured + 1e-7 >= threshold : measured <= threshold + 1e-7) ? Pass : Fail;
			checks.push(FillAnalysis.check(rule, [id], status, message, []));
			if (status != Pass) {
				var proposal:CastabilityProposal = {
					id: "proposal",
					rule: rule,
					sourceIds: [id],
					action: DeclareLimit,
					owner: DesignReview,
					measuredMm: measured,
					targetMm: threshold,
					axis: null,
					region: null,
					point: null,
					instruction: message,
					sourceParameterId: parameter,
					proposedValue: value
				};
				proposal.id = "proposal-" + haxe.crypto.Sha256.encode(ToolingCodec.canonical(proposal)).substr(0, 24);
				proposals.push(proposal);
			}
		}
		var radiusByParent = new Map<String, Float>();
		for (feature in d.input.features)
			if (feature.kind != InsertRecess) {
				var parent = feature.derivedFrom == null ? feature.id : feature.derivedFrom;
				var previous = radiusByParent.get(parent);
				radiusByParent.set(parent, previous == null ? feature.cornerRadius : Math.min(previous, feature.cornerRadius));
			}
		for (radius in radiusByParent)
			metrics.minimumCornerRadiusMm = metrics.minimumCornerRadiusMm == null ? radius : Math.min(metrics.minimumCornerRadiusMm, radius);
		var parents = [for (parent in radiusByParent.keys()) parent];
		parents.sort(Reflect.compare);
		for (parent in parents)
			finding("process.level1.corner-radius." + parent, parent, radiusByParent.get(parent), limits.minimumCornerRadiusMm,
				"Source corner radius needs the declared minimum; any change requires design acceptance and replanning.", null,
				limits.minimumCornerRadiusMm);
		var support = CastingMoldPlanner.sourceSupport(d.input);
		var unsupported = support.filter(check -> check.status == Unsupported);
		if (unsupported.length > 0) {
			for (check in unsupported) {
				var parents:Array<String> = [];
				for (id in check.affectedIds) {
					var sources = d.input.features.filter(feature -> feature.id == id);
					var parent = sources.length == 0 || sources[0].derivedFrom == null ? id : sources[0].derivedFrom;
					if (parents.indexOf(parent) < 0) parents.push(parent);
				}
				parents.sort(Reflect.compare);
				checks.push(FillAnalysis.check("process.level1.geometry." + parents.join("-"),
					parents, Unsupported, check.message, []));
			}
			return {validation: checks, proposals: proposals, metrics: metrics};
		}

		var envelope:MoldBox;
		try
			envelope = CastingMoldPlanner.envelope(d)
		catch (error:Dynamic) {
			checks.push(FillAnalysis.check("process.level1.geometry", [d.input.id], Unsupported,
				"Source-only box screening cannot represent this envelope: " + Std.string(error), []));
			return {validation: checks, proposals: proposals, metrics: metrics};
		}

		for (insert in d.input.inserts) {
			var dimensions = InsertBodies.box(insert.body, insert.id);
			var box = BoxMoldPlanner.box(dimensions.width, dimensions.depth, dimensions.height, F.compose(d.setup.sourceToMold, insert.pose));
			var edge = Math.min(Math.min(box.x - envelope.x, envelope.x + envelope.width - box.x - box.width),
				Math.min(box.y - envelope.y, envelope.y + envelope.depth - box.y - box.depth));
			var wet = BoxAlgebra.intersection(box, envelope);
			var embed = wet == null ? 0 : wet.width * wet.depth * wet.height / (dimensions.width * dimensions.depth);
			metrics.minimumInsertEdgeMm = metrics.minimumInsertEdgeMm == null ? edge : Math.min(metrics.minimumInsertEdgeMm, edge);
			metrics.minimumEmbeddedDepthMm = metrics.minimumEmbeddedDepthMm == null ? embed : Math.min(metrics.minimumEmbeddedDepthMm, embed);
			var proposed:Null<Float> = limits.minimumInsertEdgeMm == null ? null : dimensions.width - 2 * Math.max(0, limits.minimumInsertEdgeMm - edge);
			// Only propose a width change when it actually improves the limiting edge.
			if (proposed != null) {
				var candidateWidth:Float = cast proposed;
				for (hole in insert.holes)
					if (materia.casting.InsertHoleGeometry.radialWall(candidateWidth, dimensions.depth, dimensions.height, hole)
						< Math.max(d.recipe.minimumLigament, hole.limits.minWall))
						proposed = null;
				if (proposed != null) {
					var smaller = BoxMoldPlanner.box(proposed, dimensions.depth, dimensions.height, F.compose(d.setup.sourceToMold, insert.pose));
					var improved = Math.min(Math.min(smaller.x - envelope.x, envelope.x + envelope.width - smaller.x - smaller.width),
						Math.min(smaller.y - envelope.y, envelope.y + envelope.depth - smaller.y - smaller.depth));
					var requiredEdge:Float = cast limits.minimumInsertEdgeMm;
					if (improved < requiredEdge - 1e-7)
						proposed = null;
				}
			}
			if (proposed != null) {
				var features = materia.casting.CastingFeatureCodec.snapshot(d.input.features);
				var recesses = features.filter(feature -> feature.owner == insert.id);
				if (recesses.length == 0 || recesses.filter(feature -> feature.adjustable.indexOf(materia.casting.CastingFeature.CastingDimension.Width) < 0).length > 0)
					proposed = null;
				else {
					for (feature in recesses)
						feature.geometry.solid = materia.casting.CastingFeature.CastingGeometrySolid.Insert(materia.casting.InsertSpec.InsertBody.Primitive(materia.casting.CastingSolid.Box(proposed, dimensions.depth, dimensions.height)));
					try materia.casting.CastingCavities.derive(d.input.envelope, features)
					catch (error:Dynamic) proposed = null;
				}
			}
			finding("process.level1.insert-edge."
				+ insert.id, insert.id, edge, limits.minimumInsertEdgeMm,
				"Insert side edge distance needs the recipe minimum; any width change requires design acceptance and replanning.",
				"input.inserts."
				+ insert.id
				+ ".body.solid.width", proposed);
			finding("process.level1.insert-cover."
				+ insert.id, insert.id, embed, limits.minimumInsertCoverMm,
				"Average embedded depth (intersection volume / seat area) is a geometric screen, not anchorage capacity.",
				"input.inserts."
				+ insert.id
				+ ".body.solid.height", null);
			if (insert.holes.length > 0) {
				var lands = [for (hole in insert.holes) materia.casting.InsertHoleGeometry.radialWall(dimensions.width, dimensions.depth, dimensions.height, hole)];
				lands.sort(Reflect.compare);
				var land = lands[0];
				metrics.minimumFixingAccessLandMm = metrics.minimumFixingAccessLandMm == null ? land : Math.min(metrics.minimumFixingAccessLandMm, land);
				finding("process.level1.fixing-access."
					+ insert.id, insert.id, land,
					limits.minimumFixingAccessMm,
					"Declared bore-to-edge access land compared with recipe minimum; tool/fixture accessibility remains a separate check.",
					"input.inserts."
					+ insert.id
					+ ".holes", null);
			} else
				checks.push(FillAnalysis.check("process.level1.fixing-access." + insert.id, [insert.id], Pending,
					"No source bore specifies fixing access; qualify an external attachment or revise the source interface.", []));
		}
		var liftCount = d.input.inserts.filter(i -> i.role == materia.casting.InsertSpec.InsertRole.LiftingMount).length;
		finding("process.level1.lifting-points", d.input.id, liftCount, limits.minimumLiftingPoints == null ? null : 1.0 * limits.minimumLiftingPoints,
			"Typed lifting-point count versus recipe requirement; load capacity, CoM and a qualified cradle remain separate evidence.", null, null);
		var resin = BoxAlgebra.subtract([envelope], [for (feature in d.input.features) CastingMoldPlanner.featureBox(d, feature)]);
		for (strategy in materia.tooling.casting.CastingStrategies.resolve(d)) {
			var rule = "process.level1.removal-direction." + strategy.featureId;
			if (strategy.strategy != ReusableFormer || strategy.withdrawal == null || limits.removalClearanceMm == null) {
				checks.push(FillAnalysis.check(rule, [strategy.featureId], Pending,
					"Declare a rigid withdrawal and recipe clearance, or qualify sacrificial breakout.", []));
				continue;
			}
			var direction = strategy.withdrawal;
			if ([direction.x, direction.y, direction.z].filter(v -> Math.abs(v) > 1e-7).length != 1) {
				checks.push(FillAnalysis.check(rule, [strategy.featureId], Unsupported,
					"Source-only sweep supports axis-aligned withdrawal; exact CAD remains required.", []));
				continue;
			}
			var feature = d.input.features.filter(f -> f.id == strategy.featureId)[0],
				box = CastingMoldPlanner.featureBox(d, feature);
			var clearance:Float = cast limits.removalClearanceMm;
			var distance = envelope.width + envelope.depth + envelope.height + clearance;
			if (Math.abs(direction.x) < 1e-7) {
				box.x -= clearance;
				box.width += 2 * clearance;
			}
			if (Math.abs(direction.y) < 1e-7) {
				box.y -= clearance;
				box.depth += 2 * clearance;
			}
			if (Math.abs(direction.z) < 1e-7) {
				box.z -= clearance;
				box.height += 2 * clearance;
			}
			var sweep:MoldBox = {
				x: box.x + Math.min(0, direction.x * distance),
				y: box.y + Math.min(0, direction.y * distance),
				z: box.z + Math.min(0, direction.z * distance),
				width: box.width + Math.abs(direction.x * distance),
				depth: box.depth + Math.abs(direction.y * distance),
				height: box.height + Math.abs(direction.z * distance)
			};
			var blocked = resin.filter(piece -> BoxAlgebra.intersection(piece, sweep) != null).length > 0;
			metrics.checkedRemovalDirections++;
			if (blocked)
				metrics.blockedRemovalDirections++;
			checks.push(FillAnalysis.check(rule, [strategy.featureId], blocked ? Fail : Pass,
				"Exact orthogonal source-resin sweep in the declared direction; tooling-to-tooling blocking is a later CAD check.", []));
		}
		checks.sort((a, b) -> Reflect.compare(a.rule, b.rule));
		proposals.sort((a, b) -> Reflect.compare(a.id, b.id));
		return {validation: checks, proposals: proposals, metrics: metrics};
	}

	/** Explicitly accepted mapped edit, returned as a detached source revision.
	 * Unsupported mappings never become guessed source edits.
	 */
	public static function apply(document:ToolingDocument, proposal:CastabilityProposal):ToolingDocument {
		var available = analyze(document).proposals.filter(p -> p.id == proposal.id
			&& ToolingCodec.canonical(p) == ToolingCodec.canonical(proposal));
		if (available.length != 1 || proposal.sourceParameterId == null || proposal.proposedValue == null)
			throw "Unmapped or stale source proposal";
		var d = ToolingCodec.snapshot(document);
		var inserts = d.input.inserts.filter(i -> proposal.sourceParameterId == "input.inserts." + i.id + ".body.solid.width");
		var proposedValue:Float = cast proposal.proposedValue;
		if (inserts.length != 1 || proposedValue <= 0)
			throw "Unsupported accepted source parameter";
		var insert = inserts[0];
		var dimensions = InsertBodies.box(insert.body, insert.id);
		insert.body = materia.casting.InsertSpec.InsertBody.Primitive(materia.casting.CastingSolid.Box(proposedValue, dimensions.depth, dimensions.height));
		for (feature in d.input.features)
			if (feature.owner == insert.id) {
				feature.geometry.solid = materia.casting.CastingFeature.CastingGeometrySolid.Insert(insert.body);
			}
		d.input.revision++;
		materia.casting.CastingCavities.derive(d.input.envelope, d.input.features);
		d.setup.revision++;
		d.setup.sourceFingerprint = ToolingCodec.inputFingerprint(d.input);
		return ToolingCodec.snapshot(d);
	}
}
