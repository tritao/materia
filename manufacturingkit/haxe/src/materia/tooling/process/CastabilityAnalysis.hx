package materia.tooling.process;

import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;
import materia.casting.BoxAlgebra;
import materia.tooling.casting.CastingMoldPlanner;
import materia.casting.CastingFeature.CastingFeatureKind;
import materia.assembly.AssemblyFrames as F;

import materia.tooling.ToolingCodec;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingIds;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.FillAnalysis;
import materia.tooling.process.OrthogonalSections;
import materia.tooling.casting.CastingMoldPlan.MoldBox;
import materia.tooling.casting.CastingSetup.CoreStrategy;

enum abstract CastabilityAction(String) to String {
	var WidenPassage = "widen-passage";
	var ReduceSection = "reduce-section";
	var RouteVent = "route-vent";
	var ReviewCeiling = "review-ceiling-or-setup";
	var SplitOrSacrificial = "split-or-sacrificial";
	var QualifyBreakout = "qualify-breakout";
	var AnalyzeJunction = "analyze-junction";
	var DeclareLimit = "declare-process-limit";
	var DeclareLiftingPoint = "declare-lifting-point";
}

enum abstract ProposalOwner(String) to String {
	var DesignReview = "design-review";
	var ToolingReview = "tooling-review";
	var ProcessReview = "process-review";
}

/** A review alternative, never an applied edit or a claim of a routed vent.
 * Dimensions and locations are finished geometry in mold coordinates, mm.
 * Unknown parameter mappings remain explicit; the source design owns acceptance.
 */
typedef CastabilityProposal = {
	var id:String;
	var rule:String;
	var sourceIds:Array<String>;
	var action:CastabilityAction;
	var owner:ProposalOwner;
	var measuredMm:Null<Float>;
	var targetMm:Null<Float>;
	var axis:Null<SectionAxis>;
	var region:Null<MoldBox>;
	var point:Null<{x:Float, y:Float, z:Float}>;
	var instruction:String;
	var sourceParameterId:Null<String>;
	var proposedValue:Null<Float>;
}

typedef CastabilityReport = {
	var sourceFingerprint:String;
	var processFingerprint:String;
	var minimumOpposingFaceSpanMm:Null<Float>;
	var designMetrics:CastingDesignMetrics;
	var maximumInscribedThicknessMm:Null<Float>;
	var maximumDiameterLowerMm:Null<Float>;
	var maximumDiameterUpperMm:Null<Float>;
	var validation:Array<ToolingValidation>;
	var proposals:Array<CastabilityProposal>;
}

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

/** Cheap screening over exact shared section/fill records. No CFD, adhesion,
 * geometry edits or duplicate sweep solver.
 */
class CastabilityAnalysis {
	static function sources(ids:Array<String>):Array<String> {
		var out:Array<String> = [];
		for (id in ids)
			if (out.indexOf(id) < 0)
				out.push(id);
		out.sort(Reflect.compare);
		return out;
	}

	static function proposal(rule:String, ids:Array<String>, action:CastabilityAction, owner:ProposalOwner, instruction:String, ?measured:Float,
			?target:Float, ?axis:SectionAxis, ?region:MoldBox, ?point:{
			x:Float,
			y:Float,
			z:Float
		}):CastabilityProposal {
		var record:CastabilityProposal = {
			id: "proposal",
			rule: rule,
			sourceIds: sources(ids),
			action: action,
			owner: owner,
			measuredMm: measured,
			targetMm: target,
			axis: axis,
			region: region,
			point: point,
			instruction: instruction,
			sourceParameterId: null,
			proposedValue: target
		};
		record.id = ToolingIds.derive("proposal-", haxe.crypto.Sha256.encode(ToolingCodec.canonical(record)).substr(0, 24));
		return record;
		}

	public static function analyze(document:ToolingDocument, raw:CastingProcessSpec, fill:Null<FillReport>, sections:Null<OrthogonalSectionReport>,
			expectedSourceFingerprint:String, expectedProcessFingerprint:String, ?plannerChecks:Array<ToolingValidation>):CastabilityReport {
		var d = ToolingCodec.copy(document),
			spec = ProcessSpecs.copy(raw);
		if (spec.recipeFingerprint != ToolingCodec.recipeFingerprint(d.recipe))
			throw "Stale castability recipe";
		if (sections != null && (fill == null || sections.sourceFingerprint != expectedSourceFingerprint || sections.spans.length == 0)
			|| fill != null && fill.sourceFingerprint != expectedSourceFingerprint)
			throw "Mismatched castability section/fill source";
		if (fill != null && fill.processFingerprint != expectedProcessFingerprint)
			throw "Mismatched castability fill process";
		var checks:Array<ToolingValidation> = [],
			proposals:Array<CastabilityProposal> = [];
		var root = d.input.id, limits = spec.castability;
		var levelOne = sourceReview(d);
		checks = checks.concat(levelOne.validation);
		proposals = proposals.concat(levelOne.proposals);
		var minimum:Null<Float> = null, maximum:Null<Float> = null;
		if (sections == null) {
			var geometry = materia.tooling.casting.CastingSourceRegions.derive(d);
			var ids:Array<String> = [];
			for (check in geometry.validation) if (check.status == Unsupported)
				for (id in check.affectedIds) {
					var features = d.input.features.filter(feature -> feature.id == id);
					ids.push(features.length == 1 && features[0].derivedFrom != null ? features[0].derivedFrom : id);
				}
			ids = sources(ids);
			if (ids.length == 0) ids = [root];
			for (rule in ["process.castability.minimum-passage"])
				checks.push(FillAnalysis.check(rule, ids, Unsupported,
					"No exact section interval is available for this geometry; no box or scalar substitute is credited.", []));
		} else {
			var spans = sections.spans.copy();
			spans.sort((a, b) -> a.thicknessMm != b.thicknessMm ? Reflect.compare(a.thicknessMm, b.thicknessMm) : Reflect.compare(a.id, b.id));
			minimum = spans[0].thicknessMm;
			var minimumSpan = spans[0];
			var minimumOwners = sources(minimumSpan.lowerSourceIds.concat(minimumSpan.upperSourceIds));
			var minimumRule = "process.castability.minimum-passage";
			if (limits.maximumAggregateSizeMm == null || limits.minimumPassageAggregateRatio == null) {
				var missing:Array<String>=[];
				if (limits.maximumAggregateSizeMm==null) missing.push("maximum aggregate size");
				if (limits.minimumPassageAggregateRatio==null) missing.push("minimum passage/aggregate ratio");
				checks.push(FillAnalysis.check(minimumRule, minimumOwners, Pending,
					"Declare "+missing.join(" and ")+"; narrowest opposing-face span is " + minimum + " mm.", limits.evidence));
				proposals.push(proposal(minimumRule, minimumOwners, DeclareLimit, ProcessReview,
					"Supply the mix grading and recipe passage ratio, then rerun the section screen.", minimum, null, minimumSpan.axis, minimumSpan.strips[0]));
			} else {
				var required = limits.maximumAggregateSizeMm * limits.minimumPassageAggregateRatio;
				var narrow = spans.filter(s -> s.thicknessMm < required - 1e-7);
				var ids = sources([for (s in narrow) for (id in s.lowerSourceIds.concat(s.upperSourceIds)) id]);
				checks.push(FillAnalysis.check(minimumRule, ids.length == 0 ? [root] : ids, narrow.length == 0 ? Pass : Fail,
					"Opposing-face passage screen: minimum "
					+ minimum
					+ " mm; declared aggregate-based minimum "
					+ required
					+ " mm. This is a geometry screen, not a prediction of paste flow or compaction.",
					limits.evidence));
				for (s in narrow)
					proposals.push(proposal(minimumRule, s.lowerSourceIds.concat(s.upperSourceIds), WidenPassage, DesignReview,
						"Increase separation of these source faces to at least the target, or select a finer qualified aggregate mix. Recheck stiffness and tooling.",
						s.thicknessMm, required, s.axis, s.strips[0]));
			}
			maximum = sections.maximumInscribedThicknessMm;
			switch spec.compound.thickness {
				case Section(limit):
					var bound=sections.junction;
					if (bound.lowerDiameterMm>limit+1e-7)
						proposals.push(proposal("process.compound.datasheet",bound.witnessSourceIds,ReduceSection,DesignReview,
							"Reduce the section near this witness or select a compound whose datasheet admits it; recheck structural performance.",
							bound.lowerDiameterMm,limit,null,null,bound.witness));
					else if (bound.upperDiameterMm>limit+1e-7)
						proposals.push(proposal("process.compound.datasheet",bound.witnessSourceIds,AnalyzeJunction,ProcessReview,
							"Refine the section interval until it resolves the datasheet threshold; no midpoint acceptance.",
							null,limit,null,null,bound.witness));
				case Layer(_),Unresolved(_):
			}
		}
		if (fill != null) {
			// Keep the exact fill rule/status/ownership: suggestions do not clear it.
			for (check in fill.validation) {
				checks.push(check);
				if (check.rule == "process.fill.air-escape-width" && check.status != Pass)
					proposals.push(proposal(check.rule, check.affectedIds, check.status == Pending ? DeclareLimit : ReviewCeiling,
						check.status == Pending ? ProcessReview : ToolingReview,
						check.status == Pending ? "Declare the recipe ceiling air-escape-width limit and rerun filling checks."
							: "Evaluate another setup or design a vent route for these ceilings; verify its access and seal before replanning."));
			}
			for (pocket in fill.trappedPockets)
				proposals.push(proposal("process.fill.trap." + pocket.id, pocket.sourceIds, RouteVent, ToolingReview,
					"Route a continuous vent from this candidate ceiling point to atmosphere, with a sealing/plug plan; if it crosses casting material, request a source change. For a hardware-bounded recess, an explicitly specified pre-pour fill/seal is an alternative: model its actual occupied volume before treating the recess as closed. Rerun topology and CAD.",
					null, null, null, null, pocket.proposedVentPoint));
		}
		var strategies = materia.tooling.casting.CastingStrategies.resolve(d);
		strategies.sort((a, b) -> Reflect.compare(a.featureId, b.featureId));
		var reportedParents = new Map<String, Bool>();
		for (strategy in strategies) {
			var source = d.input.features.filter(feature -> feature.id == strategy.featureId)[0];
			var parent = source.derivedFrom == null ? source.id : source.derivedFrom;
			if (reportedParents.exists(parent))
				continue;
			reportedParents.set(parent, true);
			var rule = "process.castability.release." + parent;
			if (strategy.strategy == SacrificialEPS) {
				checks.push(FillAnalysis.check(rule, [parent], Pending,
					"EPS breakout accessibility, fragment size and release need a physical trial; a rigid-member sweep does not establish breakup access.",
					[]));
				proposals.push(proposal(rule, [parent], QualifyBreakout, ToolingReview,
					"Trial breakout through the declared opening after fixture release; record access, remaining foam and damage against this source revision."));
			} else if (strategy.strategy == ReusableFormer) {
				var draftRule = "tooling.former.draft." + strategy.featureId;
				var siblings = d.input.features.filter(feature -> (feature.derivedFrom == null ? feature.id : feature.derivedFrom) == parent);
				var draftRules = [for (feature in siblings) "tooling.former.draft." + feature.id];
				var actual = plannerChecks == null ? [] : plannerChecks.filter(v -> draftRules.indexOf(v.rule) >= 0);
				if (actual.length > 0) {
					for (check in actual)
						checks.push(check);
					if (siblings.length == 1)
						rule = draftRule;
					else {
						var status = actual.filter(v -> v.status == Unsupported).length > 0 ? Unsupported
							: actual.filter(v -> v.status == Fail).length > 0 ? Fail
							: actual.length < siblings.length || actual.filter(v -> v.status == Pending).length > 0 ? Pending : Pass;
						checks.push(FillAnalysis.check(rule, [parent], status,
							"Release qualification covers every fragment of the authored core unit.", []));
					}
				} else
					checks.push(FillAnalysis.check(rule, [parent], Pending,
						"Run the real planner and exact swept removal checks for rigid-former extraction; adhesion needs physical evidence.", []));
				if (actual.length == 0 || actual.filter(v -> v.status != Pass).length > 0)
					proposals.push(proposal(rule, [parent], SplitOrSacrificial, DesignReview,
						"Declare supported positive source draft aligned with withdrawal, or a supported sacrificial alternative; rerun planning and complete CAD removal checks."));
			} else {
				checks.push(FillAnalysis.check(rule, [parent], Unsupported, "Feature has no supported tooling strategy.", []));
				proposals.push(proposal(rule, [parent], SplitOrSacrificial, ToolingReview,
					"Select an explicit supported tooling strategy and rerun planning before designing its release."));
			}
		}
		checks.sort((a, b) -> Reflect.compare(a.rule, b.rule));
		proposals.sort((a, b) -> Reflect.compare(a.id, b.id));
		return {
			sourceFingerprint: expectedSourceFingerprint,
			processFingerprint: expectedProcessFingerprint,
			minimumOpposingFaceSpanMm: minimum,
			designMetrics: levelOne.metrics,
			maximumInscribedThicknessMm: maximum,
			maximumDiameterLowerMm: sections == null ? null : sections.junction.lowerDiameterMm,
			maximumDiameterUpperMm: sections == null ? null : sections.junction.upperDiameterMm,
			validation: checks,
			proposals: proposals
		};
	}

	/** Source-only screening shared by design diagnostics and the mold review. */
	public static function sourceReview(document:ToolingDocument):CastingDesignReport {
		var d = ToolingCodec.copy(document),
			limits = d.recipe.designReview;
		var checks:Array<ToolingValidation> = [],
			proposals:Array<CastabilityProposal> = [];
		var metrics:CastingDesignMetrics = {
			minimumCornerRadiusMm: null,
			minimumInsertEdgeMm: null,
			minimumEmbeddedDepthMm: null,
			minimumFixingAccessLandMm: null,
			liftingPointCount: materia.tooling.casting.CastHandling.liftingIds(d).length,
			checkedRemovalDirections: 0,
			blockedRemovalDirections: 0
		};
		function report():CastingDesignReport {
			checks.sort((a, b) -> Reflect.compare(a.rule, b.rule));
			proposals.sort((a, b) -> Reflect.compare(a.id, b.id));
			return {validation: checks, proposals: proposals, metrics: metrics};
		}
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
				var radius = materia.casting.CastingProfileGeometry.minimumInternalRadius(
					materia.casting.CastingFeatureContours.contour(feature));
				var previous = radiusByParent.get(parent);
				radiusByParent.set(parent, previous == null ? radius : Math.min(previous, radius));
			}
		for (radius in radiusByParent)
			metrics.minimumCornerRadiusMm = metrics.minimumCornerRadiusMm == null ? radius : Math.min(metrics.minimumCornerRadiusMm, radius);
		var parents = [for (parent in radiusByParent.keys()) parent];
		parents.sort(Reflect.compare);
		for (parent in parents)
			finding("process.level1.corner-radius." + parent, parent, radiusByParent.get(parent), limits.minimumCornerRadiusMm,
				"Authored source wall-profile radius needs the recipe minimum; any change requires design acceptance and replanning.", null,
				limits.minimumCornerRadiusMm);
		var liftCount = materia.tooling.casting.CastHandling.liftingIds(d).length;
		finding("process.level1.lifting-points", d.input.id, liftCount, limits.minimumLiftingPoints == null ? null : 1.0 * limits.minimumLiftingPoints,
			"Typed lifting-point count versus recipe requirement; load capacity, CoM and a qualified cradle remain separate evidence.", null, null);
		// Geometry capability is separate from anchor bearing/capacity evidence.
		// Multiple non-box fragments still produce one owned source finding.
		var support = materia.tooling.casting.CastingBoxSupport.check(d.input, "process.level1.geometry");
		var unsupported = support.filter(check -> check.status == Unsupported);
		var geometryOwners = new Map<String, Array<String>>(), geometryMessages = new Map<String, Array<String>>();
		for (check in unsupported) {
			var parents:Array<String> = [];
			for (id in check.affectedIds) {
				var sources = d.input.features.filter(feature -> feature.id == id);
				var parent = sources.length == 0 || sources[0].derivedFrom == null ? id : sources[0].derivedFrom;
				if (parents.indexOf(parent) < 0) parents.push(parent);
			}
			parents.sort(Reflect.compare);
			var key = ToolingCodec.canonical(parents);
			geometryOwners.set(key, parents);
			var messages = geometryMessages.exists(key) ? geometryMessages.get(key) : [];
			if (messages.indexOf(check.message) < 0) messages.push(check.message);
			geometryMessages.set(key, messages);
		}
		for (key in geometryOwners.keys()) {
			var parents = geometryOwners.get(key), messages = geometryMessages.get(key);
			messages.sort(Reflect.compare);
			var suffix = parents.length == 1 ? parents[0] : "group." + haxe.crypto.Sha256.encode(key).substr(0, 20);
			checks.push(FillAnalysis.check("process.level1.geometry." + suffix, parents, Unsupported, messages.join(" "), []));
		}

		var envelope:Null<MoldBox> = null;
		try
			envelope = CastingMoldPlanner.envelope(d)
		catch (error:Dynamic) {
			if (unsupported.length == 0)
				checks.push(FillAnalysis.check("process.level1.geometry", [d.input.id], Unsupported,
					"Source-only box screening cannot represent this envelope: " + Std.string(error), []));
		}

		for (insert in d.input.inserts) {
			var dimensions = switch insert.body {
				case Primitive(materia.casting.CastingSolid.Box(width, depth, height)):
					{width: width, depth: depth, height: height};
				case _: null;
			};
			if (dimensions == null) {
				for (name in ["insert-edge", "insert-cover", "fixing-access"])
					checks.push(FillAnalysis.check("process.level1." + name + "." + insert.id, [insert.id], Unsupported,
						"This source-only insert screen requires exact box stock; no cylindrical stock bound is substituted.", []));
				continue;
			}
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
			var insertToMold = F.compose(d.setup.sourceToMold, insert.pose);
			var axial = [F.transformVector(insertToMold, 1, 0, 0), F.transformVector(insertToMold, 0, 1, 0),
				F.transformVector(insertToMold, 0, 0, 1)].filter(axis -> materia.casting.CastingBounds.axis(axis) == null).length == 0;
			if (envelope == null || !axial) {
				for (name in ["insert-edge", "insert-cover"])
					checks.push(FillAnalysis.check("process.level1." + name + "." + insert.id, [insert.id], Unsupported,
						"This source-only distance/embedded-depth screen requires exact axis-aligned body and insert boxes.", []));
				continue;
			}
			var box = CastingMoldPlanner.box(dimensions.width, dimensions.depth, dimensions.height, insertToMold);
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
					var smaller = CastingMoldPlanner.box(proposed, dimensions.depth, dimensions.height, F.compose(d.setup.sourceToMold, insert.pose));
					var improved = Math.min(Math.min(smaller.x - envelope.x, envelope.x + envelope.width - smaller.x - smaller.width),
						Math.min(smaller.y - envelope.y, envelope.y + envelope.depth - smaller.y - smaller.depth));
					var requiredEdge:Float = cast limits.minimumInsertEdgeMm;
					if (improved < requiredEdge - 1e-7)
						proposed = null;
				}
			}
			if (proposed != null) {
				var features = materia.casting.CastingFeatureCodec.copy(d.input.features);
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

		}
		if (limits.minimumLiftingPoints!=null && liftCount<limits.minimumLiftingPoints && envelope!=null) {
			var requiredLifts:Int = cast limits.minimumLiftingPoints;
			// Embedded-anchor moments have their own exact analytic mass path;
			// needing native fill does not itself prevent a source mass estimate.
			var massNeedsKernel = materia.tooling.casting.CastingBoxSupport.check(d.input, "tooling.handling.geometry", false)
				.filter(check -> check.status == Unsupported).length > 0;
			if (!massNeedsKernel || CastingMoldPlanner.geometry!=null)
				proposals=proposals.concat(liftingCandidates(d,envelope,requiredLifts));
		}
		var resin = envelope==null || unsupported.length>0 ? null
			: BoxAlgebra.subtract([envelope], [for (feature in d.input.features) CastingMoldPlanner.featureBox(d, feature)]);
		for (strategy in materia.tooling.casting.CastingStrategies.resolve(d)) {
			var rule = "process.level1.removal-direction." + strategy.featureId;
			if (strategy.strategy != ReusableFormer || strategy.withdrawal == null || limits.removalClearanceMm == null) {
				checks.push(FillAnalysis.check(rule, [strategy.featureId], Pending,
					"Declare a rigid withdrawal and recipe clearance, or qualify sacrificial breakout.", []));
				continue;
			}
			if (envelope == null || resin == null) {
				checks.push(FillAnalysis.check(rule, [strategy.featureId], Unsupported,
					"The source-only box sweep cannot represent this resin geometry; complete native removal checks remain required.", []));
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
		return report();
	}

	/** Review locations only: creating a source insert is not a scalar edit.
	 * Supported box geometry uses handling's existing density-weighted moments.
	 * Source authoring must verify material, access and actual anchor occupancy. */
	static function liftingCandidates(d:ToolingDocument,envelope:MoldBox,count:Int):Array<CastabilityProposal> {
		if (count>ToolingCodec.MAX_RECORDS) return []; // The source cannot contain more insert records.
		var moments=materia.tooling.casting.CastHandling.moments(d);
		var inverse=F.inverse(d.setup.sourceToMold),mold=moments.centerOfMassMoldMm;
		var center=F.transformPoint(inverse,mold.x,mold.y,mold.z);
		var source=CastingMoldPlanner.box(envelope.width,envelope.depth,envelope.height,
			F.compose(inverse,F.translation(envelope.x+envelope.width/2,envelope.y+envelope.depth/2,envelope.z)));
		var result:Array<CastabilityProposal> = [];
		for (location in materia.casting.InsertAnchorage.InsertAnchorSelection.liftingPositions(source,center,count)) {
			var x=location.x,y=location.y;
			var point=F.transformPoint(d.setup.sourceToMold,x,y,location.z);
			var proposal:CastabilityProposal={id:"proposal",rule:"process.level1.lifting-points",sourceIds:[d.input.id],
				action:DeclareLiftingPoint,owner:DesignReview,measuredMm:null,targetMm:null,axis:null,region:null,point:point,
				instruction:"Review a source lifting connection at source XYZ="+x+","+y+","+(source.z+source.height)
					+" for nominal casting mass "+moments.massKg+" kg. The candidate pattern surrounds the projected CoM; "
					+"verify solid material, access, policy-compliant source threads, bolted anchorage and rigging capacity on replan.",
				sourceParameterId:null,proposedValue:null};
			proposal.id="proposal-"+haxe.crypto.Sha256.encode(ToolingCodec.canonical(proposal)).substr(0,24);
			result.push(proposal);
		}
		return result;
	}

	/** Explicitly accepted mapped edit, returned as a detached source revision.
	 * Unsupported mappings never become guessed source edits.
	 */
	public static function apply(document:ToolingDocument, proposal:CastabilityProposal):ToolingDocument {
		var available = sourceReview(document).proposals.filter(p -> p.id == proposal.id
			&& ToolingCodec.canonical(p) == ToolingCodec.canonical(proposal));
		if (available.length != 1 || proposal.sourceParameterId == null || proposal.proposedValue == null)
			throw "Unmapped or stale source proposal";
		var d = ToolingCodec.copy(document);
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
		return ToolingCodec.copy(d);
	}
}
