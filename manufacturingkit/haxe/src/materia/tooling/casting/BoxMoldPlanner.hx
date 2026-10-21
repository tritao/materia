package materia.tooling.casting;

import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingEnums.ContactFace;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelJoints.PanelJointKind;
import materia.tooling.panel.PanelJoints.PanelClosure;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelAssemblyPlan.PanelPlacement;
import materia.tooling.panel.PanelAssemblyPlan.PanelAssemblyRecord;
import materia.casting.CastingFeature.CastingFeatureKind;
import materia.tooling.casting.CastingSetup.CoreStrategy;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.casting.MoldPlan.PenetrationPlan;
import materia.tooling.casting.CoreSupport.CoreSupportOptions;
import materia.tooling.casting.CoreSupport.CoreSupportKind;

/** Explicit orthogonal box template. Unsupported cuts/orientations fail rather than disappear.
 * Four independently liftable walls sit in blind bed sockets outside the casting.
 * The continuous inside lands own casting size; clearance is on the outside of sockets.
 */
class BoxMoldPlanner {
	public static function box(width:Float, depth:Float, height:Float, pose:AssemblyFrame):MoldBox {
		var points = [];
		for (x in [-width / 2, width / 2])
			for (y in [-depth / 2, depth / 2])
				for (z in [0.0, height])
					points.push(F.transformPoint(pose, x, y, z));
		var lo = {x: 1e30, y: 1e30, z: 1e30},
			hi = {x: -1e30, y: -1e30, z: -1e30};
		for (p in points) {
			lo.x = Math.min(lo.x, p.x);
			lo.y = Math.min(lo.y, p.y);
			lo.z = Math.min(lo.z, p.z);
			hi.x = Math.max(hi.x, p.x);
			hi.y = Math.max(hi.y, p.y);
			hi.z = Math.max(hi.z, p.z);
		}
		for (p in points)
			if (Math.min(Math.abs(p.x - lo.x), Math.abs(p.x - hi.x)) > 1e-7
				|| Math.min(Math.abs(p.y - lo.y), Math.abs(p.y - hi.y)) > 1e-7
				|| Math.min(Math.abs(p.z - lo.z), Math.abs(p.z - hi.z)) > 1e-7)
				throw "Box mold needs axis-aligned setup geometry";
		return {
			x: lo.x,
			y: lo.y,
			z: lo.z,
			width: hi.x - lo.x,
			depth: hi.y - lo.y,
			height: hi.z - lo.z
		};
	}

	public static function volume(b:MoldBox):Float
		return b.width * b.depth * b.height;

	public static function clipped(a:MoldBox, b:MoldBox):MoldBox {
		var x = Math.max(a.x, b.x),
			y = Math.max(a.y, b.y),
			z = Math.max(a.z, b.z);
		var out = {
			x: x,
			y: y,
			z: z,
			width: Math.min(a.x + a.width, b.x + b.width) - x,
			depth: Math.min(a.y + a.depth, b.y + b.depth) - y,
			height: Math.min(a.z + a.height, b.z + b.height) - z
		};
		if (out.width <= 0 || out.depth <= 0 || out.height <= 0)
			throw "Feature misses casting envelope";
		return out;
	}

	public static function plan(input:ToolingDocument, ?supportOptions:CoreSupportOptions):MoldPlan {
		var d = ToolingCodec.snapshot(input), r = d.recipe, e = d.input.envelope;
		var options = CoreSupports.options(supportOptions == null ? CoreSupports.defaults(r) : supportOptions);
		if (e.length != 1 || e[0].profile.length != 4)
			throw "Box template needs one rectangular envelope";
		var p = e[0].profile, x = p[0].x, y = p[0].y, w = 0.0, h = 0.0;
		for (v in p) {
			x = Math.min(x, v.x);
			y = Math.min(y, v.y);
		}
		for (v in p) {
			w = Math.max(w, v.x - x);
			h = Math.max(h, v.y - y);
		}
		for (v in p)
			if ((Math.abs(v.x - x) > 1e-8 && Math.abs(v.x - x - w) > 1e-8) || (Math.abs(v.y - y) > 1e-8 && Math.abs(v.y - y - h) > 1e-8))
				throw "Envelope is not a rectangle";
		var ep = F.compose(d.setup.sourceToMold, e[0].pose),
			center = F.compose(ep, F.translation(x + w / 2, y + h / 2, 0));
		var b = box(w, h, e[0].height, center), t = r.panel.measuredThickness, c = r.jointClearance, seat = t / 2;
		if (seat < r.minimumLigament)
			throw "Bed sockets leave insufficient stock";
		var shell = MoldPanels.box(b, r),
			assembly = shell.panels,
			penetrations = shell.penetrations,
			bed = assembly.members[0].panel;
		var ox = assembly.members[0].pose.x, oy = assembly.members[0].pose.y;
		var supportPanels:Array<PanelPlacement> = [];
		var cores:Array<materia.tooling.casting.MoldPlan.CorePlan> = [];
		for (f in d.input.features)
			if (f.kind != CastingFeatureKind.InsertRecess) {
				var strategy = CastingStrategies.resolve(d).filter(s -> s.featureId == f.id)[0];
				if (strategy.strategy != CoreStrategy.SacrificialEPS)
					throw "Box template supports EPS only";
				var cb = CastingMoldPlanner.featureBox(d,f),
					l = r.linerThickness;
				if (cb.x <= b.x || cb.y <= b.y || cb.x + cb.width >= b.x + b.width || cb.y + cb.depth >= b.y + b.depth)
					throw "EPS core needs an internal pocket accessible through fill face";
				var kind = CoreSupports.kind(options, cb, b),
					bridge = kind == CoreSupportKind.Bridge;
				var foam:MoldBox = {
					x: cb.x + l,
					y: cb.y + l,
					z: cb.z + (bridge ? l : 0),
					width: cb.width - 2 * l,
					depth: cb.depth - 2 * l,
					height: cb.height - l - (bridge ? options.adhesiveThickness : 0)
				};
				if (foam.width <= 0 || foam.depth <= 0 || foam.height <= 0 || foam.width > r.core.width || foam.depth > r.core.depth
					|| foam.height > r.core.height)
					throw "EPS core exceeds stock";
				cores.push({
					id: materia.tooling.ToolingIds.derive("core-", f.id),
					featureId: f.id,
					cavity: cb,
					foam: foam,
					materialSpec: r.core.materialSpec,
					coveringThickness: l,
					removal: bridge ? BreakoutThroughFill : BreakoutThroughBed,
					support: {
						kind: kind,
						adhesiveSpec: r.adhesiveSpec,
						adhesiveThickness: options.adhesiveThickness,
						fitClearance: options.fitClearance,
						embedDepth: 0,
						bondAreaMm2: 0,
						socketOperationId: null,
						foot: null,
						panelIds: [],
						bondedPanelIds: [],
						hardware: [],
						padClosure: bridge ? PanelClosure.ExternalClamps : null
					},
					cutting: CoreSupports.cutting(foam, r.coreWireAxis),
					restraintEvidence: []
				});
			}
		if (cores.length != 1 || d.input.inserts.length != 1)
			throw "Single-pocket template needs one core and one insert";
		CoreSupports.build(cores[0], b, r, options, assembly.members[0], supportPanels, penetrations);
		var insert = d.input.inserts[0],
			dimensions = InsertBodies.box(insert.body, insert.id),
			ip = F.compose(d.setup.sourceToMold, insert.pose),
			ib = box(dimensions.width, dimensions.depth, dimensions.height, ip),
			face = F.compose(ip, F.translation(0, 0, dimensions.height)),
			normal = F.transformVector(face, 0, 0, 1);
		if (Math.abs(face.z - b.z) > 1e-7
			|| normal.z > -1 + 1e-8
			|| Math.abs(volume(clipped(ib, b)) - volume(ib)) > 1e-5)
			throw "Insert mounting face must seat on bed with fixing bore";
		var fixings:Array<materia.tooling.casting.CastingMoldPlan.InsertFixtureFixing> = [];
		for (hole in insert.holes) {
			var selected = InsertFixturePlanner.select(hole, ip, face, r.hardware);
			if (selected == null)
				continue;
			var entry = selected.entry;
			var thread:materia.casting.InsertHole.InsertThread = cast hole.thread;
			var diameter = selected.clearanceDiameter;
			var id = materia.tooling.ToolingIds.derive("insert-fixing-", hole.id);
			var operation = PanelJoints.operation(id, Drill,
				PanelContours.circle(entry.x - ox, entry.y - oy, diameter / 2), t, diameter);
			operation.through = true;
			bed.operations.push(operation);
			bed.contactOpenings.push(operation.id);
			penetrations.push({panelId: bed.id, operationId: operation.id, kind: InsertSealed,
				insertId: insert.id, sourceHoleId: hole.id, coreId: null, sealMargin: r.hardware.sealMargin,
				sealMaterial: r.hardware.sealMaterial, sealingFace: face,
				plug: selected.plug});
			fixings.push({sourceHoleId: hole.id, hardwareId: id, panelId: bed.id, operationId: id,
				entry: entry, engagement: cast materia.casting.InsertHole.InsertHoleCodec.threadLimits(hole).minEngagement, threadSpec: thread.designation});
		}
		assembly.members[0].panel = PanelCodec.snapshot(bed);
		var result:MoldPlan = {
			id: "single-pocket-box",
			document: d,
			documentFingerprint: ToolingCodec.documentFingerprint(d),
			changeRequests: [],
			cavity: b,
			panels: assembly,
			supportOptions: options,
			supportPanels: supportPanels,
			cores: cores,
			fixtures: [
				{
					insertId: insert.id,
					body: ib,
					mountingFace: face,
					panelId: bed.id,
					fixings: fixings,
					boreMasks: [for (hole in insert.holes) if (fixings.filter(f -> f.sourceHoleId == hole.id).length == 0) {
						var entry = F.compose(F.compose(d.setup.sourceToMold, insert.pose), hole.frame), axis = F.transformVector(entry, 0, 0, 1);
						InsertFixturePlanner.mask(hole, materia.tooling.ToolingIds.derive("mask-", hole.id), r.hardware.plugDepth,
							{x: -axis.x, y: -axis.y, z: -axis.z});
					}],
					threadProtection: FixingScrewAndResinSidePlug,
					finishingAllowance: null,
					datumEvidence: [],
					datums: datums(face, dimensions.width, dimensions.depth),
					precision: {
						insertId: insert.id,
						route: CastingMoldPlanner.sourceRoute(insert.finish),
						allowance: null,
						masterReference: null,
						masterFlatnessMm: null,
						releaseSpec: null,
						holeProtectionReference: null,
						evidence: []
					}
				}
			],
			penetrations: penetrations,
			validation: [
				materia.tooling.ToolingValidation.ToolingReadiness.pending("core-restraint", [cores[0].id],
					"Adhesive, pad attachment and uplift capacity require qualification"),
				materia.tooling.ToolingValidation.ToolingReadiness.pending("insert-datums", [insert.id],
					"Independent datum accuracy and thread engagement require qualification"),
				materia.tooling.ToolingValidation.ToolingReadiness.pending("anchorage-finishing", [insert.id],
					"Anchorage and precision finishing require evidence"),
				materia.tooling.ToolingValidation.ToolingReadiness.pending("materials-process", [d.recipe.id],
					"Seals, resin/EPS compatibility and cure require evidence"),
				materia.tooling.ToolingValidation.ToolingReadiness.pending("loads-handling", [d.input.id],
					"Compaction, closure loads and handling require qualification")
			],
			fabrication: {
				ready: false,
				blockingRules: []
			}
		};
		if (fixings.length == 0) {
			var rule = "tooling.fixture.source-hole." + insert.id;
			result.changeRequests.push({id: materia.tooling.ToolingIds.derive("fixture-hole-", insert.id),
				kind: materia.tooling.casting.CastingMoldPlan.DesignChangeKind.InsertFixtureHole,
				sourceIds: [insert.id], reason: "Declare an accessible source steel thread facing the bed.", blockingChecks: [rule]});
			result.validation.push(materia.tooling.ToolingValidation.ToolingReadiness.pending(rule, [insert.id],
				"Fixture blocked: no usable source threaded hole."));
		}
		if (F.transformVector(d.setup.sourceToMold, 0, 0, 1).z < 0)
			result.changeRequests.push({
				id: "lifting-flip",
				kind: materia.tooling.casting.CastingMoldPlan.DesignChangeKind.LiftingPoints,
				sourceIds: [d.input.id],
				reason: "Provide qualified lifting points and handling for the inverted casting",
				blockingChecks: ["loads-handling"]
			});
		result.validation = result.validation.concat(InsertFixturePlanner.protectionValidation(insert));
		result.validation = result.validation.concat(InsertQualifications.check(d.input.inserts)).concat(InterfaceQualification.source(d.input));
		result.fabrication = materia.tooling.ToolingValidation.ToolingReadiness.evaluate(result.validation);
		validate(result);
		return result;
	}

	public static function datums(face:AssemblyFrame, width:Float, depth:Float):materia.tooling.casting.MoldPlan.InsertDatums
		return {
			primarySupportPoints: [
				F.transformPoint(face, -width / 4, -depth / 4, 0),
				F.transformPoint(face, width / 4, -depth / 4, 0),
				F.transformPoint(face, 0, depth / 4, 0)
			],
			secondaryAxis: F.transformVector(face, 1, 0, 0),
			tertiaryAxis: F.transformVector(face, 0, 1, 0)
		};

	public static function validateInsertSeal(p:MoldPlan, record:PenetrationPlan):Void {
		var fixture = p.fixtures.filter(f -> f.insertId == record.insertId)[0],
			i = p.document.input.inserts.filter(i -> i.id == record.insertId)[0];
		var dimensions = InsertBodies.box(i.body, i.id);
		var member = p.panels.members.filter(m -> m.panel.id == record.panelId)[0],
			op = member.panel.operations.filter(o -> o.id == record.operationId)[0];
		if (record.sealingFace == null || ToolingCodec.canonical(record.sealingFace) != ToolingCodec.canonical(fixture.mountingFace))
			throw "Insert sealing face differs from fixture";
		var holes = i.holes.filter(h -> h.id == record.sourceHoleId);
		if (holes.length != 1)
			throw "Insert penetration has no unique source hole";
		var hole = holes[0], plug = record.plug;
		var thread:materia.casting.InsertHole.InsertThread = cast hole.thread;
		if (hole.extent == materia.casting.InsertHole.HoleExtent.Through) {
			if (plug == null || plug.diameter != thread.majorDiameter || plug.length <= 0
				|| plug.length + materia.casting.InsertHole.InsertHoleCodec.threadLimits(hole).minEngagement + materia.casting.InsertHole.InsertHoleCodec.threadLimits(hole).minTipGap > hole.depth + 1e-7
				|| StringTools.trim(plug.materialSpec).length == 0)
				throw "Invalid insert bore plug";
		} else if (plug != null)
			throw "Blind fixing hole needs no resin-side plug";
		if (ToolingCodec.canonical(fixture.datums) != ToolingCodec.canonical(datums(fixture.mountingFace, dimensions.width, dimensions.depth)))
			throw "Invalid insert fixture datums";
		var inverse = F.inverse(fixture.mountingFace),
			margin = record.sealMargin;
		if (record.panelId != fixture.panelId || fixture.fixings.filter(f -> f.operationId == record.operationId && f.sourceHoleId == hole.id).length != 1 || member.panel.id != "bed")
			throw "Insert seal references wrong fixing hole";
		var transform = F.compose(inverse, member.pose),
			z = member.panel.stock.measuredThickness;
		var ux = F.transformVector(transform, 1, 0, 0),
			uy = F.transformVector(transform, 0, 1, 0),
			sign = ux.x * uy.y - ux.y * uy.x;
		if (Math.abs(Math.abs(sign) - 1) > 1e-7)
			throw "Insert sealing face not parallel to bed";
		var transformed = [
			for (seg in op.contour) {
				var s = F.transformPoint(transform, seg.start.x, seg.start.y, z), e = F.transformPoint(transform, seg.end.x, seg.end.y, z), center = seg.center;
				var lc = center == null ? null : F.transformPoint(transform, center.x, center.y, z);
				if (Math.abs(s.z) > 1e-7 || Math.abs(e.z) > 1e-7) throw "Insert sealing face misses opening";
				{
					kind: seg.kind,
					start: {
						x: s.x,
						y: s.y
					},
					end: {x: e.x, y: e.y},
					center: lc == null ? null : {x: lc.x, y: lc.y},
					sweep: sign * seg.sweep
				};
			}
		];
		var bounds = materia.tooling.panel.PanelGeometry.bounds(transformed);
		if (bounds.x < -dimensions.width / 2 + margin
			|| bounds.y < -dimensions.depth / 2 + margin
			|| bounds.x + bounds.width > dimensions.width / 2 - margin
			|| bounds.y + bounds.height > dimensions.depth / 2 - margin)
			throw "Insert sealing margin exceeded";
		if (!op.through || Math.abs(fixture.mountingFace.z - p.cavity.z) > 1e-7)
			throw "Insert seal face misses bed";
	}

	public static function validate(plan:MoldPlan):Void {
		InterfaceQualification.validate(plan.document.input,plan.validation);
		if (ToolingCodec.canonical(plan.fabrication) != ToolingCodec.canonical(materia.tooling.ToolingValidation.ToolingReadiness.evaluate(plan.validation)))
			throw "Stale fabrication readiness";
		if (Math.abs(plan.document.setup.fillHeightMm - plan.cavity.z - plan.cavity.height) > 1e-7 || plan.document.setup.freeboardMm != 0)
			throw "Box template needs full-height fill and zero freeboard";
		if (ToolingCodec.documentFingerprint(plan.document) != plan.documentFingerprint)
			throw "Stale mold document";
		materia.tooling.panel.PanelAssemblyPlan.validate(plan.panels);
		if (plan.cores.length != 1 || plan.fixtures.length != 1 || plan.panels.members.length != 5)
			throw "Incomplete single-pocket mold";
		CoreSupports.options(plan.supportOptions);
		if (plan.supportPanels.length > 0)
			materia.tooling.panel.PanelAssemblyPlan.validate({
				id: "core-support",
				members: plan.supportPanels,
				joints: []
			});
		for (c in plan.cores) {
			var b = c.cutting.block, f = c.foam, pose = c.cutting.stockToMold, v = c.cutting.wireDirection;
			for (size in [b.width, b.depth, b.height, f.width, f.depth, f.height])
				if (!Math.isFinite(size) || size <= 0)
					throw "Invalid core block";
			if (b.x != 0
				|| b.y != 0
				|| b.z != 0
				|| Math.abs(b.width - f.width) + Math.abs(b.depth - f.depth) + Math.abs(b.height - f.height) > 1e-7
				|| Math.abs(pose.x - f.x) + Math.abs(pose.y - f.y) + Math.abs(pose.z - f.z) > 1e-7
				|| Math.abs(pose.qx) + Math.abs(pose.qy) + Math.abs(pose.qz) > 1e-8
				|| Math.abs(Math.abs(pose.qw) - 1) > 1e-8
				|| false)
				throw "Invalid ruled core cutting definition";
			CoreSupports.validateCutting(c.cutting, c.foam, plan.document.recipe);
			CoreSupports.validateFoot(c, plan.document.recipe, plan.supportOptions, plan.cavity);
			var support = c.support;
			if (support.kind == CoreSupportKind.Bridge) {
				var carrier = plan.supportPanels.filter(m -> m.panel.id == materia.tooling.ToolingIds.derive("", c.id, "-carrier"))[0];
				for (pad in plan.supportPanels)
					if (StringTools.endsWith(pad.panel.id, "-pad")) {
						var a = materia.tooling.panel.PanelGeometry.bounds(materia.tooling.panel.PanelContours.outside(carrier.panel));
						var b = materia.tooling.panel.PanelGeometry.bounds(materia.tooling.panel.PanelContours.outside(pad.panel));
						if (Math.min(carrier.pose.x + a.x + a.width, pad.pose.x + b.x + b.width) > Math.max(carrier.pose.x + a.x, pad.pose.x + b.x) + 1e-7
							&& Math.min(carrier.pose.y + a.y + a.height, pad.pose.y + b.y + b.height) > Math.max(carrier.pose.y + a.y, pad.pose.y + b.y) + 1e-7)
							throw "Core carrier overlaps bridge pad: " + carrier.panel.id + "/" + pad.panel.id;
						if (Math.min(plan.cavity.x + plan.cavity.width, pad.pose.x + b.x + b.width) > Math.max(plan.cavity.x, pad.pose.x + b.x) + 1e-7
							&& Math.min(plan.cavity.y + plan.cavity.depth, pad.pose.y + b.y + b.height) > Math.max(plan.cavity.y, pad.pose.y + b.y) + 1e-7)
							throw "Bridge pad covers cavity fill opening: " + pad.panel.id;
					}
			}
			if (support.kind != CoreSupportKind.Bridge && support.kind != CoreSupportKind.BedSocket)
				throw "Invalid core support kind";
			if (CoreSupports.kind(plan.supportOptions, c.cavity, plan.cavity) != support.kind)
				throw "Core support disagrees with opening";
			if (!Math.isFinite(support.adhesiveThickness)
				|| support.adhesiveThickness <= 0
				|| !Math.isFinite(support.bondAreaMm2)
				|| support.bondAreaMm2 <= 0)
				throw "Invalid core bond dimensions";
			for (id in support.panelIds)
				if (plan.supportPanels.filter(m -> m.panel.id == id).length != 1)
					throw "Missing support panel";
			for (id in support.bondedPanelIds)
				if (support.panelIds.indexOf(id) < 0)
					throw "Unknown bonded support panel";
			if (support.kind == CoreSupportKind.Bridge
				&& (support.bondedPanelIds.length != 2 || support.panelIds.length != 4 || support.foot != null || support.socketOperationId != null))
				throw "Incomplete bridge";
			if (support.kind == CoreSupportKind.BedSocket
				&& (support.bondedPanelIds.length != 0 || support.panelIds.length != 0 || support.foot == null || support.socketOperationId == null))
				throw "Incomplete bed socket";
			var l = c.coveringThickness, cb = c.cavity, bridge = support.kind == CoreSupportKind.Bridge;
			if (!Math.isFinite(l)
				|| l < 0
				|| Math.abs(f.x - cb.x - l)
				+ Math.abs(f.y - cb.y - l)
				+ Math.abs(f.z - cb.z - (bridge ? l : 0))
				+ Math.abs(f.width + 2 * l - cb.width)
				+ Math.abs(f.depth + 2 * l - cb.depth)
				+ Math.abs(f.height + l + (bridge ? support.adhesiveThickness : 0) - cb.height) > 1e-7)
				throw "Core covering does not reproduce cavity";
		}
		for (fixture in plan.fixtures) {
			var insert = plan.document.input.inserts.filter(i -> i.id == fixture.insertId)[0];
			var seenHoles = new Map<String, Bool>();
			if (fixture.fixings.length + fixture.boreMasks.length != insert.holes.length)
				throw "Every source hole needs a fixing screw or bore mask";
			for (hole in insert.holes) {
				var fixingCount = fixture.fixings.filter(f -> f.sourceHoleId == hole.id).length;
				var maskCount = fixture.boreMasks.filter(m -> m.sourceHoleId == hole.id).length;
				if (fixingCount + maskCount != 1)
					throw "Source hole protection must be unique";
			}
			for (mask in fixture.boreMasks) {
				var holes = insert.holes.filter(h -> h.id == mask.sourceHoleId);
				if (holes.length != 1)
					throw "Bore mask requires a unique source hole";
				var hole = holes[0];
				var entry = F.compose(F.compose(plan.document.setup.sourceToMold, insert.pose), hole.frame);
				var axis = F.transformVector(entry, 0, 0, 1);
				var expectedMask = InsertFixturePlanner.mask(hole, materia.tooling.ToolingIds.derive("mask-", hole.id),
					plan.document.recipe.hardware.plugDepth, {x: -axis.x, y: -axis.y, z: -axis.z});
				if (ToolingCodec.canonical(mask) != ToolingCodec.canonical(expectedMask))
					throw "Bore mask differs from source hole depth budget";
			}
			for (fixing in fixture.fixings) {
				var holes = insert.holes.filter(h -> h.id == fixing.sourceHoleId);
				if (holes.length != 1 || holes[0].thread == null || seenHoles.exists(fixing.sourceHoleId))
					throw "Fixing requires a unique source steel thread";
				seenHoles.set(fixing.sourceHoleId, true);
				var hole = holes[0], thread:materia.casting.InsertHole.InsertThread = cast hole.thread;
				var entry = F.compose(F.compose(plan.document.setup.sourceToMold, insert.pose), hole.frame);
				var panels = plan.panels.members.filter(member -> member.panel.id == fixing.panelId);
				if (panels.length != 1)
					throw "Fixing clearance references unknown panel";
				var panel = panels[0], operations = panel.panel.operations.filter(op -> op.id == fixing.operationId);
				var expected = PanelContours.circle(entry.x - panel.pose.x, entry.y - panel.pose.y,
					hole.clearanceDiameter / 2);
				if (operations.length != 1 || !operations[0].through || operations[0].kind != Drill
					|| ToolingCodec.canonical(operations[0].contour) != ToolingCodec.canonical(expected))
					throw "Fixing needs a source-sized clearance bore";
				if (fixing.engagement != materia.casting.InsertHole.InsertHoleCodec.threadLimits(hole).minEngagement || fixing.threadSpec != thread.designation
					|| ToolingCodec.canonical(fixing.entry) != ToolingCodec.canonical(entry)
					|| plan.penetrations.filter(p -> p.insertId == insert.id && p.sourceHoleId == hole.id
						&& p.operationId == fixing.operationId).length != 1)
					throw "Fixture fixing differs from source hole";
			}
			if (fixture.fixings.length == 0 && plan.changeRequests.filter(request ->
				request.kind == materia.tooling.casting.CastingMoldPlan.DesignChangeKind.InsertFixtureHole
				&& request.sourceIds.indexOf(insert.id) >= 0).length != 1)
				throw "Fixture without source fixing needs a design change request";
		}
		var seen = new Map<String, Bool>();
		for (p in plan.penetrations) {
			var key = p.panelId + "/" + p.operationId;
			if (seen.exists(key))
				throw "Duplicate penetration";
			seen.set(key, true);
			var members = plan.panels.members.filter(m -> m.panel.id == p.panelId);
			if (members.length != 1 || members[0].panel.contactOpenings.indexOf(p.operationId) < 0)
				throw "Unknown penetration";
			if (p.kind != OutsideCasting && p.kind != InsertSealed && p.kind != CoreSealed)
				throw "Unsupported penetration class";
			if (p.kind == InsertSealed && (p.insertId == null || plan.fixtures.filter(f -> f.insertId == p.insertId).length != 1))
				throw "Unknown sealing insert";
			if (p.kind == InsertSealed)
				validateInsertSeal(plan, p);
			if (p.kind == CoreSealed
				&& (p.coreId == null
					|| plan.cores.filter(c -> c.id == p.coreId && c.support.socketOperationId == p.operationId).length != 1))
				throw "Unknown sealing core";
			if (!Math.isFinite(p.sealMargin) || p.sealMargin < 0 || StringTools.trim(p.sealMaterial).length == 0)
				throw "Invalid penetration seal";
		}
		for (m in plan.panels.members)
			for (id in m.panel.contactOpenings)
				if (!seen.exists(m.panel.id + "/" + id))
					throw "Unclassified contact opening " + m.panel.id + "/" + id;
	}
}
