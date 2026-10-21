package materia.tooling.casting;

import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;
import materia.casting.BoxAlgebra;

import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.casting.MoldPanels.RaisedWalls;
import materia.tooling.casting.CastingSetup.CoreStrategy;
import materia.casting.CastingFeature.CastingFeatureKind;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelGeometry;
import materia.tooling.ToolingEnums.MachiningSide;

/** Source assessment can decline geometry without manufacturing a box stand-in. */
typedef CastingMoldAssessment = {
	var sourceFingerprint:String;
	var plan:Null<CastingMoldPlan>;
	var validation:Array<materia.tooling.ToolingValidation>;
	var changeRequests:Array<DesignChangeRequest>;
	var fabrication:materia.tooling.ToolingValidation.FabricationReadiness;
}

/** Orthogonal review template for multiple explicit EPS/former strategies.
 * All casting dimensions come from the source document. It never adds design cuts.
 */
class CastingMoldPlanner {
	public static function sourceSupport(input:CastingToolingInput):Array<materia.tooling.ToolingValidation>
		return CastingBoxSupport.check(input, "tooling.feature.box-geometry", false).concat(InsertQualifications.check(input.inserts));
	public static function envelope(d:ToolingDocument):MoldBox {
		var rows = d.input.envelope;
		if (rows.length != 1 || rows[0].profile.length != 4)
			throw "Multi-core box template needs one rectangular envelope";
		var e = rows[0], xs = [for (p in e.profile) p.x], ys = [for (p in e.profile) p.y];
		xs.sort(Reflect.compare);
		ys.sort(Reflect.compare);
		for (p in e.profile)
			if ((p.x != xs[0] && p.x != xs[3]) || (p.y != ys[0] && p.y != ys[3]))
				throw "Nonrectangular envelope unsupported by box template";
		return BoxMoldPlanner.box(xs[3] - xs[0], ys[3] - ys[0], e.height,
			F.compose(d.setup.sourceToMold, F.compose(e.pose, F.translation((xs[0] + xs[3]) / 2, (ys[0] + ys[3]) / 2, 0))));
	}

	public static function featureBox(d:ToolingDocument, f:materia.casting.CastingFeature):MoldBox {
		var cavities = materia.casting.CastingCavities.derive(d.input.envelope, d.input.features);
		var cavity = cavities.filter(c -> c.featureId == f.id)[0];
		var boxes = materia.casting.CastingCavities.boxes(cavity);
		if (boxes.length != 1) throw "Unsupported non-box finished cavity: " + f.id;
		var box = boxes[0];
		return BoxMoldPlanner.box(box.width, box.depth, box.height,
			F.compose(d.setup.sourceToMold, F.translation(box.x + box.width / 2, box.y + box.depth / 2, box.z)));
	}

	public static function assess(document:ToolingDocument, authored:CastingMoldOptions):CastingMoldAssessment {
		var d = ToolingCodec.snapshot(document), options = CastingMoldOptionsCodec.snapshot(authored);
		var checks = sourceSupport(d.input);
		// Hardware qualifications affect readiness, not whether the source cavity
		// can be panelled. Exact hardware is realized and checked by the CAD bridge.
		var unsupported = checks.filter(check -> StringTools.startsWith(check.rule,"tooling.feature.box-geometry")
			&& check.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported);
		if (unsupported.length > 0) {
			var changes:Array<DesignChangeRequest> = [for (check in unsupported) if (d.input.features.filter(feature -> check.affectedIds.indexOf(feature.id)>=0).length>0) {
				id: materia.tooling.ToolingIds.derive("geometry-", check.affectedIds.join("-")),
				kind: DesignChangeKind.ReleaseAlternative,
				sourceIds: check.affectedIds.copy(),
				reason: check.message + ". Select supported source geometry or implement profile-capable tooling before fabrication.",
				blockingChecks: [check.rule]
			}];
			checks=checks.concat(InterfaceQualification.source(d.input)).concat(InsertQualifications.manufacturing(d.input.inserts))
				.concat(ReplicationJigs.analyze(d.input,options.replicationJigs,options.precision))
				.concat(FoundationJointQualification.analyze(d.input,options.foundationJointProcesses));
			return {sourceFingerprint: ToolingCodec.documentFingerprint(d), plan: null, validation: checks,
				changeRequests: changes, fabrication: materia.tooling.ToolingValidation.ToolingReadiness.evaluate(checks)};
		}
		var result = build(d, options, checks);
		return {sourceFingerprint: result.sourceFingerprint, plan: result, validation: result.validation.copy(),
			changeRequests: result.changeRequests.copy(), fabrication: result.fabrication};
	}

	public static function plan(document:ToolingDocument, authored:CastingMoldOptions):CastingMoldPlan {
		var assessment = assess(document, authored);
		if (assessment.plan == null)
			throw [for (check in assessment.validation) check.message].join("\n");
		return cast assessment.plan;
	}

	static function build(d:ToolingDocument, o:CastingMoldOptions, geometryChecks:Array<materia.tooling.ToolingValidation>):CastingMoldPlan {
		var e = envelope(d), r = d.recipe, t = r.panel.measuredThickness, l = r.minimumLigament;
		if (o.precision.length != d.input.inserts.length)
			throw "Every insert needs a precision process record";
		var strategies = CastingStrategies.resolve(d);
		var reusable = strategies.filter(v -> v.strategy == CoreStrategy.ReusableFormer);
		for (index in 0...reusable.length)
			for (other in index + 1...reusable.length) {
				var a = reusable[index], b = reusable[other];
				if (a.priority == b.priority
					&& BoxAlgebra.intersection(featureBox(d, d.input.features.filter(f -> f.id == a.featureId)[0]),
						featureBox(d, d.input.features.filter(f -> f.id == b.featureId)[0])) != null)
					throw "Overlapping reusable formers require distinct priorities: " + a.featureId + ", " + b.featureId;
			}
		var bodies = new Map<String, MoldBox>(), low = e.z;
		for (i in d.input.inserts) {
			var dimensions = InsertBodies.box(i.body, i.id);
			var b = BoxMoldPlanner.box(dimensions.width, dimensions.depth, dimensions.height, F.compose(d.setup.sourceToMold, i.pose));
			bodies.set(i.id, b);
			low = Math.min(low, b.z);
			if (o.precision.filter(p -> p.insertId == i.id).length != 1)
				throw "Missing or duplicate insert process record";
		}
		var b:MoldBox = {
			x: e.x,
			y: e.y,
			z: low,
			width: e.width,
			depth: e.depth,
			height: e.z + e.height - low
		};
		var shell = MoldPanels.box(b, r, RaisedWalls.EastWest),
			panels = shell.panels,
			bed = panels.members[0];
		var openings:Array<MoldOpening> = [
			for (p in shell.penetrations)
				{
					panelId: p.panelId,
					operationId: p.operationId,
					kind: ExteriorOpening,
					sealingIds: [],
					sealSpec: p.sealMaterial
				}
		];
		var result:CastingMoldPlan = {
			handling: CastHandling.plan(d, o, e),
			id: materia.tooling.ToolingIds.derive("", d.input.id, "-mold"),
			document: d,
			sourceFingerprint: ToolingCodec.documentFingerprint(d),
			options: o,
			envelope: e,
			cavity: b,
			panels: panels,
			supportPanels: [],
			cores: [],
			coreUnits: [],
			formers: [],
			fixtures: [],
			features: [],
			openings: openings,
			hardware: [],
			changeRequests: [],
			validation: geometryChecks.concat(InterfaceQualification.source(d.input)).concat(InsertQualifications.manufacturing(d.input.inserts))
				.concat(ReplicationJigs.analyze(d.input,o.replicationJigs,o.precision))
				.concat(FoundationJointQualification.analyze(d.input,o.foundationJointProcesses)),
			fabrication: {
				ready: false,
				blockingRules: []
			}
		};
		var epsBoxes:Array<MoldBox> = [], insertBoxes:Array<MoldBox> = [];
		for (i in d.input.inserts) {
			var ib = bodies.get(i.id);
			if (ib == null)
				throw "Missing insert body";
			insertBoxes.push(ib);
		}
		for (f in d.input.features)
			if (f.kind != CastingFeatureKind.InsertRecess) {
				var s = strategies.filter(s -> s.featureId == f.id)[0],
					cb = featureBox(d, f);
				if (cb == null)
					throw "Feature misses envelope: " + f.id;
				if (s.strategy == CoreStrategy.SacrificialEPS) {
					if (Math.abs(cb.z + cb.height - b.z - b.height) > 1e-7
						|| cb.x <= e.x
						|| cb.x + cb.width >= e.x + e.width
						|| cb.y <= e.y
						|| cb.y + cb.depth >= e.y + e.depth)
						throw "Shared bridge needs internal fill-open EPS pocket: " + f.id;
					if (BoxAlgebra.volume(BoxAlgebra.subtract([cb], insertBoxes.concat(epsBoxes))) < BoxMoldPlanner.volume(cb) - 1e-5)
						throw "Intersecting EPS/insert cavities need an explicit allocation";
					var g = r.adhesiveLineThickness,
						cover = r.linerThickness,
						foam:MoldBox = {
							x: cb.x + cover,
							y: cb.y + cover,
							z: cb.z + cover,
							width: cb.width - 2 * cover,
							depth: cb.depth - 2 * cover,
							height: cb.height - cover - g
						};
					if (foam.width <= 0 || foam.depth <= 0 || foam.height <= 0 || foam.width > r.core.width || foam.depth > r.core.depth)
						throw "EPS block exceeds stock";
					var layers:Array<MoldBox> = [],
						count = Std.int(Math.ceil((foam.height + g) / (r.core.height + g)));
					if (count < 1 || count > 32)
						throw "Too many EPS layers";
					var layerHeight = (foam.height - g * (count - 1)) / count;
					if (layerHeight <= 0)
						throw "EPS adhesive consumes layer height";
					for (index in 0...count)
						layers.push({
							x: foam.x,
							y: foam.y,
							z: foam.z + index * (layerHeight + g),
							width: foam.width,
							depth: foam.depth,
							height: layerHeight
						});
					var id = materia.tooling.ToolingIds.derive("core-", f.id);
					result.cores.push({
						id: id,
						featureId: f.id,
						cavity: cb,
						foamLayers: layers,
						coveringThickness: cover,
						adhesiveThickness: g,
						carrierId: materia.tooling.ToolingIds.derive("", id, "-carrier"),
						bridgeId: "pending",
						bondAreaMm2: foam.width * foam.depth,
						bridgeBondAreaMm2: 0
					});
					result.features.push({featureId: f.id, kind: EPSFeature, memberIds: [id]});
					epsBoxes.push(cb);
				} else if (s.strategy == CoreStrategy.Unsupported) {
					result.features.push({featureId: f.id, kind: UnsupportedFeature, memberIds: []});
					request(result, "unsupported-" + f.id, ReleaseAlternative, [f.id],
						"This planner supports only orthogonal zero-draft rigid formers or fill-open EPS. Drafted/split formers are unsupported; choose supported EPS or revise the source to a supported geometry.",
						"tooling.feature.unsupported." + f.id, true);
				}
			}
		result.coreUnits = coreUnits(d.input, result.cores);
		CoreSupports.sharedBridges(result.cavity, r, result.cores, result.supportPanels, result.hardware);
		// Priority is explicit: inserts, EPS and datum supports are reserved first;
		// rigid formers then resolve by descending authored priority and sorted IDs.
		var ordered = ownershipOrder(d.input.features, strategies);
		// Datum fixtures occupy negative space only; neutral validation checks their resin clearance.
		for (i in d.input.inserts) {
			var dimensions = InsertBodies.box(i.body, i.id);
			var pose = F.compose(d.setup.sourceToMold, i.pose),
				face = F.compose(pose, F.translation(0, 0, dimensions.height)),
				normal = F.transformVector(face, 0, 0, 1),
				body = bodies.get(i.id);
			if (body == null)
				throw "Missing insert body";
			if (normal.z > -1 + 1e-8)
				throw "Box datum fixtures currently require downward mounting faces";
			var dt = r.hardware.datumThickness, pw = r.hardware.datumPostWidth;
			var pocketId:Null<String> = null;
			var pad:MoldBox = {
				x: body.x,
				y: body.y,
				z: face.z - dt,
				width: body.width,
				depth: body.depth,
				height: dt
			}, pieces = [pad], withdrawal = {
				x: 0.0,
				y: 0.0,
				z: -1.0
			};
			var access = ordered.filter(f -> f.kind != CastingFeatureKind.InsertRecess
				&& strategies.filter(s -> s.featureId == f.id
					&& s.strategy == CoreStrategy.ReusableFormer
					&& s.withdrawal != null
					&& Math.abs(s.withdrawal.x) > 0.9)
					.length == 1 && BoxAlgebra.intersection(featureBox(d, f), pad) != null);
			if (access.length > 0) {
				var dir = strategies.filter(s -> s.featureId == access[0].id)[0].withdrawal;
				if (dir == null)
					throw "Missing side fixture direction";
				withdrawal = {x: dir.x, y: dir.y, z: dir.z};
				pad = dir.x > 0 ? {
					x: body.x,
					y: body.y,
					z: face.z - dt,
					width: b.x + b.width + 3 * t - body.x,
					depth: body.depth,
					height: dt
				} : {
					x: b.x - 3 * t,
					y: body.y,
					z: face.z - dt,
					width: body.x + body.width - b.x + 3 * t,
					depth: body.depth,
					height: dt
					};
				pieces = [pad];
			} else {
				var padEntersBed=pad.z<b.z-1e-7;
				if (padEntersBed) {
					var inset = r.routerDiameter / 2 + r.minimumLigament;
					pad = {
						x: body.x + inset,
						y: body.y + inset,
						z: face.z - dt,
						width: body.width - 2 * inset,
						depth: body.depth - 2 * inset,
						height: dt
					};
					if (pad.width <= r.routerDiameter || pad.depth <= r.routerDiameter)
						throw "Insert too small for sealed relieved datum socket: " + i.id;
				}
				pieces=[pad];
				var baseZ=Math.min(pad.z,b.z-dt),height=pad.z-baseZ;
				if (height>1e-8) pieces.push({
					x:padEntersBed ? pad.x : face.x-pw/2,
					y:padEntersBed ? pad.y : face.y-pw/2,
					z:baseZ,width:padEntersBed ? pad.width : pw,
					depth:padEntersBed ? pad.depth : pw,height:height
				});
				// A pad less than one datum thickness above the bed crosses its
				// surface. Its full section, including locator support, needs a socket.
				var socket:MoldBox = padEntersBed ? {
					x: pad.x,
					y: pad.y,
					z: b.z - dt,
					width: pad.width,
					depth: pad.depth,
					height: dt
				} : {
					x: face.x - pw / 2,
					y: face.y - pw / 2,
					z: b.z - dt,
					width: pw,
					depth: pw,
					height: dt
					};
				var op = PanelJoints.operation(materia.tooling.ToolingIds.derive("datum-", i.id), Pocket,
					PanelContours.relievedRectangle(socket.x - bed.pose.x + socket.width / 2, socket.y - bed.pose.y + socket.depth / 2, socket.width,
						socket.depth, r.routerDiameter / 2),
					dt, r.routerDiameter, MachiningSide.PositiveZ, false, r.routerDiameter);
				pocketId = op.id;
				bed.panel.operations.push(op);
				bed.panel.contactOpenings.push(op.id);
				result.openings.push({
					panelId: "bed",
					operationId: op.id,
					kind: ToolingOpening,
					sealingIds: [materia.tooling.ToolingIds.derive("datum-", i.id)],
					sealSpec: r.hardware.sealMaterial
				});
			}
			var datum:MoldVolume = {
				id: materia.tooling.ToolingIds.derive("datum-", i.id),
				material: SteelStock,
				cuts: [],
				profiles: [],
				pieces: BoxAlgebra.subtract(pieces, [])
			};
			DatumFixtures.add(result, i, face, datum, withdrawal, bed, pocketId);
			result.features.push({
				featureId: d.input.features.filter(f -> f.kind == CastingFeatureKind.InsertRecess && f.owner == i.id)[0].id,
				kind: InsertFeature,
				memberIds: [materia.tooling.ToolingIds.derive("insert-", i.id), datum.id]
			});
			result.validation.push(materia.tooling.ToolingValidation.ToolingReadiness.pending("tooling.fixture.attachment." + i.id, [i.id],
				"Bed pins register the nominal support only. Qualify insert XY location/clamping, datum accuracy and release. Screws use declared source threads only."));
		}
		var fixtures:Array<MoldBox> = [];
		for (f in result.fixtures)
			for (p in f.datum.pieces)
				fixtures.push(p);
		var occupied = insertBoxes.concat(epsBoxes).concat(fixtures),
			previous:Array<MoldBox> = [];
		var backing = BoxAlgebra.subtract([b], [e].concat(occupied));
		for (group in BoxAlgebra.groups(backing)) {
			var id = "bed-backing-" + result.formers.length;
			result.formers.push({
				volume: {
					id: id,
					material: FormerStock,
					cuts: [],
					profiles: [],
					pieces: group
				},
				draftDegrees: 0,
				featureIds: [],
				withdrawal: {
					x: 0,
					y: 0,
					z: -1
				}
			});
		}
		for (f in ordered)
			if (f.kind != CastingFeatureKind.InsertRecess) {
				var s = strategies.filter(s -> s.featureId == f.id)[0];
				if (s.strategy != CoreStrategy.ReusableFormer || f.draft.degrees > 0)
					continue;
				var cb = featureBox(d, f);
				if (cb == null)
					throw "Missing former intersection";
				var direction = s.withdrawal;
				if (direction == null)
					throw "Former needs a withdrawal";
				var add = [cb];
				var profiles:Array<materia.tooling.casting.CastingMoldPlan.MoldVolumeProfile> = [];
				if (Math.abs(direction.x) > 0.9) {
					var side = direction.x > 0 ? e.x + e.width : e.x,
						wall = panels.members.filter(m -> m.panel.id == (direction.x > 0 ? "east" : "west"))[0];
					var extended = BoxAlgebra.copy(cb);
					if (direction.x > 0)
						extended.width = b.x + b.width + t - cb.x;
					else {
						extended.x = b.x - t;
						extended.width = cb.x + cb.width - extended.x;
					}
					add = [extended];
					var flange:MoldBox = {
						x: direction.x > 0 ? side + t : side - t - r.hardware.formerFlangeThickness,
						y: cb.y - r.hardware.formerFlangeMargin,
						z: cb.z - r.hardware.formerFlangeMargin,
						width: r.hardware.formerFlangeThickness,
						depth: cb.depth + 2 * r.hardware.formerFlangeMargin,
						height: cb.height + 2 * r.hardware.formerFlangeMargin
					};
					add.push(flange);
					var inverse = F.inverse(wall.pose),
						a = F.transformPoint(inverse, side, cb.y, cb.z),
						c = F.transformPoint(inverse, side, cb.y + cb.depth, cb.z + cb.height);
					var x = Math.min(a.x, c.x), y = Math.min(a.y, c.y);
					var op = PanelJoints.operation(materia.tooling.ToolingIds.derive("access-", f.id), InsideProfile,
						PanelContours.relievedRectangle((a.x + c.x) / 2, (a.y + c.y) / 2, Math.abs(c.x - a.x), Math.abs(c.y - a.y), r.routerDiameter / 2), t,
						r.routerDiameter, MachiningSide.PositiveZ, true, r.routerDiameter);
					profiles.push({
						panelId: wall.panel.id,
						operationId: op.id,
						contour: op.contour,
						pose: wall.pose,
						depth: t,
						subtract: occupied.concat(previous)
					});
					wall.panel.operations.push(op);
					wall.panel.contactOpenings = [for (id in wall.panel.contactOpenings) id].concat([op.id]);
					result.openings.push({
						panelId: wall.panel.id,
						operationId: op.id,
						kind: ToolingOpening,
						sealingIds: [],
						sealSpec: r.hardware.sealMaterial
					});
				}
				var pieces = BoxAlgebra.subtract(add, occupied.concat(previous)),
					ids:Array<String> = [],
					groups = BoxAlgebra.groups(pieces);
				if (profiles.length > 0 && groups.length != 1)
					throw "Access former needs an explicit split for its sealing profile: " + f.id;
				for (index in 0...groups.length) {
					var id = materia.tooling.ToolingIds.derive("former-", f.id, "-" + index);
					ids.push(id);
					result.formers.push({
						volume: {
							id: id,
							material: FormerStock,
							cuts: [],
							profiles: profiles,
							pieces: groups[index]
						},
						draftDegrees: f.draft.degrees,
						featureIds: [f.id],
						withdrawal: {
							x: direction.x,
							y: direction.y,
							z: direction.z
						}
					});
				}
				if (ids.length == 0) {
					// A feature wholly consumed by earlier tooling still keeps explicit ownership.
					ids = [
						for (former in result.formers)
							if (former.volume.pieces.filter(p -> BoxAlgebra.intersection(p, cb) != null).length > 0) former.volume.id
					];
					// Final ownership below also includes reserved EPS, inserts and supports.
				}
				result.features.push({featureId: f.id, kind: FormerFeature, memberIds: ids});
				for (p in pieces)
					previous.push(p);
				for (open in result.openings)
					if (open.operationId == materia.tooling.ToolingIds.derive("access-", f.id)) {
						open.sealingIds = ids.copy();
						for (fixture in result.fixtures)
							if (fixture.datum.pieces.filter(p -> BoxAlgebra.intersection(p, cb) != null).length > 0)
								open.sealingIds.push(fixture.datum.id);
					}
			}
		for (m in panels.members)
			m.panel = PanelCodec.snapshot(m.panel);
		for (feature in result.features)
			if (feature.kind == FormerFeature) {
				var source = d.input.features.filter(f -> f.id == feature.featureId)[0],
					cb = featureBox(d, source);
				if (cb == null)
					throw "Missing cavity in ownership allocation";
				feature.memberIds = [];
				for (former in result.formers)
					if (former.volume.pieces.filter(v -> BoxAlgebra.intersection(v, cb) != null).length > 0) {
						feature.memberIds.push(former.volume.id);
						if (former.featureIds.indexOf(source.id) < 0)
							former.featureIds.push(source.id);
					}
				for (core in result.cores)
					if (BoxAlgebra.intersection(core.cavity, cb) != null)
						feature.memberIds.push(core.id);
				for (fixture in result.fixtures)
					if (fixture.datum.pieces.filter(v -> BoxAlgebra.intersection(v, cb) != null).length > 0)
						feature.memberIds.push(fixture.datum.id);
				for (insert in d.input.inserts)
					if (BoxAlgebra.intersection(bodies.get(insert.id), cb) != null)
						feature.memberIds.push(materia.tooling.ToolingIds.derive("insert-", insert.id));
				feature.memberIds.sort(Reflect.compare);
				if (feature.memberIds.length == 0) {
					feature.kind = UnsupportedFeature;
					request(result, "unallocated-" + source.id, ReleaseAlternative, [source.id],
						"Feature has no supported physical tooling owner; choose an explicit alternative.", "tooling.feature.unsupported." + source.id, true);
				}
			}
		for (strategy in strategies)
			if (strategy.strategy == CoreStrategy.ReusableFormer && d.input.features.filter(feature -> feature.id == strategy.featureId)[0].draft.degrees == 0)
				request(result, "draft-" + strategy.featureId, ReleaseAlternative, [strategy.featureId],
					"Zero-draft rigid former has no qualified physical release. Only supported EPS or a source change to supported release geometry clears this rule; drafted/split formers are not implemented.",
					"tooling.former.draft." + strategy.featureId);
		for (former in result.formers)
			if (former.featureIds.length == 0 && former.draftDegrees == 0)
				request(result, "draft-" + former.volume.id, ReleaseAlternative, [former.volume.id],
					"Bed-backing zero-draft rigid former needs a source/release change; drafted and split formers are unsupported.",
					"tooling.former.draft." + former.volume.id);
		var handlingQualified=CastHandling.qualified(d,o,result.handling);
		if (result.handling.requiresFlip && !CastHandling.sourceDeclared(d) && !handlingQualified)
			request(result,"handling-source-points",LiftingPoints,[d.input.id],
				"Declare lifting inserts with unoccupied pre-cast threads, or a qualified external cradle.",
				"tooling.handling.lifting-points");
		else result.validation.push(materia.tooling.process.FillAnalysis.check("tooling.handling.lifting-points",
			[d.input.id].concat(CastHandling.liftingIds(d)),Pass,
			"Source lifting connections are declared, a cradle is qualified, or this setup needs no flip.",[]));
		result.validation.push(materia.tooling.process.FillAnalysis.check("tooling.handling.flip",[d.input.id],
			!result.handling.requiresFlip || handlingQualified ? Pass : Pending,
			"Equipment capacity, measured mass/CoM and the flip procedure require separate handling evidence.",[]));
		for (seat in o.precision) {
			var source = d.input.inserts.filter(i -> i.id == seat.insertId)[0];
			var stock=InterfaceQualification.finishingStock(d.input,source);
			if (stock.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass) result.validation.push(stock);
			else requestDecision(result,"allowance-"+seat.insertId,InterfaceAllowance,stock);
			var replication=InterfaceQualification.replicationRequirement(source,seat);
			if (replication!=null) result.validation.push(replication);
		}
		for (insert in d.input.inserts) switch insert.origin {
			case Fabricated(manufacturing):
				for (report in InsertManufacturingPlan.mountingHoleReports(manufacturing))
					if (report.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail) {
						var rule=InsertManufacturingPlan.mountingRule(insert.id,report.holeId);
						var finding=result.validation.filter(check -> check.rule==rule)[0];
						requestDecision(result,"hole-route-"+insert.id+"-"+report.holeId,DesignChangeKind.MountingHoleRoute,finding);
					}
			case Catalogue(_):
		}
		var missingJoint=FoundationJointQualification.missingSource(d.input);
		if (missingJoint!=null) requestDecision(result,"foundation-joint-source",DesignChangeKind.FoundationJointDefinition,missingJoint);

		result.validation = result.validation.concat([
			materia.tooling.ToolingValidation.ToolingReadiness.pending("closure-loads", [d.input.id],
				"Wall/pad clamps, braces and uplift capacity require qualification"),
			materia.tooling.ToolingValidation.ToolingReadiness.pending("insert-fixtures", [for (i in d.input.inserts) i.id],
				"Attachment, datum accuracy and thread protection require evidence"),
			materia.tooling.ToolingValidation.ToolingReadiness.pending("materials-process", [d.recipe.id],
				"Adhesive, covering, release and cure compatibility require qualification"),
			materia.tooling.ToolingValidation.ToolingReadiness.pending("precision-interfaces", [d.input.id],
				"Replication and grout interfaces require evidence"),
			materia.tooling.ToolingValidation.ToolingReadiness.pending("anchorage-compaction", [d.input.id], "Anchorage and compaction loads require evidence")
		]);
		result.fabrication = materia.tooling.ToolingValidation.ToolingReadiness.evaluate(result.validation);
		validate(result);
		return result;
	}

	/** Ownership ordering accepts raw caller order, before codec canonicalisation. */
	public static function ownershipOrder(features:Array<materia.casting.CastingFeature>,
			strategies:Array<materia.tooling.casting.CastingSetup.CastingFeatureStrategy>):Array<materia.casting.CastingFeature> {
		var ordered = features.copy();
		ordered.sort((a, b) -> {
			var aa = strategies.filter(v -> v.featureId == a.id),
				bb = strategies.filter(v -> v.featureId == b.id);
			var ap = aa.length == 0 ? 0 : aa[0].priority,
				bp = bb.length == 0 ? 0 : bb[0].priority;
			return ap == bp ? Reflect.compare(a.id, b.id) : Reflect.compare(bp, ap);
		});
		return ordered;
	}

	public static function coreUnits(input:CastingToolingInput, cores:Array<CastingMoldCore>):Array<CastingCoreUnit> {
		var groups = new Map<String, CastingCoreUnit>();
		for (core in cores) {
			var sources = input.features.filter(feature -> feature.id == core.featureId);
			if (sources.length != 1)
				throw "Core has no unique source feature: " + core.id;
			var source = sources[0], parent = source.derivedFrom == null ? source.id : source.derivedFrom;
			var group = groups.get(parent);
			if (group == null) {
				group = {id: materia.tooling.ToolingIds.derive("core-unit-", parent), parentFeatureId: parent, featureIds: [], coreIds: []};
				groups.set(parent, group);
			}
			group.featureIds.push(source.id);
			group.coreIds.push(core.id);
		}
		var result = [for (group in groups) group];
		for (group in result) {
			group.featureIds.sort(Reflect.compare);
			group.coreIds.sort(Reflect.compare);
		}
		result.sort((a, b) -> Reflect.compare(a.id, b.id));
		return result;
	}

	static function request(p:CastingMoldPlan, id:String, kind:DesignChangeKind, ids:Array<String>, message:String, rule:String,
			unsupported:Bool = false):Void {
		var check = materia.tooling.ToolingValidation.ToolingReadiness.pending(rule, ids, message);
		if (unsupported) {
			check.status = materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported;
			check.severity = materia.tooling.ToolingDiagnostic.ToolingSeverity.Error;
		}
		requestDecision(p,id,kind,check);
	}

	static function requestDecision(p:CastingMoldPlan,id:String,kind:DesignChangeKind,check:materia.tooling.ToolingValidation):Void {
		p.changeRequests.push({id:materia.tooling.ToolingIds.derive("",id),kind:kind,
			sourceIds:check.affectedIds.copy(),reason:check.message,blockingChecks:[check.rule]});
		var existing=p.validation.filter(item -> item.rule==check.rule);
		if (existing.length==0) p.validation.push(check);
		else if (existing.length!=1 || ToolingCodec.canonical(existing[0])!=ToolingCodec.canonical(check))
			throw "Conflicting design-change validation: "+check.rule;
	}

	public static function resinAtFill(document:ToolingDocument, envelope:MoldBox):Array<MoldBox> {
		var wetted = BoxAlgebra.copy(envelope);
		wetted.height = Math.min(wetted.height, document.setup.fillHeightMm - wetted.z);
		return BoxAlgebra.subtract(wetted.height > 0 ? [wetted] : [], [for (feature in document.input.features) featureBox(document, feature)]);
	}

	public static function validate(p:CastingMoldPlan):Void {
		FoundationJointQualification.validate(p.document.input,p.options.foundationJointProcesses,p.validation);
		ReplicationJigs.validate(p.document.input,p.options.replicationJigs,p.options.precision,p.validation);
		InterfaceQualification.validate(p.document.input,p.validation);
		InsertQualifications.validateManufacturing(p.document.input.inserts,p.validation);
		InsertQualifications.validateAnchorage(p.document.input.inserts,p.validation);
		if (ToolingCodec.canonical(p.fabrication) != ToolingCodec.canonical(materia.tooling.ToolingValidation.ToolingReadiness.evaluate(p.validation)))
			throw "Stale fabrication readiness";
		if (Math.abs(p.document.setup.fillHeightMm - p.cavity.z - p.cavity.height) > 1e-7 || p.document.setup.freeboardMm != 0)
			throw "Box template needs full-height fill and zero freeboard";
		if (ToolingCodec.documentFingerprint(p.document) != p.sourceFingerprint)
			throw "Stale multi-core mold source";
		materia.tooling.panel.PanelAssemblyPlan.validate(p.panels);
		if (p.supportPanels.length > 0)
			materia.tooling.panel.PanelAssemblyPlan.validate({
				id: "shared-bridges",
				members: p.supportPanels,
				joints: []
			});
		CastingMoldOptionsCodec.snapshot(p.options);
		if (ToolingCodec.canonical(p.handling) != ToolingCodec.canonical(CastHandling.plan(p.document, p.options, p.envelope)))
			throw "Stale source-derived handling plan";
		DatumFixtures.validate(p);
		var members = new Map<String, Bool>();
		function member(id:String):Void {
			materia.tooling.panel.PanelCodec.identifier(id);
			if (members.exists(id))
				throw "Duplicate mold member";
			members.set(id, true);
		}
		for (m in p.panels.members.concat(p.supportPanels))
			member(m.panel.id);
		for (c in p.cores) {
			member(c.id);
			if (c.foamLayers.length == 0 || c.foamLayers.length > 32)
				throw "Invalid EPS layers";
		}
		if (ToolingCodec.canonical(p.coreUnits) != ToolingCodec.canonical(coreUnits(p.document.input, p.cores)))
			throw "Core units disagree with source feature lineage";
		for (f in p.formers)
			member(f.volume.id);
		for (f in p.fixtures)
			member(f.datum.id);
		for (i in p.document.input.inserts)
			member(materia.tooling.ToolingIds.derive("insert-", i.id));
		var ruleIds = [for (v in p.validation) v.rule];
		for (strategy in CastingStrategies.resolve(p.document))
			if (strategy.strategy == CoreStrategy.ReusableFormer
				&& p.document.input.features.filter(feature -> feature.id == strategy.featureId)[0].draft.degrees == 0
				&& ruleIds.indexOf("tooling.former.draft." + strategy.featureId) < 0)
				throw "Zero-draft former needs a release validation";
		for (former in p.formers)
			if (former.featureIds.length == 0
				&& former.draftDegrees == 0
				&& ruleIds.indexOf("tooling.former.draft." + former.volume.id) < 0)
				throw "Zero-draft bed-backing former needs a release validation";
		for (seat in p.options.precision) {
			var source = p.document.input.inserts.filter(i -> i.id == seat.insertId)[0];
			var replication=InterfaceQualification.replicationRequirement(source,seat);
			var replicationChecks=p.validation.filter(check -> check.rule=="tooling.precision.replication."+source.id);
			if (replication==null ? replicationChecks.length!=0 : replicationChecks.length!=1
				|| ToolingCodec.canonical(replicationChecks[0])!=ToolingCodec.canonical(replication))
				throw "Replication requirement differs from source interfaces and process choices";
			var stock=InterfaceQualification.finishingStock(p.document.input,source);
			var decisions=p.validation.filter(decision -> decision.rule==stock.rule);
			if (decisions.length!=1 || ToolingCodec.canonical(decisions[0])!=ToolingCodec.canonical(stock))
				throw "Finishing stock decision differs from source geometry";
			var requests=p.changeRequests.filter(request -> request.id=="allowance-"+seat.insertId);
			if (stock.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass ? requests.length!=0
				: requests.length!=1 || requests[0].blockingChecks.indexOf(stock.rule)<0)
				throw "Finishing stock change request differs from source decision";
		}
		for (request in p.changeRequests)
			for (rule in request.blockingChecks)
				if (ruleIds.indexOf(rule) < 0)
					throw "Design change references unknown validation rule: " + rule;
		for (feature in p.features)
			if (feature.kind == UnsupportedFeature
				&& p.validation.filter(v -> v.status == "unsupported" && v.affectedIds.indexOf(feature.featureId) >= 0).length == 0)
				throw "Unsupported feature needs blocking validation";
		var ids = new Map<String, Bool>();
		for (f in p.features) {
			for (id in f.memberIds)
				if (!members.exists(id))
					throw "Unknown feature tooling member";
			if (ids.exists(f.featureId) || (f.memberIds.length == 0 && f.kind != UnsupportedFeature))
				throw "Invalid feature ownership";
			ids.set(f.featureId, true);
		}
		for (f in p.document.input.features)
			if (!ids.exists(f.id))
				throw "Unclassified source feature: " + f.id;
		if (p.features.length != p.document.input.features.length)
			throw "Unexpected feature classification";
		var seen = new Map<String, Bool>();
		for (o in p.openings) {
			var key = o.panelId + "/" + o.operationId;
			if (seen.exists(key))
				throw "Duplicate mold opening";
			seen.set(key, true);
			if (o.kind == ToolingOpening && o.sealingIds.length == 0)
				throw "Unsealed tooling opening";
			for (id in o.sealingIds)
				if (!members.exists(id))
					throw "Unknown sealing member";
		}
		for (m in p.panels.members)
			for (id in m.panel.contactOpenings)
				if (!seen.exists(m.panel.id + "/" + id))
					throw "Unclassified mold opening";
		for (f in p.formers) {
			MoldVolumeCodec.snapshot(f.volume);
			for (profile in f.volume.profiles) {
				var panels = p.panels.members.filter(m -> m.panel.id == profile.panelId);
				if (panels.length != 1)
					throw "Unknown former sealing panel";
				var operations = panels[0].panel.operations.filter(o -> o.id == profile.operationId);
				if (operations.length != 1
					|| ToolingCodec.canonical(operations[0].contour) != ToolingCodec.canonical(profile.contour)
					|| ToolingCodec.canonical(panels[0].pose) != ToolingCodec.canonical(profile.pose)
					|| profile.depth != panels[0].panel.stock.measuredThickness)
					throw "Former sealing profile differs from panel operation";
			}
		}
		var resin = resinAtFill(p.document, p.envelope);
		for (fixture in p.fixtures) {
			MoldVolumeCodec.snapshot(fixture.datum);
			if (fixture.datum.profiles.length > 0)
				throw "Neutral datum/resin validation does not support datum profiles";
			for (piece in fixture.datum.pieces)
				for (resinPiece in resin)
					if (BoxAlgebra.intersection(piece, resinPiece) != null)
						throw "Datum fixture intersects resin: " + fixture.datum.id;
		}
	}
}
