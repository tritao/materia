package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.casting.MoldPanels.RaisedWalls;
import materia.tooling.casting.CastingSetup.CoreStrategy;
import materia.tooling.casting.CastingToolingInput.CastingFeatureKind;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelGeometry;
import materia.tooling.ToolingEnums.MachiningSide;

/** Orthogonal review template for multiple explicit EPS/former strategies.
 * All casting dimensions come from the source document. It never adds design cuts.
 */
class CastingMoldPlanner {
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

	public static function featureBox(d:ToolingDocument, f:materia.tooling.casting.CastingToolingInput.CastingToolingFeature):MoldBox
		return BoxMoldPlanner.box(f.width, f.depth, f.height, F.compose(d.setup.sourceToMold, f.pose));

	public static function plan(document:ToolingDocument, authored:CastingMoldOptions):CastingMoldPlan {
		var d = ToolingCodec.snapshot(document), o = CastingMoldOptionsCodec.snapshot(authored), e = envelope(d), r = d.recipe, t = r.panel.measuredThickness,
			l = r.minimumLigament;
		if (o.precision.length != d.input.inserts.length || o.anchorage.length != d.input.inserts.length)
			throw "Every insert needs precision and anchorage records";
		for (j in o.groutedJoints)
			for (id in j.insertIds)
				if (d.input.inserts.filter(i -> i.id == id).length != 1)
					throw "Unknown grouted interface insert";
		var bodies = new Map<String, MoldBox>(), low = e.z;
		for (i in d.input.inserts) {
			var b = BoxMoldPlanner.box(i.width, i.depth, i.height, F.compose(d.setup.sourceToMold, i.pose));
			bodies.set(i.id, b);
			low = Math.min(low, b.z);
			if (o.precision.filter(p -> p.insertId == i.id).length != 1 || o.anchorage.filter(p -> p.insertId == i.id).length != 1)
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
			formers: [],
			fixtures: [],
			features: [],
			openings: openings,
			hardware: [],
			changeRequests: [],
			validation: [],
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
			if (f.kind == CastingFeatureKind.Void) {
				var s = d.setup.features.filter(s -> s.featureId == f.id)[0],
					cb = BoxVolumes.intersection(featureBox(d, f), e);
				if (cb == null)
					throw "Feature misses envelope: " + f.id;
				if (s.strategy == CoreStrategy.SacrificialEPS) {
					if (Math.abs(cb.z + cb.height - b.z - b.height) > 1e-7
						|| cb.x <= e.x
						|| cb.x + cb.width >= e.x + e.width
						|| cb.y <= e.y
						|| cb.y + cb.depth >= e.y + e.depth)
						throw "Shared bridge needs internal fill-open EPS pocket: " + f.id;
					if (BoxVolumes.volume(BoxVolumes.subtract([cb], insertBoxes.concat(epsBoxes))) < BoxMoldPlanner.volume(cb) - 1e-5)
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
				} else if (s.strategy == CoreStrategy.Unsupported || s.draftDegrees > 0) {
					result.features.push({featureId: f.id, kind: UnsupportedFeature, memberIds: []});
					request(result, "unsupported-" + f.id, DraftOrSplit, [f.id],
						"Select a supported split/sacrificial strategy or supply drafted source geometry for this feature.",
						"tooling.feature.unsupported." + f.id, true);
				}
			}
		CoreSupports.sharedBridges(result.cavity, r, result.cores, result.supportPanels, result.hardware);
		// Priority is explicit: inserts, EPS and datum supports are reserved first;
		// rigid formers then resolve by descending authored priority and sorted IDs.
		var ordered = d.input.features.copy();
		ordered.sort((a, b) -> {
			var aa = d.setup.features.filter(v -> v.featureId == a.id),
				bb = d.setup.features.filter(v -> v.featureId == b.id);
			var ap = aa.length == 0 ? 0 : aa[0].priority,
				bp = bb.length == 0 ? 0 : bb[0].priority;
			return ap == bp ? Reflect.compare(a.id, b.id) : Reflect.compare(bp, ap);
		});
		// Datum fixtures occupy negative space only. Their overlap with casting is checked in CAD.
		for (i in d.input.inserts) {
			var pose = F.compose(d.setup.sourceToMold, i.pose),
				face = F.compose(pose, F.translation(0, 0, i.height)),
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
			var access = ordered.filter(f -> f.kind == CastingFeatureKind.Void
				&& d.setup.features.filter(s -> s.featureId == f.id
					&& s.strategy == CoreStrategy.ReusableFormer
					&& s.withdrawal != null
					&& Math.abs(s.withdrawal.x) > 0.9)
					.length == 1 && BoxVolumes.intersection(featureBox(d, f), pad) != null);
			if (access.length > 0) {
				var dir = d.setup.features.filter(s -> s.featureId == access[0].id)[0].withdrawal;
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
				var baseZ = Math.min(face.z - dt, b.z - dt),
					height = face.z - dt - baseZ;
				if (height > 1e-8)
					pieces.push({
						x: face.x - pw / 2,
						y: face.y - pw / 2,
						z: baseZ,
						width: pw,
						depth: pw,
						height: height
					});
				if (face.z <= b.z + 1e-7) {
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
					pieces = [pad];
				}
				var socket:MoldBox = face.z <= b.z + 1e-7 ? {
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
				pieces: BoxVolumes.subtract(pieces, [])
			};
			DatumFixtures.add(result, i, face, datum, withdrawal, bed, pocketId);
			result.features.push({
				featureId: d.input.features.filter(f -> f.kind == CastingFeatureKind.InsertRecess && f.insertId == i.id)[0].id,
				kind: InsertFeature,
				memberIds: [materia.tooling.ToolingIds.derive("insert-", i.id), datum.id]
			});
			result.validation.push(materia.tooling.ToolingValidation.ToolingReadiness.pending("tooling.fixture.attachment." + i.id, [i.id],
				"Bed pins register the nominal support only. Qualify insert XY location/clamping, tooling threads, datum accuracy and bore-mask release; no casting hole/thread is assumed."));
		}
		var fixtures:Array<MoldBox> = [];
		for (f in result.fixtures)
			for (p in f.datum.pieces)
				fixtures.push(p);
		var occupied = insertBoxes.concat(epsBoxes).concat(fixtures),
			previous:Array<MoldBox> = [];
		var backing = BoxVolumes.subtract([b], [e].concat(occupied));
		for (group in BoxVolumes.groups(backing)) {
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
			if (f.kind == CastingFeatureKind.Void) {
				var s = d.setup.features.filter(s -> s.featureId == f.id)[0];
				if (s.strategy != CoreStrategy.ReusableFormer || s.draftDegrees > 0)
					continue;
				var cb = BoxVolumes.intersection(featureBox(d, f), e);
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
					var extended = BoxVolumes.copy(cb);
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
				var pieces = BoxVolumes.subtract(add, occupied.concat(previous)),
					ids:Array<String> = [],
					groups = BoxVolumes.groups(pieces);
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
						draftDegrees: s.draftDegrees,
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
							if (former.volume.pieces.filter(p -> BoxVolumes.intersection(p, cb) != null).length > 0) former.volume.id
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
							if (fixture.datum.pieces.filter(p -> BoxVolumes.intersection(p, cb) != null).length > 0)
								open.sealingIds.push(fixture.datum.id);
					}
			}
		for (m in panels.members)
			m.panel = PanelCodec.snapshot(m.panel);
		for (feature in result.features)
			if (feature.kind == FormerFeature) {
				var source = d.input.features.filter(f -> f.id == feature.featureId)[0],
					cb = BoxVolumes.intersection(featureBox(d, source), e);
				if (cb == null)
					throw "Missing cavity in ownership allocation";
				feature.memberIds = [];
				for (former in result.formers)
					if (former.volume.pieces.filter(v -> BoxVolumes.intersection(v, cb) != null).length > 0) {
						feature.memberIds.push(former.volume.id);
						if (former.featureIds.indexOf(source.id) < 0)
							former.featureIds.push(source.id);
					}
				for (core in result.cores)
					if (BoxVolumes.intersection(core.cavity, cb) != null)
						feature.memberIds.push(core.id);
				for (fixture in result.fixtures)
					if (fixture.datum.pieces.filter(v -> BoxVolumes.intersection(v, cb) != null).length > 0)
						feature.memberIds.push(fixture.datum.id);
				for (insert in d.input.inserts)
					if (BoxVolumes.intersection(bodies.get(insert.id), cb) != null)
						feature.memberIds.push(materia.tooling.ToolingIds.derive("insert-", insert.id));
				feature.memberIds.sort(Reflect.compare);
				if (feature.memberIds.length == 0) {
					feature.kind = UnsupportedFeature;
					request(result, "unallocated-" + source.id, DraftOrSplit, [source.id],
						"Feature has no supported physical tooling owner; choose an explicit alternative.", "tooling.feature.unsupported." + source.id, true);
				}
			}
		for (strategy in d.setup.features)
			if (strategy.strategy == CoreStrategy.ReusableFormer && strategy.draftDegrees == 0)
				request(result, "draft-" + strategy.featureId, DraftOrSplit, [strategy.featureId],
					"Zero-draft rigid former requires source draft, a split, or a sacrificial alternative before physical release.",
					"tooling.former.draft." + strategy.featureId);
		if (result.handling.requiresFlip && !CastHandling.qualified(d, o, result.handling))
			request(result, "handling-lift-and-flip", LiftingPoints, [d.input.id],
				"Provide verified lifting/flip equipment and source lifting points or a qualified external cradle.", "tooling.handling.flip");
		for (seat in o.precision) {
			var source = d.input.inserts.filter(i -> i.id == seat.insertId)[0];
			if (seat.route != PrecisionRoute.AsCast && (seat.allowance == null || seat.allowance <= 0))
				request(result, "allowance-" + seat.insertId, InterfaceAllowance, [seat.insertId],
					"Specify finishing stock and a source revision where needed for the selected machining/replication route.",
					"tooling.precision.allowance." + seat.insertId);
			if (seat.route != sourceRoute(source.finish))
				request(result, "finish-" + seat.insertId, InterfaceAllowance, [seat.insertId],
					"Selected finish route differs from source finish intent; approve a source finish revision.",
					"tooling.precision.source-finish." + seat.insertId);
		}
		for (j in o.groutedJoints)
			if (j.gapMm == null)
				request(result, "grout-" + j.id, InterfaceAllowance, j.insertIds,
					"Define grout gap, jacking, sleeves and cure-before-torque; obtain source changes if needed.", "tooling.grouted-joint." + j.id);

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

	public static function sourceRoute(finish:materia.tooling.ToolingEnums.InsertFinish):PrecisionRoute
		return switch finish {
			case GroundAfterCasting, MachinedAfterCasting: PrecisionRoute.Machined;
			case ReplicatedAfterCasting: PrecisionRoute.Replicated;
			case SourceAsCast: PrecisionRoute.AsCast;
			default: throw "Unsupported source precision finish";
		};

	static function request(p:CastingMoldPlan, id:String, kind:DesignChangeKind, ids:Array<String>, message:String, rule:String,
			unsupported:Bool = false):Void {
		p.changeRequests.push({
			id: materia.tooling.ToolingIds.derive("", id),
			kind: kind,
			sourceIds: ids.copy(),
			reason: message,
			blockingChecks: [rule]
		});
		var check = materia.tooling.ToolingValidation.ToolingReadiness.pending(rule, ids, message);
		if (unsupported) {
			check.status = materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported;
			check.severity = materia.tooling.ToolingDiagnostic.ToolingSeverity.Error;
		}
		p.validation.push(check);
	}

	public static function validate(p:CastingMoldPlan):Void {
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
		for (f in p.formers)
			member(f.volume.id);
		for (f in p.fixtures)
			member(f.datum.id);
		for (i in p.document.input.inserts)
			member(materia.tooling.ToolingIds.derive("insert-", i.id));
		var ruleIds = [for (v in p.validation) v.rule];
		for (strategy in p.document.setup.features)
			if (strategy.strategy == CoreStrategy.ReusableFormer
				&& strategy.draftDegrees == 0
				&& ruleIds.indexOf("tooling.former.draft." + strategy.featureId) < 0)
				throw "Zero-draft former needs a release validation";
		for (seat in p.options.precision) {
			var source = p.document.input.inserts.filter(i -> i.id == seat.insertId)[0];
			if (seat.route != sourceRoute(source.finish) && ruleIds.indexOf("tooling.precision.source-finish." + seat.insertId) < 0)
				throw "Finish route differs from source without a change request";
			if (seat.route != PrecisionRoute.AsCast
				&& (seat.allowance == null || seat.allowance <= 0)
				&& ruleIds.indexOf("tooling.precision.allowance." + seat.insertId) < 0)
				throw "Finishing allowance needs a validation decision";
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
		for (f in p.fixtures)
			MoldVolumeCodec.snapshot(f.datum);
	}
}
