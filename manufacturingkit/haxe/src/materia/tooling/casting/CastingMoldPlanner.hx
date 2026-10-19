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
					var g = 0.2, cover = r.linerThickness, foam:MoldBox = {
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
				} else if (s.strategy != CoreStrategy.ReusableFormer)
					throw "Unsupported feature strategy: " + f.id;
			}
		buildBridges(result);
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
			var pad:MoldBox = {
				x: body.x,
				y: body.y,
				z: face.z - 8,
				width: body.width,
				depth: body.depth,
				height: 8
			}, pieces = [pad], withdrawal = {
				x: 0.0,
				y: 0.0,
				z: -1.0
			};
			var access = d.input.features.filter(f -> f.kind == CastingFeatureKind.Void
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
					z: face.z - 8,
					width: b.x + b.width + 3 * t - body.x,
					depth: body.depth,
					height: 8
				} : {
					x: b.x - 3 * t,
					y: body.y,
					z: face.z - 8,
					width: body.x + body.width - b.x + 3 * t,
					depth: body.depth,
					height: 8
					};
				pieces = [pad];
			} else {
				var baseZ = Math.min(face.z - 8, b.z - 8),
					height = face.z - 8 - baseZ;
				if (height > 1e-8)
					pieces.push({
						x: face.x - 10,
						y: face.y - 10,
						z: baseZ,
						width: 20,
						depth: 20,
						height: height
					});
				if (face.z <= b.z + 1e-7) {
					var inset = r.routerDiameter + r.minimumLigament;
					pad = {
						x: body.x + inset,
						y: body.y + inset,
						z: face.z - 8,
						width: body.width - 2 * inset,
						depth: body.depth - 2 * inset,
						height: 8
					};
					if (pad.width <= r.routerDiameter || pad.depth <= r.routerDiameter)
						throw "Insert too small for sealed relieved datum socket: " + i.id;
					pieces = [pad];
				}
				var socket:MoldBox = face.z <= b.z + 1e-7 ? {
					x: pad.x,
					y: pad.y,
					z: b.z - 8,
					width: pad.width,
					depth: pad.depth,
					height: 8.0
				} : {
					x: face.x - 10,
					y: face.y - 10,
					z: b.z - 8,
					width: 20.0,
					depth: 20.0,
					height: 8.0
					};
				var op = PanelJoints.operation(materia.tooling.ToolingIds.derive("datum-", i.id), Pocket,
					PanelContours.relievedRectangle(socket.x - bed.pose.x + socket.width / 2, socket.y - bed.pose.y + socket.depth / 2, socket.width,
						socket.depth, r.routerDiameter / 2),
					8, r.routerDiameter, MachiningSide.PositiveZ, false, r.routerDiameter);
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
				profiles: [],
				pieces: BoxVolumes.subtract(pieces, [])
			};
			result.fixtures.push({
				insertId: i.id,
				datums: BoxMoldPlanner.datums(face, i.width, i.depth),
				datum: datum,
				mountingFace: face,
				withdrawal: withdrawal,
				attachmentEvidence: []
			});
			result.features.push({
				featureId: d.input.features.filter(f -> f.kind == CastingFeatureKind.InsertRecess && f.insertId == i.id)[0].id,
				kind: InsertFeature,
				memberIds: [materia.tooling.ToolingIds.derive("insert-", i.id), datum.id]
			});
			result.changeRequests.push({
				id: materia.tooling.ToolingIds.derive("attachment-", i.id),
				kind: FixtureAttachment,
				sourceIds: [i.id],
				reason: "Provide and qualify detachable fixing/clamping to the independent datum fixture; existing bore is not assumed to be a fixture thread.",
				blockingChecks: ["insert-fixture-attachment", "thread-protection"]
			});
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
					profiles: [],
					pieces: group
				},
				featureIds: [],
				withdrawal: {
					x: 0,
					y: 0,
					z: -1
				}
			});
		}
		for (f in d.input.features)
			if (f.kind == CastingFeatureKind.Void) {
				var s = d.setup.features.filter(s -> s.featureId == f.id)[0];
				if (s.strategy != CoreStrategy.ReusableFormer)
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
						x: direction.x > 0 ? side + t : side - t - 12,
						y: cb.y - 10,
						z: cb.z - 10,
						width: 12,
						depth: cb.depth + 20,
						height: cb.height + 20
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
							profiles: profiles,
							pieces: groups[index]
						},
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
					if (ids.length == 0)
						throw "Feature has no tooling owner: " + f.id;
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
		if (o.requiresFlip && o.liftingEvidence.length == 0)
			result.changeRequests.push({
				id: "handling-lift-and-flip",
				kind: LiftingPoints,
				sourceIds: [d.input.id],
				reason: "Upside-down casting needs qualified lift/flip points and handling equipment before release.",
				blockingChecks: ["handling", "demolding-handling"]
			});
		for (seat in o.precision)
			if (seat.route == PrecisionRoute.Replicated && seat.allowance == null)
				result.changeRequests.push({
					id: "replication-" + seat.insertId,
					kind: InterfaceAllowance,
					sourceIds: [seat.insertId],
					reason: "Specify replication allowance below the final plane and obtain a source revision if the current nominal insert must move; the mold does not establish final alignment.",
					blockingChecks: ["precision-replication"]
				});
		for (j in o.groutedJoints)
			if (j.gapMm == null)
				result.changeRequests.push({
					id: "grout-" + j.id,
					kind: InterfaceAllowance,
					sourceIds: j.insertIds,
					reason: "Define grout gap, jacking/squaring, bolt sleeves and cure-before-torque; obtain source interface changes before manufacturing.",
					blockingChecks: ["grouted-joint"]
				});
		result.validation = [
			materia.tooling.ToolingValidation.ToolingReadiness.pending("closure-loads", [d.input.id],
				"Wall/pad clamps, braces and uplift capacity require qualification"),
			materia.tooling.ToolingValidation.ToolingReadiness.pending("insert-fixtures", [for (i in d.input.inserts) i.id],
				"Attachment, datum accuracy and thread protection require evidence"),
			materia.tooling.ToolingValidation.ToolingReadiness.pending("materials-process", [d.recipe.id],
				"Adhesive, covering, release and cure compatibility require qualification"),
			materia.tooling.ToolingValidation.ToolingReadiness.pending("precision-interfaces", [d.input.id],
				"Replication and grout interfaces require evidence"),
			materia.tooling.ToolingValidation.ToolingReadiness.pending("anchorage-compaction", [d.input.id], "Anchorage and compaction loads require evidence")
		];
		for (request in result.changeRequests)
			result.validation.push(materia.tooling.ToolingValidation.ToolingReadiness.pending("design-change-" + request.id, request.sourceIds,
				request.reason));
		result.fabrication = materia.tooling.ToolingValidation.ToolingReadiness.evaluate(result.validation);
		validate(result);
		return result;
	}

	static function buildBridges(p:CastingMoldPlan):Void {
		if (p.cores.length == 0)
			return;
		var b = p.cavity, r = p.document.recipe, t = r.panel.measuredThickness, l = r.minimumLigament, edge = 2 * l + r.routerDiameter,
			pad = r.hardware.padDepth;
		var rows:Array<Float> = [];
		for (c in p.cores) {
			var y = c.cavity.y + c.cavity.depth / 2;
			if (rows.filter(v -> Math.abs(v - y) < 1e-7).length == 0)
				rows.push(y);
		}
		rows.sort(Reflect.compare);
		function place(panel:materia.tooling.panel.PanelPlan, x:Float, y:Float, z:Float):Void
			p.supportPanels.push({
				panel: PanelCodec.snapshot(panel),
				pose: F.translation(x, y, z),
				withdrawal: {
					x: 0,
					y: 0,
					z: 1
				}
			});
		var rails = [];
		for (right in [false, true]) {
			var x = right ? b.x + b.width : b.x - pad - edge,
				y = b.y - t - edge;
			var rail = PanelJoints.blank(right ? "bridge-right-rail" : "bridge-left-rail", pad + edge, b.depth + 2 * t + 2 * edge, r.panel, r.routerDiameter,
				l);
			rail.labelX = right ? pad + edge - 16 : 16;
			rail.operations.push(PanelJoints.operation("wall-top-socket", Rebate,
				PanelContours.relievedRectangle(right ? t / 2 : pad + edge - t / 2, (b.depth + 2 * t + 2 * edge) / 2, t + r.jointClearance,
					b.depth + r.jointClearance, r.routerDiameter / 2),
				t / 2, r.routerDiameter, MachiningSide.NegativeZ, false, r.routerDiameter));
			rails.push({
				panel: rail,
				x: x,
				y: y,
				right: right
			});
		}
		for (index in 0...rows.length) {
			var y = rows[index],
				width = Math.min(r.hardware.minimumBridgeWidth, [
					for (c in p.cores)
						if (Math.abs(c.cavity.y + c.cavity.depth / 2 - y) < 1e-7) c.cavity.depth
				][0] / 2);
			var id = "core-bridge-" + index,
				beam = PanelJoints.blank(id, b.width + 2 * (pad + edge), width, r.panel, r.routerDiameter, l),
				bx = b.x - pad - edge,
				by = y - width / 2;
			beam.labelY = width / 2;
			for (rail in rails) {
				var x = rail.right ? b.x + b.width + 2 * t : b.x - 2 * t,
					py = y - width / 4,
					byy = y + width / 4,
					d = r.hardware.locatingPinDiameter + r.jointClearance;
				rail.panel.operations.push(PanelJoints.operation("pin-" + index, Drill, PanelContours.circle(x - rail.x, py - rail.y, d / 2), t, d));
				rail.panel.operations.push(PanelJoints.operation("bolt-" + index, Drill,
					PanelContours.circle(x - rail.x, byy - rail.y, r.hardware.supportClearanceDiameter / 2), t, r.hardware.supportClearanceDiameter));
				beam.operations.push(PanelJoints.operation(rail.right ? "secondary-slot" : "primary-pin", rail.right ? InsideProfile : Drill,
					rail.right ? PanelContours.obround(x - bx, py - by, r.hardware.locatingPinDiameter + r.hardware.secondaryPinTravel + r.jointClearance,
						d) : PanelContours.circle(x - bx, py - by, d / 2),
					t, rail.right ? r.routerDiameter : d));
				beam.operations.push(PanelJoints.operation(rail.right ? "right-bolt" : "left-bolt", Drill,
					PanelContours.circle(x - bx, byy - by, r.hardware.supportClearanceDiameter / 2), t, r.hardware.supportClearanceDiameter));
				p.hardware.push({
					id: id + (rail.right ? "-right-pin" : "-left-pin"),
					kind: LocatingPin,
					pose: F.translation(x, py, b.z + b.height),
					diameter: r.hardware.locatingPinDiameter,
					length: 2 * t
				});
				p.hardware.push({
					id: id + (rail.right ? "-right-bolt" : "-left-bolt"),
					kind: ClampBolt,
					pose: F.translation(x, byy, b.z + b.height + 2 * t),
					diameter: r.hardware.supportScrewDiameter,
					length: 2 * t
				});
			}
			place(beam, bx, by, b.z + b.height + t);
			for (c in p.cores)
				if (Math.abs(c.cavity.y + c.cavity.depth / 2 - y) < 1e-7) {
					c.bridgeId = id;
					var f = c.foamLayers[0];
					c.bridgeBondAreaMm2 = f.width * width;
					place(PanelJoints.blank(c.carrierId, f.width, f.depth, r.panel, r.routerDiameter, l), f.x, f.y, b.z + b.height);
				}
		}
		for (rail in rails)
			place(rail.panel, rail.x, rail.y, b.z + b.height);
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
		var ids = new Map<String, Bool>();
		for (f in p.features) {
			for (id in f.memberIds)
				if (!members.exists(id))
					throw "Unknown feature tooling member";
			if (ids.exists(f.featureId) || f.memberIds.length == 0)
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
