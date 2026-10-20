package materia.tooling.casting;
import materia.casting.BoxAlgebra;

import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingIds;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelContours;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingToolingInput.CastingToolingInsert;
import materia.tooling.casting.CoreSupport;
import materia.tooling.casting.MoldPlan;
import materia.tooling.panel.PanelAssemblyPlan.PanelPlacement;

/** Nominal supports, located by pins and clamped into source steel threads.
 * Loose pins/screws/masks must be released before the solid-only removal graphs.
 * No source insert thread or pin hole is invented.
 */
class DatumFixtures {
	/** Nominal access preference only; the native swept-removal graph verifies it.
	 * Side fixtures expose the through bore toward the open core behind the seat.
	 * Bed fixtures expose the mounting face. Blind holes always withdraw at entry.
	 */
	static function maskWithdrawal(p:CastingMoldPlan, insert:CastingToolingInsert, hole:materia.casting.InsertHole,
			withdrawal:{x:Float, y:Float, z:Float}):{x:Float, y:Float, z:Float} {
		var stock = F.compose(p.document.setup.sourceToMold, insert.pose);
		var entry = F.compose(stock, hole.frame), axis = F.transformVector(entry, 0, 0, 1);
		var sign = -1.0;
		if (hole.thread == null && hole.extent == materia.casting.InsertHole.HoleExtent.Through) {
			var preferred = F.transformVector(stock, 0, 0, Math.abs(withdrawal.z) < 0.9 ? -1 : 1);
			if (axis.x * preferred.x + axis.y * preferred.y + axis.z * preferred.z > 0)
				sign = 1;
		}
		return {x: sign * axis.x, y: sign * axis.y, z: sign * axis.z};
	}
	public static function add(p:CastingMoldPlan, i:CastingToolingInsert, face:materia.assembly.AssemblyRecord.AssemblyFrame, datum:MoldVolume,
			withdrawal:{x:Float, y:Float, z:Float}, bed:PanelPlacement, pocketId:Null<String>):Void {
		p.validation = p.validation.concat(InsertFixings.protectionValidation(i));
		var r = p.document.recipe,
			h = r.hardware,
			dt = h.datumThickness,
			pw = h.datumPostWidth;
		var cx = face.x, cy = face.y, floor = p.cavity.z - dt;
		if (Math.abs(withdrawal.x) > 0.9) {
			cx = withdrawal.x > 0 ? p.cavity.x + p.cavity.width + 2 * r.panel.measuredThickness : p.cavity.x - 2 * r.panel.measuredThickness;
			floor = p.cavity.z;
			var top = face.z - dt;
			if (top > floor)
				datum.pieces.push({
					x: cx - pw / 2,
					y: cy - pw / 2,
					z: floor,
					width: pw,
					depth: pw,
					height: top - floor
				});
			datum.pieces = BoxAlgebra.subtract(datum.pieces, []);
		}
		var id = datum.id, pinDiameter = h.fixturePinDiameter, d = pinDiameter + r.jointClearance, embed = h.fixturePinEmbed;
		var primary = ToolingIds.derive("", id, "-primary-pin"),
			secondary = ToolingIds.derive("", id, "-secondary-pin");
		var pinSpacing = h.fixturePinSpacing, pinAlongY = false;
		for (hole in i.holes) {
			var fixing = InsertFixings.select(hole, F.compose(p.document.setup.sourceToMold, i.pose), face, h);
			if (fixing != null)
				pinSpacing = Math.max(pinSpacing, fixing.clearanceDiameter + d + h.secondaryPinTravel + 2 * r.minimumLigament);
		}
		if (pinSpacing > h.fixturePinSpacing) {
			var feet = datum.pieces.filter(piece -> Math.abs(piece.z - floor) < 1e-7);
			var requiredSpan = pinSpacing + d + h.secondaryPinTravel + 2 * r.minimumLigament;
			var alongX = feet.filter(piece -> piece.width >= requiredSpan).length > 0;
			var alongY = feet.filter(piece -> piece.depth >= requiredSpan).length > 0;
			pinAlongY = !alongX && alongY;
			// A support too small for the required baseline remains blocked by the
			// normal bore/ligament checks; no source hole or support is invented.
			if (!alongX && !alongY)
				pinSpacing = h.fixturePinSpacing;
		}
		var penetrations:Array<PenetrationPlan> = [];
		for (index in 0...2) {
			var operationId = index == 0 ? primary : secondary;
			var offset = index == 0 ? -pinSpacing / 2 : pinSpacing / 2;
			var x = cx + (pinAlongY ? 0 : offset), y = cy + (pinAlongY ? offset : 0);
			var diameter = d;
			var depth = r.panel.measuredThickness;
			var operation = PanelJoints.operation(operationId, Drill, PanelContours.circle(x - bed.pose.x, y - bed.pose.y, diameter / 2), depth, diameter);
			operation.through = true;
			operation.accessVia = pocketId;
			bed.panel.operations.push(operation);
			bed.panel.contactOpenings.push(operation.id);
			p.openings.push({
				panelId: bed.panel.id,
				operationId: operation.id,
				kind: ToolingOpening,
				sealingIds: [datum.id],
				sealSpec: h.sealMaterial
			});
			penetrations.push({
				panelId: bed.panel.id,
				operationId: operation.id,
				kind: InsertSealed,
				insertId: i.id,
				coreId: null,
				sealMargin: 0,
				sealMaterial: h.sealMaterial,
				sealingFace: F.translation(x, y, floor),
				plug: null
			});
			var receiver = index == 1 ? PanelContours.obround(x, y, d + (pinAlongY ? 0 : h.secondaryPinTravel), d + (pinAlongY ? h.secondaryPinTravel : 0)) : PanelContours.circle(x, y, d / 2);
			if (index == 1 && h.fixtureSlotToolDiameter > d + 1e-7)
				throw "Fixture secondary-slot cutter exceeds slot width";
			var receiverDepth = embed;
			var machining = PanelJoints.operation(operationId, index == 1 ? Pocket : Drill, receiver, receiverDepth,
				index == 1 ? h.fixtureSlotToolDiameter : d, MachiningSide.NegativeZ,
				Math.abs(floor + receiverDepth - face.z) < 1e-7);
			datum.cuts.push({operation: machining, pose: F.translation(0, 0, floor)});
			p.hardware.push({
				id: operationId,
				kind: FixturePin,
				diameter: pinDiameter,
				length: floor - bed.pose.z + embed,
				pose: F.translation(x, y, bed.pose.z)
			});
		}
		var fixings:Array<InsertFixtureFixing> = [];
		var sourceCandidates = 0;
		for (hole in i.holes) {
			var selected = InsertFixings.select(hole, F.compose(p.document.setup.sourceToMold, i.pose), face, h);
			if (selected == null)
				continue;
			var entry = selected.entry, engagement = selected.engagement;
			var thread:materia.casting.InsertHole.InsertThread = cast hole.thread;
			sourceCandidates++;
			var diameter = selected.clearanceDiameter;
			// A straight clearance bore must remain inside the supporting datum.
			var path:Array<MoldBox> = [{x: entry.x - diameter / 2, y: entry.y - diameter / 2,
				z: Math.max(floor, bed.pose.z + r.panel.measuredThickness), width: diameter, depth: diameter,
				height: entry.z - Math.max(floor, bed.pose.z + r.panel.measuredThickness)}];
			if (path[0].height > 1e-7 && BoxAlgebra.volume(BoxAlgebra.subtract(path, datum.pieces)) > 1e-7)
				continue;
			var fixingId = ToolingIds.derive("fixing-", i.id + "-" + hole.id);
			var clearance = PanelJoints.operation(fixingId, Drill,
				PanelContours.circle(entry.x - bed.pose.x, entry.y - bed.pose.y, diameter / 2),
				r.panel.measuredThickness, diameter);
			clearance.through = true;
			clearance.accessVia = pocketId;
			var conflicts = bed.panel.operations.filter(other -> other.kind != OutsideProfile && other.id != pocketId
				&& materia.tooling.panel.PanelGeometry.distanceBetween(clearance.contour, other.contour) < r.minimumLigament - 1e-7);
			if (conflicts.length > 0)
				continue;
			bed.panel.operations.push(clearance);
			bed.panel.contactOpenings.push(fixingId);
			p.openings.push({panelId: bed.panel.id, operationId: fixingId, kind: ToolingOpening,
				sealingIds: [ToolingIds.derive("insert-", i.id), datum.id], sealSpec: h.sealMaterial});
			penetrations.push({panelId: bed.panel.id, operationId: fixingId, kind: InsertSealed,
				insertId: i.id, sourceHoleId: hole.id, coreId: null, sealMargin: h.sealMargin, sealMaterial: h.sealMaterial,
				sealingFace: F.translation(entry.x, entry.y, floor), plug: selected.plug});
			if (selected.plug != null) {
				var plug = selected.plug;
				p.hardware.push({id: ToolingIds.derive("resin-plug-", i.id + "-" + hole.id), kind: BorePlug,
					diameter: plug.diameter, length: plug.length,
					pose: F.compose(entry, F.translation(0, 0, hole.depth - plug.length))});
			}
			var depth = entry.z - floor;
			if (depth > 1e-7) {
				var receiver = PanelJoints.operation(fixingId, Drill, PanelContours.circle(entry.x, entry.y, diameter / 2),
					depth, diameter, MachiningSide.NegativeZ, true);
				datum.cuts.push({operation: receiver, pose: F.translation(0, 0, floor)});
			}
			p.hardware.push({id: fixingId, kind: FixtureScrew, diameter: thread.majorDiameter,
				length: entry.z - bed.pose.z + engagement,
				pose: F.fromRotationMatrix(entry.x, entry.y, bed.pose.z, [1, 0, 0, 0, -1, 0, 0, 0, -1])});
			fixings.push({sourceHoleId: hole.id, hardwareId: fixingId, panelId: bed.panel.id,
				operationId: fixingId, entry: entry, engagement: engagement, threadSpec: thread.designation});
		}
		if (fixings.length == 0 && sourceCandidates == 0) {
			var rule = "tooling.fixture.source-hole." + i.id;
			p.changeRequests.push({id: ToolingIds.derive("fixture-hole-", i.id), kind: InsertFixtureHole,
				sourceIds: [i.id], reason: "Declare an accessible steel threaded hole facing the bed for insert clamping.",
				blockingChecks: [rule]});
			p.validation.push(materia.tooling.ToolingValidation.ToolingReadiness.pending(rule, [i.id],
				"Fixture blocked: no usable source threaded hole; tooling cannot invent one."));
		}
		if (fixings.length == 0 && sourceCandidates > 0)
			p.validation.push(materia.tooling.ToolingValidation.ToolingReadiness.pending("tooling.fixture.clearance." + i.id, [i.id, datum.id],
				"Source threads exist, but the support/locator layout leaves no clear screw path; revise the tooling fixture."));
		var masks:Array<BoreMaskProtection> = [];
		for (hole in i.holes) {
			if (fixings.filter(f -> f.sourceHoleId == hole.id).length > 0)
				continue;
			if (h.plugMaterial != "steel")
				throw "Unsupported nominal bore-mask material";
			var entry = F.compose(F.compose(p.document.setup.sourceToMold, i.pose), hole.frame);
			var protection = InsertFixings.mask(hole, ToolingIds.derive("mask-", i.id + "-" + hole.id), h.plugDepth,
				maskWithdrawal(p, i, hole, withdrawal));
			var mask:SupportHardware = {
				id: ToolingIds.derive("mask-", i.id + "-" + hole.id),
				kind: BoreMask,
				diameter: hole.thread == null ? hole.diameter : hole.thread.majorDiameter,
				length: protection.plugLength == null ? hole.depth : protection.plugLength,
				pose: entry
			};
			p.hardware.push(mask);
			masks.push(protection);
			if (protection.farEndId != null)
				p.hardware.push({id: protection.farEndId, kind: BorePlug, diameter: mask.diameter, length: mask.length,
					pose: F.compose(entry, F.translation(0, 0, hole.depth - mask.length))});
		}
		p.fixtures.push({
			insertId: i.id,
			datums: BoxMoldPlanner.datums(face, i.width, i.depth),
			datum: datum,
			mountingFace: face,
			withdrawal: withdrawal,
			attachmentEvidence: [],
			assemblyClearance: {
				x: withdrawal.x,
				y: withdrawal.y,
				z: Math.abs(withdrawal.x) > 0.9 ? 0 : 1
			},
			lateralLocation: {primaryId: primary, secondaryId: secondary, baselineMm: pinSpacing},
			fixings: fixings,
			penetrations: penetrations,
			boreMasks: masks
		});
	}

	public static function validate(p:CastingMoldPlan):Void {
		for (f in p.fixtures) {
			if (f.penetrations.length != 2 + f.fixings.length || f.datum.cuts.length < 2)
				throw "Incomplete fixture locator/fixing records";
			var ids = [for (c in f.datum.cuts) c.operation.id];
			if (ids.indexOf(f.lateralLocation.primaryId) < 0
				|| ids.indexOf(f.lateralLocation.secondaryId) < 0)
				throw "Fixture locator references unknown receiver";
			for (pen in f.penetrations) {
				var panel = p.panels.members.filter(m -> m.panel.id == pen.panelId),
					face = pen.sealingFace;
				if (panel.length != 1 || pen.insertId != f.insertId || face == null)
					throw "Invalid insert-sealed fixture penetration";
				var operations = panel[0].panel.operations.filter(o -> o.id == pen.operationId);
				if (operations.length != 1)
					throw "Missing fixture penetration operation";
				var b = materia.tooling.panel.PanelGeometry.bounds(operations[0].contour);
				var center = F.transformPoint(panel[0].pose, b.x + b.width / 2, b.y + b.height / 2, panel[0].panel.stock.measuredThickness);
				if (Math.abs(center.x - face.x) + Math.abs(center.y - face.y) > 1e-7)
					throw "Fixture seal face misses penetration";
				var covered = false;
				for (foot in f.datum.pieces)
					if (Math.abs(foot.z - face.z) < 1e-7
						&& center.x - b.width / 2 >= foot.x - 1e-7
						&& center.x + b.width / 2 <= foot.x + foot.width + 1e-7
						&& center.y - b.height / 2 >= foot.y - 1e-7
						&& center.y + b.height / 2 <= foot.y + foot.depth + 1e-7)
						covered = true;
				if (!covered)
					throw "Fixture sealing footprint does not cover bed penetration";
			}
			var source = p.document.input.inserts.filter(i -> i.id == f.insertId)[0];
			if (f.fixings.length == 0 && p.validation.filter(v -> v.rule == "tooling.fixture.clearance." + source.id).length == 0
				&& p.changeRequests.filter(request -> request.kind == InsertFixtureHole
				&& request.sourceIds.indexOf(source.id) >= 0
				&& request.blockingChecks.indexOf("tooling.fixture.source-hole." + source.id) >= 0).length != 1)
				throw "Fixture without source fixing needs a design change request";
			if (f.boreMasks.length + f.fixings.length != source.holes.length)
				throw "Every source hole needs a fixing screw or explicit release mask";
			for (hole in source.holes) {
				var fixings = f.fixings.filter(fixing -> fixing.sourceHoleId == hole.id);
				if (fixings.length > 0) {
					var thread:materia.casting.InsertHole.InsertThread = cast hole.thread;
					var fixing = fixings[0], screws = p.hardware.filter(h -> h.id == fixing.hardwareId);
					var entry = F.compose(F.compose(p.document.setup.sourceToMold, source.pose), hole.frame);
					var selected = InsertFixings.select(hole, F.compose(p.document.setup.sourceToMold, source.pose), f.mountingFace,
						p.document.recipe.hardware);
					var penetrations = f.penetrations.filter(pen -> pen.sourceHoleId == hole.id && pen.operationId == fixing.operationId);
					if (selected == null || penetrations.length != 1
						|| ToolingCodec.canonical(penetrations[0].plug) != ToolingCodec.canonical(selected.plug))
						throw "Fixture fixing resin-side protection differs from source depth budget: " + hole.id;
					if (selected.plug != null) {
						var plug = selected.plug;
						var hardware = p.hardware.filter(h -> h.id == ToolingIds.derive("resin-plug-", source.id + "-" + hole.id));
						if (hardware.length != 1 || hardware[0].kind != BorePlug || hardware[0].diameter != plug.diameter
							|| hardware[0].length != plug.length
							|| ToolingCodec.canonical(hardware[0].pose) != ToolingCodec.canonical(F.compose(entry, F.translation(0, 0, hole.depth - plug.length))))
							throw "Missing or inconsistent resin-side fixing plug hardware: " + hole.id;
					}
					var receivers = f.datum.cuts.filter(cut -> cut.operation.id == fixing.operationId);
					var expected = PanelContours.circle(entry.x, entry.y,
						hole.clearanceDiameter / 2);
					if (receivers.length != 1 || receivers[0].operation.kind != Drill || !receivers[0].operation.through
						|| ToolingCodec.canonical(receivers[0].operation.contour) != ToolingCodec.canonical(expected))
						throw "Datum fixing receiver must be a source-sized clearance bore";
					if (fixings.length != 1 || hole.thread == null || screws.length != 1
						|| screws[0].kind != FixtureScrew || screws[0].diameter != thread.majorDiameter
						|| Math.abs(screws[0].length - (entry.z - screws[0].pose.z + materia.casting.InsertHole.InsertHoleCodec.threadLimits(hole).minEngagement)) > 1e-7
						|| Math.abs(screws[0].pose.x - entry.x) + Math.abs(screws[0].pose.y - entry.y) > 1e-7
						|| f.penetrations.filter(pen -> pen.sourceHoleId == hole.id && pen.operationId == fixing.operationId).length != 1
						|| fixing.engagement != materia.casting.InsertHole.InsertHoleCodec.threadLimits(hole).minEngagement
						|| fixing.threadSpec != thread.designation
						|| ToolingCodec.canonical(fixing.entry) != ToolingCodec.canonical(entry)
						|| f.boreMasks.filter(mask -> mask.sourceHoleId == hole.id).length != 0)
						throw "Fixture fixing differs from source thread: " + hole.id;
					continue;
				}
				var protections = f.boreMasks.filter(mask -> mask.sourceHoleId == hole.id);
				if (protections.length != 1)
					throw "Source hole needs one unique release mask: " + hole.id;
				var protection = protections[0], hardware = p.hardware.filter(h -> h.id == protection.hardwareId);
				var expectedProtection = InsertFixings.mask(hole, protection.hardwareId, p.document.recipe.hardware.plugDepth,
					maskWithdrawal(p, source, hole, f.withdrawal));
				if (ToolingCodec.canonical(protection) != ToolingCodec.canonical(expectedProtection))
					throw "Source thread protection differs from its depth budget: " + hole.id;
				if (protection.farEndId != null) {
					var far = p.hardware.filter(h -> h.id == protection.farEndId);
					var entry = F.compose(F.compose(p.document.setup.sourceToMold, source.pose), hole.frame);
					if (far.length != 1 || far[0].kind != BorePlug || far[0].length != protection.plugLength
						|| ToolingCodec.canonical(far[0].pose) != ToolingCodec.canonical(F.compose(entry, F.translation(0, 0, hole.depth - far[0].length))))
						throw "Missing resin-side thread mask: " + hole.id;
				}
				var entry = F.compose(F.compose(p.document.setup.sourceToMold, source.pose), hole.frame);
				if (hardware.length != 1 || hardware[0].kind != BoreMask || hardware[0].diameter != (hole.thread == null ? hole.diameter : hole.thread.majorDiameter)
					|| hardware[0].length != (protection.plugLength == null ? hole.depth : protection.plugLength) || ToolingCodec.canonical(hardware[0].pose) != ToolingCodec.canonical(entry)
					|| ToolingCodec.canonical(protection.withdrawal) != ToolingCodec.canonical(maskWithdrawal(p, source, hole, f.withdrawal)))
					throw "Bore mask differs from its source hole: " + hole.id;
			}
		}
	}
}
