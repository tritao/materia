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

/** Nominal supports, with explicit locator and tool-only thread geometry.
 * Loose pins/screws/masks must be released before the solid-only removal graphs.
 * No source insert thread or pin hole is invented.
 */
class DatumFixtures {
	public static function add(p:CastingMoldPlan, i:CastingToolingInsert, face:materia.assembly.AssemblyRecord.AssemblyFrame, datum:MoldVolume,
			withdrawal:{x:Float, y:Float, z:Float}, bed:PanelPlacement, pocketId:Null<String>):Void {
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
			secondary = ToolingIds.derive("", id, "-secondary-pin"),
			fixing = ToolingIds.derive("", id, "-fixing");
		var penetrations:Array<PenetrationPlan> = [];
		var boltY = cy + h.fixturePinSpacing * 0.75;
		for (index in 0...3) {
			var operationId = index == 0 ? primary : index == 1 ? secondary : fixing;
			var x = index == 0 ? cx - h.fixturePinSpacing / 2 : index == 1 ? cx + h.fixturePinSpacing / 2 : cx,
				y = index == 2 ? boltY : cy;
			var diameter = index == 2 ? h.insertClearanceDiameter : d;
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
			var receiver = index == 2 ? PanelContours.circle(x, y,
				h.insertBoreDiameter / 2) : index == 1 ? PanelContours.obround(x, y, d + h.secondaryPinTravel, d) : PanelContours.circle(x, y, d / 2);
			if (index == 1 && h.fixtureSlotToolDiameter > d + 1e-7)
				throw "Fixture secondary-slot cutter exceeds slot width";
			var receiverDepth = index == 2 ? h.insertEngagement : embed;
			var machining = PanelJoints.operation(operationId, index == 1 ? Pocket : Drill, receiver, receiverDepth,
				index == 2 ? h.insertBoreDiameter : index == 1 ? h.fixtureSlotToolDiameter : d, MachiningSide.NegativeZ,
				Math.abs(floor + receiverDepth - face.z) < 1e-7);
			datum.cuts.push({operation: machining, pose: F.translation(0, 0, floor)});
			if (index != 2)
				p.hardware.push({
					id: operationId,
					kind: FixturePin,
					diameter: pinDiameter,
					length: floor - bed.pose.z + embed,
					pose: F.translation(x, y, bed.pose.z)
				});
		}
		var pose = F.fromRotationMatrix(cx, boltY, bed.pose.z, [1, 0, 0, 0, -1, 0, 0, 0, -1]);
		p.hardware.push({
			id: fixing,
			kind: FixtureScrew,
			diameter: h.insertScrewDiameter,
			length: floor - bed.pose.z + h.insertEngagement,
			pose: pose
		});
		var mask:Null<SupportHardware> = null;
		if (materia.casting.InsertHoleGeometry.diameter(i.holes) > 0) {
			if (h.plugMaterial != "steel")
				throw "Unsupported nominal bore-mask material";
			mask = {
				id: ToolingIds.derive("mask-", i.id),
				kind: BoreMask,
				diameter: materia.casting.InsertHoleGeometry.diameter(i.holes),
				length: i.height,
				pose: F.compose(p.document.setup.sourceToMold, i.pose)
			};
			p.hardware.push(mask);
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
			boreMaskWithdrawal: materia.casting.InsertHoleGeometry.diameter(i.holes) == 0 ? null : {x: 0, y: 0, z: Math.abs(withdrawal.x) > 0.9 ? 1 : -1},
			lateralLocation: {primaryId: primary, secondaryId: secondary, baselineMm: h.fixturePinSpacing},
			receiverThreadSpec: h.insertThreadSpec,
			fixingId: fixing,
			penetrations: penetrations,
			boreMask: mask
		});
	}

	public static function validate(p:CastingMoldPlan):Void {
		for (f in p.fixtures) {
			if (f.penetrations.length != 3 || f.datum.cuts.length != 3)
				throw "Incomplete fixture locator/fixing records";
			var ids = [for (c in f.datum.cuts) c.operation.id];
			if (ids.indexOf(f.lateralLocation.primaryId) < 0
				|| ids.indexOf(f.lateralLocation.secondaryId) < 0
				|| ids.indexOf(f.fixingId) < 0)
				throw "Fixture locator references unknown receiver";
			if (f.receiverThreadSpec != p.document.recipe.hardware.insertThreadSpec)
				throw "Fixture thread differs from recipe";
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
			if ((materia.casting.InsertHoleGeometry.diameter(source.holes) > 0) != (f.boreMask != null))
				throw "Source bore needs an explicit release mask";
		}
	}
}
