package materia.tooling.export;

import materia.tooling.export.FabricationSpec.FabricationSpecs;

import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlanner;
import materia.tooling.casting.CastHandling;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingValidation.ToolingReadiness;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.panel.PanelPlan;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelContours;
import materia.casting.CastingProfileGeometry;
import materia.tooling.panel.PanelCoupon;
import materia.tooling.panel.PanelJoints;
import materia.sheet.SheetPartRequirement;
import materia.tooling.export.FabricationSpec;
import materia.tooling.export.ToolingLoads.LoadReport;

typedef MoldEffortMetrics = {
	var panels:Int;
	var uniquePanels:Int;
	var sheetCount:Int;

	/** Bounding blank area / supplied sheet area, not contour nesting yield. */
	var blankSheetUtilization:Float;

	var formers:Int;
	var epsCores:Int;
	var sacrificialEPSVolumeMm3:Float;
	var logicalHardwareStacks:Int;
	var castingMassKg:Float;
	var mixMassKg:Float;
	var openChangeRequests:Int;
	var requiresFlip:Bool;
}

typedef BlankPlacement = {
	var panelId:String;
	var sheet:Int;
	var xMm:Float;
	var yMm:Float;
	var widthMm:Float;
	var heightMm:Float;
	var rotationDegrees:Int;
}

enum abstract ReplacementUse(String) to String {
	var Reusable = "reusable";
	var PerCasting = "per-casting";
}

typedef ReplacementGroup = {var id:String; var use:ReplacementUse; var memberIds:Array<String>;}

typedef FabricationPlan = {
	var lengthUnit:String;
	var sourceFingerprint:String;
	var fingerprint:String;
	var spec:FabricationSpec;
	var panelRequirements:Array<SheetPartRequirement>;
	var sheetStock:materia.sheet.SheetStockSpec;
	var blankPlacements:Array<BlankPlacement>;
	var coupon:PanelCouponPlan;
	var locatorCoupon:materia.tooling.panel.PanelJoints.PanelJointPair;
	var loads:LoadReport;
	var validation:Array<ToolingValidation>;
	var fabrication:FabricationReadiness;
	var replacementGroups:Array<ReplacementGroup>;
}

/** Pure fabrication inputs and reports. No CAD, CAM, drawing writers or inventory mutations. */
class ToolingFabrication {
	public static function requirement(p:PanelPlan, marginMm:Float, materialId:String):SheetPartRequirement {
		p = PanelCodec.copy(p);
		if (!Math.isFinite(marginMm) || marginMm < 0)
			throw "Invalid panel blank margin";
		var b = CastingProfileGeometry.bounds(PanelContours.outside(p)), s = p.stock;
		return {
			id: p.id,
			revision: p.revision,
			partId: p.id,
			partRevision: p.revision,
			quantity: 1,
			materialId: materialId,
			lengthUnit: "mm",
			thickness: s.measuredThickness,
			thicknessToleranceMinus: s.thicknessVariation,
			thicknessTolerancePlus: s.thicknessVariation,
			blankWidth: b.width + 2 * marginMm,
			blankHeight: b.height + 2 * marginMm,
			widthToleranceMinus: 0,
			widthTolerancePlus: 0,
			heightToleranceMinus: 0,
			heightTolerancePlus: 0,
			allowedRotations: s.allowedRotations.copy(),
			directionConstraint: s.directionConstraint
		};
	}

	/** Deterministic shelf layout of bounding blanks; not a contour nest or executable sheet inventory plan. */
	public static function layout(parts:Array<SheetPartRequirement>, width:Float, height:Float, kerf:Float):Array<BlankPlacement> {
		ToolingLoads.positive(width);
		ToolingLoads.positive(height);
		if (!Math.isFinite(kerf) || kerf < 0)
			throw "Invalid blank kerf";
		var sorted = parts.copy();
		sorted.sort((a, b) -> {
			var h = Reflect.compare(b.blankHeight, a.blankHeight);
			return h == 0 ? Reflect.compare(a.id, b.id) : h;
		});
		var sheet = 1, x = 0.0, y = 0.0, row = 0.0, result:Array<BlankPlacement> = [];
		for (p in sorted) {
			var rotation = p.allowedRotations.indexOf(0) >= 0 ? 0 : 90;
			var w = rotation == 0 ? p.blankWidth : p.blankHeight,
				h = rotation == 0 ? p.blankHeight : p.blankWidth;
			if (w > width || h > height) {
				if (p.allowedRotations.indexOf(90 - rotation) < 0 || h > width || w > height)
					throw 'Panel blank exceeds sheet: ${p.id}';
				var temp = w;
				w = h;
				h = temp;
				rotation = 90 - rotation;
			}
			if (x + w > width) {
				x = 0;
				y += row + kerf;
				row = 0;
			}
			if (y + h > height) {
				sheet++;
				x = 0;
				y = 0;
				row = 0;
			}
			result.push({
				panelId: p.partId,
				sheet: sheet,
				xMm: x,
				yMm: y,
				widthMm: w,
				heightMm: h,
				rotationDegrees: rotation
			});
			x += w + kerf;
			row = Math.max(row, h);
		}
		return result;
	}

	public static function plan(p:CastingMoldPlan, authored:FabricationSpec, ?process:materia.tooling.process.CastingProcessAnalysis.CastingProcessReport):FabricationPlan {
		CastingMoldPlanner.validate(p);
		var s = FabricationSpecs.copy(authored),
			checks = p.validation.copy();
		var eps = p.cores.length > 0;
		checks.push(ToolingLoads.check("fabrication.materials.compatibility", [p.document.recipe.id],
			(eps || s.adhesiveContainsEPS != false)
			&& s.solvent == StyreneBearing ? Fail : s.solvent == Unknown || s.compatibilityEvidence.length == 0 ? Pending : Pass,
			"Styrene-bearing resin is incompatible with EPS; the actual resin/EPS/adhesive/release combination requires chemical and observed cure-temperature compatibility evidence. When EPS is present, its declared stock limit is "
			+p.document.recipe.core.maximumTemperatureC+" C.",
			s.compatibilityEvidence));
		if (process!=null && (process.sourceFingerprint!=CastingMoldPlanner.fingerprint(p)
			|| process.processFingerprint!=materia.tooling.process.CastingProcessAnalysis.fingerprint(p,process.spec,s.gravityMps2,s)))
			throw "Stale fabrication process report";
		checks.push(process==null
			? materia.tooling.process.CastingProcessAnalysis.compoundPending(p.document.input.id)
			: process.compound);
		var loads = ToolingLoads.report(p, s);
		checks = checks.concat(loads.validation);
		for (c in p.changeRequests)
			checks.push(ToolingLoads.check("fabrication.change." + c.id, c.sourceIds, Pending, c.reason, []));
		for (f in p.formers)
			checks.push(ToolingLoads.check("fabrication.former-machining." + f.volume.id, [f.volume.id], Pending,
				"Three-dimensional former blanking, setups and fixture access require a manufacturing route; panel CAM does not cover these solids.", []));
		for (f in p.fixtures)
			checks.push(ToolingLoads.check("fabrication.fixture-machining." + f.datum.id, [f.datum.id], Pending,
				"Datum support machining and bore-mask release require qualified shop setups.", []));
		var coupon = PanelCoupon.make(p.document.recipe.panel, s.couponClearancesMm, p.document.recipe.routerDiameter);
		var hardware = p.document.recipe.hardware;
		var locator = PanelJoints.pinBolt("locator-coupon", p.document.recipe.panel, hardware.fixturePinDiameter, p.document.recipe.jointClearance,
			hardware.secondaryPinTravel, hardware.supportClearanceDiameter, hardware.fixtureSlotToolDiameter, p.document.recipe.minimumLigament);
		var panels = [for (m in p.panels.members.concat(p.supportPanels)) m.panel].concat([coupon.receiver, coupon.plug, locator.fixed, locator.moving]);
		var ids = new Map<String, Bool>();
		for (panel in panels) {
			if (ids.exists(panel.id))
				throw "Duplicate fabrication panel ID";
			ids.set(panel.id, true);
			if (panel.stock.materialSpec != p.document.recipe.panel.materialSpec
				|| panel.stock.measuredThickness != p.document.recipe.panel.measuredThickness)
				throw "Mixed panel stocks need separate blank layouts";
		}
		var requirements = [for (panel in panels) requirement(panel, s.blankMarginMm, s.sheetMaterialId)];
		requirements.sort((a, b) -> Reflect.compare(a.id, b.id));
		var placements = layout(requirements, p.document.recipe.panel.width, p.document.recipe.panel.height, s.blankKerfMm);
		checks.sort((a, b) -> Reflect.compare(a.rule, b.rule));
		var source = CastingMoldPlanner.fingerprint(p);
		return {
			lengthUnit: "mm",
			sourceFingerprint: source,
			fingerprint: haxe.crypto.Sha256.encode(ToolingCodec.canonical({
				source: source,
				spec: s,
				process: process==null ? null : process.processFingerprint
			})),
			spec: s,
			panelRequirements: requirements,
			sheetStock: {
				id: materia.tooling.ToolingIds.derive("", p.document.recipe.id, "-sheet-stock"),
				revision: p.document.recipe.revision,
				materialId: s.sheetMaterialId,
				lengthUnit: "mm",
				width: p.document.recipe.panel.width,
				height: p.document.recipe.panel.height,
				thickness: p.document.recipe.panel.measuredThickness
			},
			blankPlacements: placements,
			coupon: coupon,
			locatorCoupon: locator,
			loads: loads,
			validation: checks,
			fabrication: ToolingReadiness.evaluate(checks),
			replacementGroups: [
				{
					id: "panels-and-coupons",
					use: Reusable,
					memberIds: [for (v in requirements) v.partId]
				},
				{id: "formers", use: Reusable, memberIds: [for (v in p.formers) v.volume.id]},
				{id: "datum-supports", use: Reusable, memberIds: [for (v in p.fixtures) v.datum.id]},
				{id: "hardware", use: Reusable, memberIds: [for (v in p.hardware) v.id]},
				{id: "eps-cores", use: PerCasting, memberIds: [for (v in p.cores) v.id]},
				{
					id: "embedded-inserts",
					use: PerCasting,
					memberIds: [
						for (v in p.document.input.inserts)
							materia.tooling.ToolingIds.derive("insert-", v.id)
					]
				}
			]
		};
	}

	public static function csv(rows:Array<Array<String>>):String {
		return [
			for (row in rows) [for (value in row) '"' + StringTools.replace(value, '"', '""') + '"'].join(",")
		].join("\n") + "\n";
	}
	public static function effort(p:CastingMoldPlan, fabrication:FabricationSpec):MoldEffortMetrics {
		return effortFrom(p, ToolingFabrication.plan(p, fabrication));
	}

	static function effortFrom(p:CastingMoldPlan, f:FabricationPlan):MoldEffortMetrics {
		var panels = p.panels.members.concat(p.supportPanels),
			unique:Array<String> = [];
		var blanks = 0.0, sheets = 0, eps = 0.0;
		for (m in panels) {
			var panel = m.panel;
			var key = ToolingCodec.canonical({
				stock: panel.stock,
				outline: PanelContours.outside(panel),
				operations: [
					for (op in panel.operations)
						{
							kind: op.kind,
							contour: op.contour,
							depth: op.depth,
							through: op.through,
							side: op.side,
							toolDiameter: op.toolDiameter,
							accessVia: op.accessVia
						}
				]
			});
			if (unique.indexOf(key) < 0)
				unique.push(key);
		}
		// Exclude trial coupons from production effort, retain their layouts separately.
		var production = f.panelRequirements.filter(r -> panels.filter(m -> m.panel.id == r.partId).length > 0);
		for (r in production)
			blanks += r.blankWidth * r.blankHeight;
		var layout = ToolingFabrication.layout(production, p.document.recipe.panel.width, p.document.recipe.panel.height, f.spec.blankKerfMm);
		for (placement in layout)
			sheets = Std.int(Math.max(sheets, placement.sheet));
		for (c in p.cores)
			for (layer in c.layers) eps += layer.block.width*layer.block.depth*layer.block.height;
		var mixVolume=CastHandling.resinVolume(p.document);
		return {
			panels: panels.length,
			uniquePanels: unique.length,
			sheetCount: sheets,
			blankSheetUtilization: sheets == 0 ? 0 : blanks / (sheets * p.document.recipe.panel.width * p.document.recipe.panel.height),
			formers: p.formers.length,
			epsCores: p.cores.length,
			sacrificialEPSVolumeMm3: eps,
			logicalHardwareStacks: p.hardware.length,
			castingMassKg: p.handling.castingMassKg,
			mixMassKg: mixVolume * p.document.recipe.mixDensityKgM3 * 1e-9,
			openChangeRequests: p.changeRequests.length,
			requiresFlip: p.handling.requiresFlip
		};
	}


}
