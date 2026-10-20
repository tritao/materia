package machinekit.tooling;

import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;

import materia.tooling.ToolingCodec;
import materia.tooling.export.ToolingFabrication;
import materia.tooling.export.ToolingFabrication.FabricationPlan;
import materia.tooling.export.FabricationSpecCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.panel.PanelPlan;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelGeometry;
import cadkit.drawing.Drawing2D;
import cadkit.drawing.DrawingEntity;
import cadkit.drawing.Svg;
import cadkit.drawing.Dxf;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;

enum abstract HardwareClassification(String) to String {
	var NonHardware = "non-hardware";
	var SupportedHardware = "supported-hardware";
	var UnsupportedHardware = "unsupported-hardware";
}

typedef HardwareDimensions = {
	var classification:HardwareClassification;
	var lengthMm:Null<Float>;
	var diameterMm:Null<Float>;
}

/** Lists and shop-review drawings from neutral records. No alternative SVG/DXF writers. */
class ToolingFabricationExport {
	public static function classify(component:machinekit.component.MachineComponent):HardwareDimensions {
		if (Std.isOfType(component, ToolingPanel)
			|| Std.isOfType(component, ToolingVolume)
			|| Std.isOfType(component, machinekit.structural.CastInsert))
			return {classification: NonHardware, lengthMm: null, diameterMm: null};
		if (Std.isOfType(component, machinekit.standard.SocketHeadCapScrew)) {
			var screw:machinekit.standard.SocketHeadCapScrew = cast component;
			return {classification: SupportedHardware, lengthMm: screw.length, diameterMm: screw.diameter};
		}
		if (Std.isOfType(component, machinekit.standard.FlatWasher)) {
			var washer:machinekit.standard.FlatWasher = cast component;
			return {classification: SupportedHardware, lengthMm: washer.thickness, diameterMm: washer.outerDiameter};
		}
		if (Std.isOfType(component, machinekit.standard.HexNut)) {
			var nut:machinekit.standard.HexNut = cast component;
			return {classification: SupportedHardware, lengthMm: nut.height, diameterMm: null};
		}
		if (Std.isOfType(component, machinekit.standard.SocketSetScrew)) {
			var plug:machinekit.standard.SocketSetScrew = cast component;
			return {classification: SupportedHardware, lengthMm: plug.length, diameterMm: plug.thread.majorDiameter};
		}
		if (Std.isOfType(component, ToolingPin))
			return {classification: SupportedHardware, lengthMm: component.parameterNumber("length"),
				diameterMm: Math.max(component.parameterNumber("diameter"), component.parameterNumber("headDiameter"))};
		return {classification: UnsupportedHardware, lengthMm: null, diameterMm: null};
	}

	static function text(dir:String, name:String, s:String):Void
		sys.io.File.saveContent(dir + "/" + name, s);

	static function data(dir:String, name:String, v:Dynamic):Void
		text(dir, name, materia.tooling.ToolingCodec.canonical(v) + "\n");

	static function drawing(dir:String, name:String, d:Drawing2D):Void {
		text(dir, name + ".svg", Svg.write(d));
		text(dir, name + ".dxf", Dxf.write(d));
	}

	public static function panels(p:CastingMoldPlan, f:FabricationPlan):Array<PanelPlan> {
		var result = [for (m in p.panels.members.concat(p.supportPanels)) m.panel].concat([f.coupon.receiver, f.coupon.plug, f.locatorCoupon.fixed, f.locatorCoupon.moving]);
		result.sort((a, b) -> Reflect.compare(a.id, b.id));
		return result;
	}

	public static function write(p:CastingMoldPlan, f:FabricationPlan, dir:String):Array<materia.tooling.ToolingValidation> {
		var classificationChecks:Array<materia.tooling.ToolingValidation> = [];
		var unknown:Array<{memberId:String, typeId:Null<String>}> = [];
		if (f.sourceFingerprint != materia.tooling.casting.CastingMoldCodec.fingerprint(p))
			throw "Stale fabrication plan";
		sys.FileSystem.createDirectory(dir);
		data(dir, "fabrication-plan.json", f);
		text(dir, "fabrication-spec.json", FabricationSpecCodec.encode(f.spec));
		data(dir, "load-estimates.json", f.loads);
		data(dir, "panel-blank-requirements.json", f.panelRequirements);
		data(dir, "sheet-stock.json", f.sheetStock);
		data(dir, "blank-layout.json", {
			kind: "bounding-blank-shelves",
			inventoryConsumed: false,
			contourNesting: false,
			sheetWidthMm: p.document.recipe.panel.width,
			sheetHeightMm: p.document.recipe.panel.height,
			placements: f.blankPlacements
		});
		var blankRows = [
			[
				"panel_id",
				"revision",
				"quantity",
				"material",
				"thickness_mm",
				"blank_width_mm",
				"blank_height_mm",
				"source_min_x_mm",
				"source_min_y_mm",
				"blank_margin_mm"
			]
		];
		var operations = [
			[
				"panel_id",
				"operation_id",
				"kind",
				"side",
				"setup_id",
				"depth_mm",
				"through",
				"tool_diameter_mm",
				"access_via"
			]
		];
		for (panel in panels(p, f)) {
			var requirement = f.panelRequirements.filter(r -> r.partId == panel.id)[0];
			var b = PanelGeometry.bounds(PanelContours.outside(panel));
			blankRows.push([
				panel.id,
				Std.string(panel.revision),
				"1",
				panel.stock.materialSpec,
				Std.string(panel.stock.measuredThickness),
				Std.string(requirement.blankWidth),
				Std.string(requirement.blankHeight),
				Std.string(b.x),
				Std.string(b.y),
				Std.string(f.spec.blankMarginMm)
			]);
			var rows = [operations[0]];
			var sorted = panel.operations.copy();
			sorted.sort((a, b) -> Reflect.compare(a.id, b.id));
			for (op in sorted) {
				var row = [
					panel.id,
					op.id,
					(op.kind : String),
					(op.side : String),
					op.setupId,
					Std.string(op.depth),
					Std.string(op.through),
					Std.string(op.toolDiameter),
					op.accessVia == null ? "" : op.accessVia
				];
				operations.push(row);
				rows.push(row);
			}
			text(dir, panel.id + ".operations.csv", ToolingFabrication.csv(rows));
		}
		text(dir, "panel-blanks.csv", ToolingFabrication.csv(blankRows));
		text(dir, "panel-operations.csv", ToolingFabrication.csv(operations));
		for (panel in [f.coupon.receiver, f.coupon.plug, f.locatorCoupon.fixed, f.locatorCoupon.moving]) {
			var base = PanelDrawing.drawing(panel), entities = base.entities();
			entities.push(Label(Text, new Point2(0, -22), 3, "FIT COUPON: measure stock and fit before using full mold panels"));
			drawing(dir, panel.id, new Drawing2D(entities));
			text(dir, panel.id + ".panel.json", PanelCodec.encode(panel));
			var part = ToolingPanel.fromPlan(panel).geometry();
			try
				part.shape.exportStep(dir + "/" + panel.id + ".step")
			catch (e:Dynamic) {
				part.close();
				throw e;
			}
			part.close();
		}
		var sheets = [for (v in f.blankPlacements) v.sheet];
		var count = 0;
		for (n in sheets)
			count = Std.int(Math.max(count, n));
		for (sheet in 1...(count + 1)) {
			var entities:Array<DrawingEntity> = [];
			function rectangle(x:Float, y:Float, w:Float, h:Float):Void {
				for (edge in PanelContours.rectangle(x, y, w, h))
					entities.push(Curve(Outline, Segment.line(new Point2(edge.start.x, edge.start.y), new Point2(edge.end.x, edge.end.y))));
			}
			rectangle(0, 0, p.document.recipe.panel.width, p.document.recipe.panel.height);
			for (v in f.blankPlacements)
				if (v.sheet == sheet) {
					rectangle(v.xMm, v.yMm, v.widthMm, v.heightMm);
					entities.push(Label(Text, new Point2(v.xMm + 3, v.yMm + 6), 3, v.panelId + " R" + v.rotationDegrees));
				}
			entities.push(Label(Text, new Point2(0, -12), 5,
				"REVIEW ONLY: rectangular blanks, kerf " + f.spec.blankKerfMm + " mm; routing separate; no inventory allocation"));
			drawing(dir, "blank-sheet-" + sheet, new Drawing2D(entities));
		}
		var coreRows = [
			[
				"core_id",
				"feature_id",
				"layer",
				"width_mm",
				"depth_mm",
				"height_mm",
				"adhesive_mm",
				"covering_mm",
				"consumption"
			]
		];
		for (c in p.cores)
			for (index in 0...c.foamLayers.length) {
				var b = c.foamLayers[index];
				coreRows.push([
					c.id,
					c.featureId,
					Std.string(index + 1),
					Std.string(b.width),
					Std.string(b.depth),
					Std.string(b.height),
					Std.string(c.adhesiveThickness),
					Std.string(c.coveringThickness),
					"per-casting"
				]);
			}
		text(dir, "eps-layers.csv", ToolingFabrication.csv(coreRows));
		var volumeRows = [
			[
				"member_id",
				"material",
				"piece",
				"width_mm",
				"depth_mm",
				"height_mm",
				"reusable"
			]
		];
		for (volume in [for (v in p.formers) v.volume].concat([for (v in p.fixtures) v.datum]))
			for (index in 0...volume.pieces.length) {
				var b = volume.pieces[index];
				volumeRows.push([
					volume.id,
					(volume.material : String),
					Std.string(index + 1),
					Std.string(b.width),
					Std.string(b.depth),
					Std.string(b.height),
					"true"
				]);
			}
		text(dir, "former-fixture-pieces.csv", ToolingFabrication.csv(volumeRows));

		var hardwareRows = [
			[
				"id",
				"part_number",
				"component_type",
				"outside_diameter_mm",
				"actual_length_mm",
				"planned_grip_or_length_mm",
				"quantity",
				"reusable"
			]
		];
		var bill = new machinekit.component.Bom(),
			assembly = CastingMoldCad.assembly(p);
		var members = assembly.components();
		members.sort((a, b) -> Reflect.compare(a.id, b.id));
		for (member in members) {
			var component = member.component, dimensions = classify(component);
			if (dimensions.classification == NonHardware)
				continue;
			if (dimensions.classification == UnsupportedHardware) {
				var type = component.componentType();
				unknown.push({memberId: member.id, typeId: type == null ? null : type.id});
				classificationChecks.push(materia.tooling.export.ToolingLoads.check("fabrication.hardware." + member.id, [member.id],
					materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported,
					"Unclassified hardware type is retained in the BOM; dimensions and procurement require an explicit handler.", []));
			}
			var length = dimensions.lengthMm == null ? "" : Std.string(dimensions.lengthMm),
				diameter = dimensions.diameterMm == null ? "" : Std.string(dimensions.diameterMm);
			var planned = p.hardware.filter(h -> h.id == member.id),
				item = component.bom;
			bill.addComponent(component);
			hardwareRows.push([
				member.id,
				item.partNumber,
				item.typeId == null ? "" : item.typeId,
				diameter,
				length,
				planned.length == 0 ? "" : Std.string(planned[0].length),
				Std.string(item.quantity),
				"true"
			]);
		}
		text(dir, "hardware.csv", ToolingFabrication.csv(hardwareRows));
		data(dir, "planned-hardware.json", p.hardware);
		data(dir, "core-units.json", p.coreUnits);
		data(dir, "hardware-unsupported.json", unknown);
		data(dir, "hardware-validation.json", classificationChecks);
		var lines = bill.lines();
		lines.sort((a, b) -> Reflect.compare(a.partNumber, b.partNumber));
		data(dir, "hardware-bom.json", lines);
		var bomRows = [["part_number", "description", "material", "quantity"]];
		for (line in lines)
			bomRows.push([
				line.partNumber,
				line.description,
				line.material == null ? "" : line.material,
				Std.string(line.quantity)
			]);
		text(dir, "hardware-bom.csv", ToolingFabrication.csv(bomRows));

		var insertRows = [
			[
				"insert_id",
				"role",
				"finish",
				"width_mm",
				"depth_mm",
				"height_mm",
				"holes_json",
				"quantity",
				"consumption"
			]
		];
		for (i in p.document.input.inserts) {
			var dimensions = InsertBodies.box(i.body, i.id);
			insertRows.push([
				i.id,
				(i.role : String),
				(i.finish : String),
				Std.string(dimensions.width),
				Std.string(dimensions.depth),
				Std.string(dimensions.height),
				ToolingCodec.canonical(i.holes),
				"1",
				"per-casting"
			]);
		}
		text(dir, "embedded-inserts.csv", ToolingFabrication.csv(insertRows));
		data(dir, "replacement-groups.json", f.replacementGroups);

		var trial = [
			[
				"fabrication_input_fingerprint",
				"recipe_id",
				"recipe_revision",
				"stock_measured_mm",
				"coupon_clearance_mm",
				"measured_slot_mm",
				"measured_tab_mm",
				"fit_observation",
				"pin_alignment_error_mm",
				"core_method",
				"core_slip_mm",
				"operator",
				"machine",
				"tool",
				"evidence_reference"
			]
		];
		for (clearance in f.coupon.clearances)
			trial.push([
				f.fingerprint,
				p.document.recipe.id,
				Std.string(p.document.recipe.revision),
				"",
				Std.string(clearance),
				"",
				"",
				"",
				"",
				"",
				"",
				"",
				"",
				"",
				""
			]);
		text(dir, "coupon-trial.csv", ToolingFabrication.csv(trial));
		text(dir, "coupon-instructions.md",
			"# Fit and restraint trial\n\nREVIEW ONLY. This record does not qualify the full mold.\n\nFabrication input: " + f.fingerprint +
			"\n\n1. Measure stock thickness at several locations and retain the supplier/tool references.\n2. Cut the receiver and plug from the same stock, face and tools as the planned panels. Slots fit-0 onward correspond to the ordered clearances in fabrication-plan.json.\n3. Record slot/tab dimensions and fit, with repeatable insertion and withdrawal. Record pin alignment separately using the planned round/slot locator pair.\n4. Trial both a glued bed socket and a bridge/carrier where the source cavity permits; record core movement under a measured load at least equal to its reported design uplift. Adhesive, EPS bearing, joint strength and cure exposure need their own evidence.\n5. Enter observations in coupon-trial.csv and revise the recipe explicitly. A successful coupon alone does not release the full mold.\n");
		return classificationChecks;
	}
}
