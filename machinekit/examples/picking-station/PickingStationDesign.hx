import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import materia.project.MaterialLibrary;
import materia.sheet.SheetCut;
import materia.sheet.SheetCutPlan;
import materia.sheet.SheetPartPlacement;
import materia.sheet.SheetPartRequirement;
import materia.sheet.SheetPlanValidator;
import materia.sheet.SheetProjectRecord;
import materia.sheet.SheetRequirementReference;
import materia.sheet.SheetStockSpec;
import machinekit.component.Bom;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Authored picking-station panel requirements, manual layouts, and CAD panel definitions. */
class PickingStationDesign {
	public static function stock():SheetStockSpec return {id: "birch-ply-18-2440x1220",
		revision: 1, materialId: "plywood-birch", lengthUnit: "mm",
		width: 2440, height: 1220, thickness: 18,
		supplier: "Example supplier", supplierCode: "BB-18-2440-1220"};

	public static function requirements():Array<SheetPartRequirement> return [
		requirement("bench-top", "picking-bench/top", 1, 900, 600, [0, 90], false),
		requirement("bench-back", "picking-bench/back", 1, 900, 500, [0, 90], false),
		requirement("bench-shelf", "picking-bench/shelf", 1, 880, 450, [0, 90], false),
		requirement("bench-side", "picking-bench/side", 2, 550, 450, [0, 90], false),
		requirement("tote-divider", "tote-station/divider", 2, 850, 300, [0, 90], true)
	];

	static function requirement(id:String, partId:String, quantity:Int, width:Float,
		height:Float, rotations:Array<Int>, widthAligned:Bool):SheetPartRequirement return {
		id: id, revision: 1, partId: partId, partRevision: 1, quantity: quantity,
		materialId: "plywood-birch", lengthUnit: "mm", thickness: 18,
		thicknessToleranceMinus: 0.3, thicknessTolerancePlus: 0.3,
		blankWidth: width, blankHeight: height,
		widthToleranceMinus: 0.5, widthTolerancePlus: 0.5,
		heightToleranceMinus: 0.5, heightTolerancePlus: 0.5,
		allowedRotations: rotations, directionConstraint: widthAligned ? "width-aligned" : "free"
	};

	public static function plans(stock:SheetStockSpec, requirements:Array<SheetPartRequirement>):Array<SheetCutPlan> {
		var first = [for (item in requirements) if (item.id != "tote-divider") item];
		var second = [for (item in requirements) if (item.id == "tote-divider") item];
		return [
			{schemaVersion: 1, id: "picking-bench-sheet-1", revision: 1,
				stockSpecId: stock.id, stockSpecRevision: stock.revision,
				stockSpecFingerprint: SheetPlanValidator.stockFingerprint(stock),
				lengthUnit: "mm", kerf: 3, edgeMargin: 10,
				requirements: SheetPlanValidator.captureRequirements(first),
				placements: [
					placement("top-blank", "bench-top", 10, 10, 900, 600, 0, "top"),
					placement("back-blank", "bench-back", 10, 613, 900, 500, 0, "back"),
					placement("shelf-blank", "bench-shelf", 913, 10, 880, 450, 0, "shelf"),
					placement("side-blank-1", "bench-side", 1796, 10, 550, 450, 0, "side-1"),
					placement("side-blank-2", "bench-side", 1796, 463, 550, 450, 0, "side-2")
				],
				cuts: [
					cut("cut-01", "stock", "vertical", 900, "left", "right"),
					cut("cut-02", "left", "horizontal", 600, "top", "left-lower"),
					cut("cut-03", "left-lower", "horizontal", 500, "back", "back-tail"),
					cut("cut-04", "right", "vertical", 880, "shelf-column", "side-column"),
					cut("cut-05", "shelf-column", "horizontal", 450, "shelf", "shelf-drop"),
					cut("cut-06", "side-column", "vertical", 550, "side-stack", "narrow-strip"),
					cut("cut-07", "side-stack", "horizontal", 450, "side-1", "side-lower"),
					cut("cut-08", "side-lower", "horizontal", 450, "side-2", "side-drop")
				],
				retainedRegionIds: ["back-tail", "shelf-drop", "narrow-strip", "side-drop"]},
			{schemaVersion: 1, id: "tote-divider-from-shelf-drop", revision: 1,
				stockSpecId: stock.id, stockSpecRevision: stock.revision,
				stockSpecFingerprint: SheetPlanValidator.stockFingerprint(stock),
				lengthUnit: "mm", kerf: 3, edgeMargin: 5,
				requirements: SheetPlanValidator.captureRequirements(second),
				placements: [
					placement("divider-blank-1", "tote-divider", 5, 5, 850, 300, 0, "divider-1"),
					placement("divider-blank-2", "tote-divider", 5, 308, 850, 300, 0, "divider-2")
				],
				cuts: [
					cut("remnant-cut-01", "stock", "vertical", 850, "divider-stack", "right-strip"),
					cut("remnant-cut-02", "divider-stack", "horizontal", 300, "divider-1", "divider-upper"),
					cut("remnant-cut-03", "divider-upper", "horizontal", 300, "divider-2", "divider-remnant")
				],
				retainedRegionIds: ["right-strip", "divider-remnant"]}
		];
	}

	static function placement(id:String, requirementId:String, x:Float, y:Float,
		width:Float, height:Float, rotation:Int, regionId:String):SheetPartPlacement return {
		id: id, requirementId: requirementId, x: x, y: y, width: width, height: height,
		rotation: rotation, regionId: regionId};
	static function cut(id:String, regionId:String, axis:String, position:Float,
		first:String, second:String):SheetCut return {id: id, regionId: regionId,
		axis: axis, position: position, firstRegionId: first, secondRegionId: second};

	public static function initialRecord():SheetProjectRecord {
		var sheet = stock(), requirements = requirements();
		return {schemaVersion: 1, stockSpecs: [sheet], requirements: requirements,
			plans: plans(sheet, requirements), stockPieces: [], executions: []};
	}

	public static function panels():Array<PickingStationPanel> return [
		new PickingStationPanel("PICK-TOP", "Bench top", "picking-bench/top", 900, 600, 18,
			[{x: -400, y: -250}, {x: 400, y: -250}, {x: -400, y: 250}, {x: 400, y: 250}]),
		new PickingStationPanel("PICK-BACK", "Bench back", "picking-bench/back", 900, 500, 18, []),
		new PickingStationPanel("PICK-SHELF", "Bench shelf", "picking-bench/shelf", 880, 450, 18, []),
		new PickingStationPanel("PICK-SIDE", "Bench side", "picking-bench/side", 550, 450, 18, [])
	];

	public static function finishedBom():Bom {
		var result = new Bom();
		for (panel in panels()) result.addComponent(panel,
			switch panel.partId { case "picking-bench/side": 2; default: 1; });
		result.addComponent(new PickingStationPanel("TOTE-DIVIDER", "Tote divider",
			"tote-station/divider", 850, 300, 18, []), 2);
		return result;
	}

	public static function panelByPartId(id:String):PickingStationPanel {
		for (panel in panels()) if (panel.partId == id) return panel;
		if (id == "tote-station/divider")
			return new PickingStationPanel("TOTE-DIVIDER", "Tote divider", id, 850, 300, 18, []);
		throw 'No CAD panel geometry for part "$id"';
	}
}

/** CAD definition connects a finished design part to its rectangular cut blank. */
class PickingStationPanel extends MachineComponent {
	public final partId:String;
	public final width:Float;
	public final height:Float;
	public final thickness:Float;
	final holeCenters:Array<{x:Float, y:Float}>;

	public function new(designation:String, description:String, partId:String, width:Float,
		height:Float, thickness:Float, holeCenters:Array<{x:Float, y:Float}>) {
		super(designation, description, "birch plywood");
		this.partId = partId; this.width = width; this.height = height; this.thickness = thickness;
		this.holeCenters = holeCenters;
	}

	/** Envelope is the cut blank; Preview is the finished CAD panel with any authored holes. */
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var blank = Part.box(width, height, thickness);
		if (detail == Envelope || holeCenters.length == 0) return blank;
		var tools:Array<Part> = [];
		for (center in holeCenters) tools.push(Solids.cylinder(4, -1, thickness + 1,
			center.x, center.y));
		return Solids.cut(blank, tools);
	}
}
