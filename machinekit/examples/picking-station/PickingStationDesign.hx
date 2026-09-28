import cadkit.modeling.Part;
import materia.sheet.SheetCut;
import materia.sheet.SheetCutPlan;
import materia.sheet.SheetPartPlacement;
import materia.sheet.SheetPartRequirement;
import materia.sheet.SheetPlanValidator;
import materia.sheet.SheetProjectRecord;
import materia.sheet.SheetStockSpec;
import machinekit.component.Bom;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import pickingstation.PickingStation;
import pickingstation.PickingStationConfig;
import pickingstation.ShelfAssembly;

/** Sheet requirements derived from the virtual station, plus one authored default layout. */
class PickingStationDesign {
	public static function stock():SheetStockSpec return {id: "birch-ply-18-2440x1220",
		revision: 1, materialId: "plywood-birch", lengthUnit: "mm",
		width: 2440, height: 1220, thickness: 18,
		supplier: "Example supplier", supplierCode: "BB-18-2440-1220"};

	public static function requirements(?config:PickingStationConfig):Array<SheetPartRequirement> {
		var value = config == null ? PickingStationConfig.defaults() : config;
		return [
			requirement("bench-worktop", "operator-bench/worktop", 1, value.benchWidth, value.benchDepth),
			requirement("rack-shelf", "rack-01/shelf/panel", value.shelfCount,
				value.shelfWidth, value.shelfDepth),
			requirement("rack-lip", "rack-01/shelf/front-lip", value.shelfCount,
				value.shelfWidth, ShelfAssembly.RETAINING_LIP_HEIGHT)
		];
	}

	static function requirement(id:String, partId:String, quantity:Int, width:Float,
		height:Float):SheetPartRequirement return {
		id: id, revision: 1, partId: partId, partRevision: 1, quantity: quantity,
		materialId: "plywood-birch", lengthUnit: "mm", thickness: 18,
		thicknessToleranceMinus: 0.3, thicknessTolerancePlus: 0.3,
		blankWidth: width, blankHeight: height,
		widthToleranceMinus: 0.5, widthTolerancePlus: 0.5,
		heightToleranceMinus: 0.5, heightTolerancePlus: 0.5,
		allowedRotations: [0, 90], directionConstraint: "free"
	};

	/** The manual layout is authored for the default two-shelf configuration only. */
	public static function plans(sheet:SheetStockSpec, requirements:Array<SheetPartRequirement>,
		?config:PickingStationConfig):Array<SheetCutPlan> {
		var value = config == null ? PickingStationConfig.defaults() : config;
		if (value.shelfCount != 2 || value.benchWidth != 900 || value.benchDepth != 600 ||
			value.shelfWidth != 740 || value.shelfDepth != 340)
			return [];
		return [{schemaVersion: 1, id: "station-default-sheet-1", revision: 1,
			stockSpecId: sheet.id, stockSpecRevision: sheet.revision,
			stockSpecFingerprint: SheetPlanValidator.stockFingerprint(sheet),
			lengthUnit: "mm", kerf: 3, edgeMargin: 10,
			requirements: SheetPlanValidator.captureRequirements(requirements),
			placements: [
				placement("worktop", "bench-worktop", 10, 10, 900, 600, "worktop"),
				placement("shelf-01", "rack-shelf", 913, 10, 740, 340, "shelf-01"),
				placement("shelf-02", "rack-shelf", 913, 353, 740, 340, "shelf-02"),
				placement("lip-01", "rack-lip", 1656, 10, 740, 55, "lip-01"),
				placement("lip-02", "rack-lip", 1656, 68, 740, 55, "lip-02")
			],
			cuts: [
				cut("cut-01", "stock", "vertical", 900, "worktop-column", "right"),
				cut("cut-02", "worktop-column", "horizontal", 600, "worktop", "worktop-remnant"),
				cut("cut-03", "right", "vertical", 740, "shelf-column", "lip-right"),
				cut("cut-04", "shelf-column", "horizontal", 340, "shelf-01", "shelf-rest"),
				cut("cut-05", "shelf-rest", "horizontal", 340, "shelf-02", "shelf-remnant"),
				cut("cut-06", "lip-right", "vertical", 740, "lip-column", "lip-strip"),
				cut("cut-07", "lip-column", "horizontal", 55, "lip-01", "lip-rest"),
				cut("cut-08", "lip-rest", "horizontal", 55, "lip-02", "lip-remnant")
			], retainedRegionIds: ["worktop-remnant", "shelf-remnant", "lip-remnant", "lip-strip"]}];
	}

	static function placement(id:String, requirementId:String, x:Float, y:Float,
		width:Float, height:Float, regionId:String):SheetPartPlacement return {
		id: id, requirementId: requirementId, x: x, y: y, width: width, height: height,
		rotation: 0, regionId: regionId};
	static function cut(id:String, regionId:String, axis:String, position:Float,
		first:String, second:String):SheetCut return {id: id, regionId: regionId,
		axis: axis, position: position, firstRegionId: first, secondRegionId: second};

	public static function initialRecord(?config:PickingStationConfig):SheetProjectRecord {
		var value = config == null ? PickingStationConfig.defaults() : config;
		var sheet = stock(), items = requirements(value);
		return {schemaVersion: 1, stockSpecs: [sheet], requirements: items,
			plans: plans(sheet, items, value), stockPieces: [], executions: []};
	}

	public static function finishedBom(?config:PickingStationConfig):Bom {
		var value = config == null ? PickingStationConfig.defaults() : config;
		var result = new PickingStation(value).billOfMaterials();
		for (item in requirements(value)) result.addComponent(panelByPartId(item.partId, value), item.quantity);
		return result;
	}

	public static function panelByPartId(id:String, ?config:PickingStationConfig):PickingStationPanel {
		for (item in requirements(config)) if (item.partId == id)
			return new PickingStationPanel(item.partId, item.blankWidth, item.blankHeight, item.thickness);
		throw 'No sheet panel geometry for part "$id"';
	}
}

/** Flat finished panel whose envelope is its rectangular cutting blank. */
class PickingStationPanel extends MachineComponent {
	public final partId:String;
	public final width:Float;
	public final height:Float;
	public final thickness:Float;

	public function new(partId:String, width:Float, height:Float, thickness:Float) {
		super('SHEET-${partId}-${width}x${height}', 'Finished flat panel $partId', "birch plywood");
		this.partId = partId; this.width = width; this.height = height; this.thickness = thickness;
	}

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.box(width, height, thickness);
}
