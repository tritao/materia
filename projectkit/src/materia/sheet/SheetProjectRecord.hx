package materia.sheet;

import materia.project.MaterialDef;

/** Versioned manufacturing project. Plans are design records; pieces and executions are inventory. */
typedef SheetProjectRecord = {
	var schemaVersion:Int;
	var stockSpecs:Array<SheetStockSpec>;
	var requirements:Array<SheetPartRequirement>;
	var plans:Array<SheetCutPlan>;
	var stockPieces:Array<StockPiece>;
	var executions:Array<SheetCutExecution>;
	/** Optional project-local definitions; built-ins remain shared through MaterialLibrary. */
	@:optional var customMaterials:Array<MaterialDef>;
}
