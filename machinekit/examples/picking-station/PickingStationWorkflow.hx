import haxe.Json;
import materia.sheet.SheetInventory;
import materia.sheet.SheetPlanExporter;
import materia.sheet.SheetPlanValidation;
import materia.sheet.SheetPlanValidator;
import materia.sheet.SheetProjectCodec;
import materia.project.SceneArtifact;
import machinekit.component.BomItem;
import sys.FileSystem;
import sys.io.AtomicFile;

/** Small local command-line interface around the versioned picking-station sheet project. */
class PickingStationWorkflow {
	static function main():Int {
		var args = Sys.args();
		try {
			if (args.length == 0) return usage();
			switch (args[0]) {
				case "init": init(required(args, 1));
				case "register":
					var inventory = SheetInventory.open(required(args, 1));
					var piece = inventory.registerSheet(required(args, 2), "birch-ply-18-2440x1220",
						null, null, null, null, args.length > 3 ? args[3] : null,
						args.length > 4 ? args[4] : null);
					Sys.println('Registered ${piece.id} (${piece.width} × ${piece.height} × ${piece.thickness} ${piece.lengthUnit})');
				case "allocate":
					var inventory = SheetInventory.open(required(args, 1));
					inventory.allocate(required(args, 2), required(args, 3));
					Sys.println('Allocated ${args[2]} to ${args[3]}');
				case "cancel":
					var inventory = SheetInventory.open(required(args, 1));
					inventory.cancelAllocation(required(args, 2), required(args, 3));
					Sys.println('Released ${args[2]}');
				case "preview": preview(required(args, 1), required(args, 2), required(args, 3),
					args.length > 4 ? args[4] : null);
				case "execute":
					if (args.length != 6 || args[5] != "CONFIRM")
						throw 'Usage: execute <project> <piece-id> <plan-id> <execution-id> CONFIRM';
					var inventory = SheetInventory.open(args[1]);
					var execution = inventory.execute(args[3], args[2], args[4], true);
					Sys.println('Recorded ${execution.id}: consumed ${execution.consumedPieceId}, produced ${execution.blanks.length} blanks and ${execution.remnantPieceIds.length} remnants');
				case "status": status(required(args, 1));
				case "bom": bom();
				case "layout-check": layoutCheck();
				case "layout-artifact": layoutArtifact(required(args, 1));
				case "demo": demo(required(args, 1), required(args, 2));
				default: return usage();
			}
			return 0;
		} catch (error:Dynamic) {
			Sys.stderr().writeString("Picking-station sheet workflow: " + Std.string(error) + "\n");
			return 1;
		}
	}

	static function init(path:String):Void {
		if (FileSystem.exists(path)) throw 'Refusing to replace existing project "$path"';
		AtomicFile.create(path, SheetProjectCodec.encode(PickingStationDesign.initialRecord()));
		Sys.println('Created ${FileSystem.fullPath(path)}');
	}

	static function preview(path:String, pieceId:String, planId:String, exportPrefix:Null<String>):Void {
		var inventory = SheetInventory.open(path), validation = inventory.preview(planId, pieceId);
		printValidation(validation);
		if (!validation.valid) throw "Plan cannot be exported until validation passes";
		if (exportPrefix != null) {
			var targetPlan = inventory.plan(planId), stock = inventory.stockSpec(targetPlan.stockSpecId);
			var requirements = [for (reference in targetPlan.requirements) inventory.requirement(reference.id)];
			AtomicFile.write(exportPrefix + ".csv", SheetPlanExporter.csv(targetPlan, stock,
				requirements, validation, inventory.piece(pieceId)));
			AtomicFile.write(exportPrefix + ".svg", SheetPlanExporter.svg(targetPlan, stock,
				requirements, validation, inventory.piece(pieceId)));
			Sys.println('Exported ${exportPrefix}.csv and ${exportPrefix}.svg');
		}
	}

	static function printValidation(value:SheetPlanValidation):Void {
		Sys.println('Validation: ${value.valid ? "valid" : "invalid"}');
		Sys.println('Produced part area: ${round(value.partAreaMm2)} mm²');
		Sys.println('Retained remnant area: ${round(value.retainedAreaMm2)} mm²');
		Sys.println('Kerf loss: ${round(value.kerfLossAreaMm2)} mm²');
		Sys.println('Discarded material, including margins: ${round(value.discardedAreaMm2)} mm²');
		Sys.println('Area balance error: ${round(value.areaBalanceErrorMm2)} mm²');
		for (message in value.messages) Sys.println('  - $message');
	}

	static function status(path:String):Void {
		var inventory = SheetInventory.open(path);
		Sys.println('Stock pieces: ${inventory.record.stockPieces.length}');
		for (piece in inventory.record.stockPieces)
			Sys.println('${piece.id} · ${piece.state} · ${piece.width} × ${piece.height} × ${piece.thickness} ${piece.lengthUnit}' +
				(piece.parentPieceId == null ? "" : ' · from ${piece.parentPieceId}'));
		Sys.println('Executions: ${inventory.record.executions.length}');
		for (execution in inventory.record.executions)
			Sys.println('${execution.id} · ${execution.blanks.length} blanks · ${execution.remnantPieceIds.length} remnants');
	}

	static function bom():Void {
		Sys.println("Finished-part BOM (separate from consumed sheet stock):");
		for (line in PickingStationDesign.finishedBom().lines())
			Sys.println('${line.partNumber},${line.quantity},${line.material},${line.description}');
	}

	static function layoutCheck():Void {
		var preview = SceneArtifact.decode(PickingStationPreview.layout());
		if (preview.parts.length != 15) throw "Expected 15 preview solids, found " + preview.parts.length;
		Sys.println("CAD layout preview contains " + preview.parts.length +
			" stock, blank, remnant, and finished-panel solids");
	}

	static function layoutArtifact(path:String):Void {
		var artifact = PickingStationPreview.layout();
		AtomicFile.writeBytes(path, artifact);
		Sys.println("Wrote planning-preview artifact " + FileSystem.fullPath(path) +
			" (" + artifact.length + " bytes)");
	}

	/** Runs and persists the acceptance sequence, including reopening and remnant reuse. */
	static function demo(path:String, exportDirectory:String):Void {
		init(path);
		var inventory = SheetInventory.open(path);
		inventory.registerSheet("sheet-001", "birch-ply-18-2440x1220", null, null,
			null, null, "batch-demo", "sheet-rack-A");
		inventory.allocate("sheet-001", "picking-bench-sheet-1");
		var firstValidation = inventory.preview("picking-bench-sheet-1", "sheet-001");
		printValidation(firstValidation);
		if (!firstValidation.valid) throw "Picking-bench example plan is invalid";
		FileSystem.createDirectory(exportDirectory);
		var firstPlan = inventory.plan("picking-bench-sheet-1"), firstStock = inventory.stockSpec(firstPlan.stockSpecId);
		var firstRequirements = [for (reference in firstPlan.requirements) inventory.requirement(reference.id)];
		AtomicFile.write(exportDirectory + "/picking-bench-cut-list.csv",
			SheetPlanExporter.csv(firstPlan, firstStock, firstRequirements, firstValidation, inventory.piece("sheet-001")));
		AtomicFile.write(exportDirectory + "/picking-bench-layout.svg",
			SheetPlanExporter.svg(firstPlan, firstStock, firstRequirements, firstValidation, inventory.piece("sheet-001")));
		inventory.execute(firstPlan.id, "sheet-001", "bench-cut-001", true);
		inventory = SheetInventory.open(path);
		var remnantId = "bench-cut-001/remnant/shelf-drop";
		inventory.allocate(remnantId, "tote-divider-from-shelf-drop");
		var secondValidation = inventory.preview("tote-divider-from-shelf-drop", remnantId);
		printValidation(secondValidation);
		if (!secondValidation.valid) throw "Tote-divider remnant plan is invalid";
		var secondPlan = inventory.plan("tote-divider-from-shelf-drop"), secondStock = inventory.stockSpec(secondPlan.stockSpecId);
		var secondRequirements = [for (reference in secondPlan.requirements) inventory.requirement(reference.id)];
		AtomicFile.write(exportDirectory + "/tote-divider-cut-list.csv",
			SheetPlanExporter.csv(secondPlan, secondStock, secondRequirements, secondValidation, inventory.piece(remnantId)));
		AtomicFile.write(exportDirectory + "/tote-divider-layout.svg",
			SheetPlanExporter.svg(secondPlan, secondStock, secondRequirements, secondValidation, inventory.piece(remnantId)));
		inventory.execute(secondPlan.id, remnantId, "divider-cut-001", true);
		inventory = SheetInventory.open(path);
		Sys.println('Reopened project after ${inventory.record.executions.length} executions');
		status(path);
		Sys.println('Planning exports saved below ${FileSystem.fullPath(exportDirectory)}');
	}

	static function required(args:Array<String>, index:Int):String {
		if (args.length <= index || args[index] == null || args[index].length == 0)
			throw "Missing command argument";
		return args[index];
	}
	static function round(value:Float):String return Std.string(Math.round(value * 100) / 100);
	static function usage():Int {
		Sys.println("Commands:");
		Sys.println("  init <project.json>");
		Sys.println("  register <project.json> <piece-id> [batch] [location]");
		Sys.println("  allocate <project.json> <piece-id> <plan-id>");
		Sys.println("  preview <project.json> <piece-id> <plan-id> [export-prefix]");
		Sys.println("  execute <project.json> <piece-id> <plan-id> <execution-id> CONFIRM");
		Sys.println("  cancel <project.json> <piece-id> <plan-id>");
		Sys.println("  status <project.json> | bom | layout-check | layout-artifact <file>");
		Sys.println("  demo <new-project.json> <export-directory>");
		return 0;
	}
}
