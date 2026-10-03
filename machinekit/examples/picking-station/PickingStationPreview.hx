import machinekit.assembly.AssemblyPreview;
import haxe.io.Bytes;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;
import materia.sheet.SheetPartPlacement;
import materia.sheet.SheetPlanValidator;
import materia.sheet.SheetProjectRecord;
import materia.sheet.SheetCutRegion;
import materia.units.LengthUnit;
import cadkit.modeling.Align;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import pickingstation.PickingStation;
import pickingstation.StationInstance;

/** Materia preview: stock and cutting layout below, finished CAD panels above it. */
class PickingStationPreview {
	/** Geometry artifact for the interactive rack and adjacent operator bench. */
	public static function station():Bytes {
		var station = new PickingStation();
		var model = new AssemblyModel("mm");
		var parts:Array<SceneArtifactPart> = [];
		var definitionByOccurrence = new Map<String, String>();
		var definitionByDesignation = new Map<String, String>();
		for (entry in station.instances()) {
			model.add(entry.id, entry.pose);
			var definitionId = definitionByDesignation.get(entry.component.designation);
			if (definitionId == null) {
				definitionId = entry.id;
				definitionByDesignation.set(entry.component.designation, definitionId);
				parts.push(AssemblyPreview.part(definitionId, entry.component.designation,
					entry.component.geometry(ComponentDetail.Preview), entry.component.materialId));
			}
			definitionByOccurrence.set(entry.id, definitionId);
		}
		var definition = model.definition("picking-station");
		AssemblyPreview.shareDefinitions(definition, definitionByOccurrence);
		return SceneArtifact.encode({lengthUnit: "mm",
			metresPerUnit: LengthUnit.metresPerUnit("mm"), parts: parts,
			assemblyDefinition: definition,
			assemblyState: model.initialState("picking-station").record()});
	}

	public static function layout():Bytes {
		var record:SheetProjectRecord = PickingStationDesign.initialRecord();
		var stock = record.stockSpecs[0], plan = record.plans[0];
		var requirements = record.requirements;
		var validation = SheetPlanValidator.validate(plan, stock, requirements);
		if (!validation.valid) throw "Picking-station example layout failed: " + validation.messages.join("; ");
		var parts:Array<SceneArtifactPart> = [];
		parts.push(AssemblyPreview.part("physical-sheet", "Selected sheet · 2440 × 1220 × 18 mm",
			Part.box(stock.width, stock.height, stock.thickness, Align.Min, Align.Min, Align.Min), stock.materialId));
		for (placement in plan.placements) addPlacement(parts, placement, stock.thickness, stock.materialId);
		for (region in validation.regions) if (region.retained)
			parts.push(AssemblyPreview.part("remnant-" + region.id,
				'Reusable remnant ${region.id} · ${fmt(region.widthMm)} × ${fmt(region.heightMm)} mm',
				Part.box(region.widthMm, region.heightMm, 2, Align.Min, Align.Min, Align.Min)
					.translated(new Vector(region.xMm, region.yMm, stock.thickness + 1)), stock.materialId));

		var rowY = stock.height + 100, rowX = 20.0;
		for (item in requirements) {
			var panel = PickingStationDesign.panelByPartId(item.partId);
			var geometry = panel.geometry(ComponentDetail.Preview);
			geometry = geometry.translated(new Vector(rowX + panel.width / 2,
				rowY + panel.height / 2, stock.thickness + 1));
			parts.push(AssemblyPreview.part("finished-" + item.id, 'Finished CAD · ${panel.designation}',
				geometry, panel.materialId));
			rowX += panel.width + 40;
		}
		return SceneArtifact.encode({lengthUnit: "mm", metresPerUnit: LengthUnit.metresPerUnit("mm"),
			parts: parts});
	}

	static function addPlacement(parts:Array<SceneArtifactPart>, placement:SheetPartPlacement,
			thickness:Float, materialId:String):Void {
		var part = Part.box(placement.width, placement.height, thickness,
			Align.Min, Align.Min, Align.Min).translated(new Vector(placement.x,
			placement.y, thickness + 1));
		parts.push(AssemblyPreview.part("blank-" + placement.id,
			'Rectangular blank ${placement.id} · ${fmt(placement.width)} × ${fmt(placement.height)} mm',
			part, materialId));
	}
	static function fmt(value:Float):String return Std.string(Math.round(value * 10) / 10);
}

function main():Void {
	var station = SceneArtifact.decode(PickingStationPreview.station());
	if (station.parts.length < 6 || station.assemblyDefinition == null)
		throw "Picking-station preview is missing rack, bins, or bench assembly";
	var layout = SceneArtifact.decode(PickingStationPreview.layout());
	if (layout.parts.length < 5) throw "Picking-station cut layout is missing sheet parts";
	Sys.println('Generated ${station.parts.length} station definitions and ${layout.parts.length} manufacturing preview parts');
}
