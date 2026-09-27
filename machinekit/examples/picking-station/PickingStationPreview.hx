import haxe.io.Bytes;
import materia.project.MaterialLibrary;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;
import materia.sheet.SheetPartPlacement;
import materia.sheet.SheetPlanValidator;
import materia.sheet.SheetProjectRecord;
import materia.sheet.SheetCutRegion;
import materia.units.LengthUnit;
import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;

/** Materia preview: stock and cutting layout below, finished CAD panels above it. */
class PickingStationPreview {
	public static function layout():Bytes {
		var record:SheetProjectRecord = PickingStationDesign.initialRecord();
		var stock = record.stockSpecs[0], plan = record.plans[0];
		var requirements = [for (item in record.requirements)
			if (item.id != "tote-divider") item];
		var validation = SheetPlanValidator.validate(plan, stock, requirements);
		if (!validation.valid) throw "Picking-station example layout failed: " + validation.messages.join("; ");
		var parts:Array<SceneArtifactPart> = [];
		var material = MaterialLibrary.require(stock.materialId), color = material.visual.baseColor;
		addPart(parts, "physical-sheet", "Selected sheet · 2440 × 1220 × 18 mm",
			Part.box(stock.width, stock.height, stock.thickness, Align.Min, Align.Min, Align.Min), stock.materialId);
		for (placement in plan.placements) addPlacement(parts, placement, stock.thickness, stock.materialId);
		for (region in validation.regions) if (region.retained)
			addPart(parts, "remnant-" + region.id,
				'Reusable remnant ${region.id} · ${fmt(region.widthMm)} × ${fmt(region.heightMm)} mm',
				Part.box(region.widthMm, region.heightMm, 2, Align.Min, Align.Min, Align.Min)
					.translated(new Vector(region.xMm, region.yMm, stock.thickness + 1)), stock.materialId);

		var rowY = stock.height + 100;
		var panelPositions = [
			{partId: "picking-bench/top", x: 20.0, y: rowY},
			{partId: "picking-bench/back", x: 960.0, y: rowY},
			{partId: "picking-bench/shelf", x: 20.0, y: rowY + 650},
			{partId: "picking-bench/side", x: 940.0, y: rowY + 650},
			{partId: "tote-station/divider", x: 1530.0, y: rowY + 650}
		];
		for (entry in panelPositions) {
			var panel = PickingStationDesign.panelByPartId(entry.partId);
			var geometry = panel.geometry(ComponentDetail.Preview);
			geometry = geometry.translated(new Vector(entry.x + panel.width / 2,
				entry.y + panel.height / 2, stock.thickness + 1));
			addPart(parts, "finished-" + entry.partId, 'Finished CAD · ${panel.designation}',
				geometry, panel.materialId);
		}
		return SceneArtifact.encode({lengthUnit: "mm", metresPerUnit: LengthUnit.metresPerUnit("mm"),
			parts: parts});
	}

	static function addPlacement(parts:Array<SceneArtifactPart>, placement:SheetPartPlacement,
			thickness:Float, materialId:String):Void {
		var part = Part.box(placement.width, placement.height, thickness,
			Align.Min, Align.Min, Align.Min).translated(new Vector(placement.x,
			placement.y, thickness + 1));
		addPart(parts, "blank-" + placement.id,
			'Rectangular blank ${placement.id} · ${fmt(placement.width)} × ${fmt(placement.height)} mm',
			part, materialId);
	}

	static function addPart(parts:Array<SceneArtifactPart>, id:String, name:String,
			part:Part, materialId:String):Void {
		var material = MaterialLibrary.require(materialId), color = material.visual.baseColor;
		try {
			var properties = part.massProperties(), mesh = part.shape.tessellateRelative();
			parts.push({id: id, name: name, red: color[0], green: color[1], blue: color[2],
				appearance: MaterialLibrary.appearance(materialId), materialId: materialId,
				vertexCount: mesh.vertexCount, indexCount: mesh.indexCount, volume: properties.volume,
				centerOfMass: [properties.centerOfMass.x, properties.centerOfMass.y,
					properties.centerOfMass.z], vertices: mesh.vertices, normals: mesh.normals,
				indices: mesh.indices, edgeSegments: mesh.edgeSegments, edgeIds: mesh.edgeIds,
				faceRanges: [for (range in mesh.faceRanges) {faceIndex: range.faceIndex,
					firstIndex: range.firstIndex, indexCount: range.indexCount}]});
		} catch (error:Dynamic) {
			part.close();
			throw error;
		}
		part.close();
	}
	static function fmt(value:Float):String return Std.string(Math.round(value * 10) / 10);
}

function main():Void {
	var preview = SceneArtifact.decode(PickingStationPreview.layout());
	if (preview.parts.length < 10) throw "Picking-station preview is missing planned stock or finished panels";
	Sys.println('Generated ${preview.parts.length} picking-station and cut-layout preview parts');
}
