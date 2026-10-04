package cadkit.sheetmetal;

import materia.sheet.SheetPartRequirement;

/** Rectangular stock requirement derived from the exact flat pattern and explicit production context. */
class SheetBlank {
	public static function requirement(flat:FlatPattern, order:SheetBlankOrder):SheetPartRequirement {
		if (order.id == null || StringTools.trim(order.id).length == 0 || order.partId == null || StringTools.trim(order.partId).length == 0
			|| order.revision < 1 || order.partRevision < 1 || order.quantity < 1)
			throw new SheetDiagnosis("sheet.blank-order", "A blank needs explicit requirement/part identities, positive revisions and quantity");
		var bounds = flat.bounds(); var width = bounds.maxX - bounds.minX; var height = bounds.maxY - bounds.minY;
		order.thicknessTolerance.validate(flat.material.thickness, "thickness");
		order.widthTolerance.validate(width, "width"); order.heightTolerance.validate(height, "height");
		var grained = flat.material.grainAngle != null;
		return {id: order.id, revision: order.revision, partId: order.partId, partRevision: order.partRevision, quantity: order.quantity,
			materialId: flat.material.id, lengthUnit: "mm", thickness: flat.material.thickness,
			thicknessToleranceMinus: order.thicknessTolerance.minus, thicknessTolerancePlus: order.thicknessTolerance.plus,
			blankWidth: width, blankHeight: height,
			widthToleranceMinus: order.widthTolerance.minus, widthTolerancePlus: order.widthTolerance.plus,
			heightToleranceMinus: order.heightTolerance.minus, heightTolerancePlus: order.heightTolerance.plus,
			allowedRotations: grained ? [0] : [0, 90], directionConstraint: grained ? "width-aligned" : "free"};
	}
}
