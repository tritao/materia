package cadkit.sheetmetal;

import cadkit.modeling.Vector;

/** Derived bounds; never stored in the sheet graph. */
class SheetBounds {
	public var minX(default, null):Float = Math.POSITIVE_INFINITY;
	public var minY(default, null):Float = Math.POSITIVE_INFINITY;
	public var minZ(default, null):Float = Math.POSITIVE_INFINITY;
	public var maxX(default, null):Float = Math.NEGATIVE_INFINITY;
	public var maxY(default, null):Float = Math.NEGATIVE_INFINITY;
	public var maxZ(default, null):Float = Math.NEGATIVE_INFINITY;
	public function new() {}
	public function include(point:Vector):Void {
		minX = Math.min(minX, point.x);
		minY = Math.min(minY, point.y);
		minZ = Math.min(minZ, point.z);
		maxX = Math.max(maxX, point.x);
		maxY = Math.max(maxY, point.y);
		maxZ = Math.max(maxZ, point.z);
	}
}
