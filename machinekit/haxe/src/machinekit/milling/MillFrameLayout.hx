package machinekit.milling;

/** Functional datums supplied by the machine using the frame, in millimetres.
 * Y rails run toward +Y; column rails run toward +Z on a face normal to -Y.
 */
typedef MillFrameInterfaces = {
	final yRailSpacing:Float;
	final yRailStart:Float;
	final yRailLength:Float;
	final columnFront:Float;
	final columnHeight:Float;
	final zRailSpacing:Float;
	final zRailStart:Float;
	final zRailLength:Float;
}

typedef MillFrameOptions = {
	?baseWidth:Float;
	?baseHeight:Float;
	?columnWidth:Float;
	?columnDepth:Float;
	?wall:Float;
	?roof:Float;
	?seatWidth:Float;
	?insertThickness:Float;
	?seatRise:Float;
}

/** Mill-specific layout, independent of a particular example and of enclosure size.
 * Dimensions are a design concept, not material/process or stiffness qualification.
 */
class MillFrameLayout {
	public final yRailSpacing:Float;
	public final yRailStart:Float;
	public final yRailLength:Float;
	public final columnFront:Float;
	public final columnHeight:Float;
	public final zRailSpacing:Float;
	public final zRailStart:Float;
	public final zRailLength:Float;
	public final baseWidth:Float;
	public final baseHeight:Float;
	public final columnWidth:Float;
	public final columnDepth:Float;
	public final wall:Float;
	public final roof:Float;
	public final seatWidth:Float;
	public final insertThickness:Float;
	public final seatRise:Float;
	public final front:Float;
	public final rear:Float;
	public final baseDepth:Float;
	public final baseCentreY:Float;
	public final columnCentreY:Float;
	public final railPlane:Float;

	public function new(interfaces:MillFrameInterfaces, ?options:MillFrameOptions) {
		if (interfaces == null) throw "Mill frame needs axis interfaces";
		if (options == null) options = {};
		yRailSpacing = interfaces.yRailSpacing; yRailStart = interfaces.yRailStart; yRailLength = interfaces.yRailLength;
		columnFront = interfaces.columnFront; columnHeight = interfaces.columnHeight;
		zRailSpacing = interfaces.zRailSpacing; zRailStart = interfaces.zRailStart; zRailLength = interfaces.zRailLength;
		baseWidth = options.baseWidth ?? 520; baseHeight = options.baseHeight ?? 220;
		columnWidth = options.columnWidth ?? 320; columnDepth = options.columnDepth ?? 260;
		wall = options.wall ?? 40; roof = options.roof ?? 60; seatWidth = options.seatWidth ?? 40;
		insertThickness = options.insertThickness ?? 20; seatRise = options.seatRise ?? 4;
		for (value in [yRailSpacing, yRailStart, yRailLength, columnFront, columnHeight, zRailSpacing,
			zRailStart, zRailLength, baseWidth, baseHeight, columnWidth, columnDepth, wall, roof,
			seatWidth, insertThickness, seatRise]) if (!Math.isFinite(value)) throw "Mill frame dimensions must be finite";
		for (value in [yRailSpacing, yRailLength, columnHeight, zRailSpacing, zRailLength,
			baseWidth, baseHeight, columnWidth, columnDepth, wall, roof, seatWidth, insertThickness])
			if (value <= 0) throw "Mill frame dimensions must be positive";
		if (seatRise < 0 || seatRise >= insertThickness || insertThickness >= roof ||
			roof >= baseHeight || 3 * wall >= columnWidth || 2 * roof + wall >= columnHeight ||
			columnDepth <= 3 * roof || columnWidth > baseWidth - 2 * wall ||
			yRailSpacing <= seatWidth || yRailSpacing + seatWidth + 2 * wall >= baseWidth ||
			zRailSpacing + seatWidth >= columnWidth || zRailStart < 0 || zRailStart + zRailLength > columnHeight ||
			yRailStart + yRailLength > columnFront || columnFront - yRailStart <= 3 * wall)
			throw "Mill frame interfaces leave no continuous walls, webs or insert support";
		front = yRailStart - wall;
		// The column's finished steel face lies seatRise ahead of its casting.
		rear = columnFront + seatRise + columnDepth + wall;
		baseDepth = rear - front; baseCentreY = (front + rear) / 2;
		columnCentreY = columnFront + seatRise + columnDepth / 2;
		railPlane = baseHeight + seatRise;
	}
}
