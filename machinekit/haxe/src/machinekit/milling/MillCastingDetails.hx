package machinekit.milling;

/** Explicit mill casting intent, in mm; no Boolean extension margins. */
class MillCastingDetails {
	public var shoulderClearance:Float = 12;
	public var shoulderDepth:Float = 20;
	public var undersidePocketSetback:Float = 160;
	public var driveBearingSeatWidth:Float = 100;
	public var footEndSetback:Float = 80;
	public var footAccessDepth:Float = 90;
	public var footAccessHeight:Float = 100;
	public var footAccessFloor:Float = 40;
	public var footInnerClearance:Float = 1;
	public var footStemSize:Float = 20;
	public var footStemTop:Float = 41;
	public var frontBearingSeatWidth:Float = 106;
	public var rearBayDesignClearance:Float = 1;
	public var levelingClearanceDiameter:Float = 18;
	public var selectedShaftClearance:Float = 22;
	public var hatchAnchorWidth:Float = 20;
	public var hatchAnchorDepth:Float = 20;
	public var hatchAnchorThickness:Float = 10;
	public var hatchAnchorSideOffset:Float = 75;
	public var hatchAnchorRearInset:Float = 8;
	public var zStripBearingExtension:Float = 20;
	public var columnBoltAccessHalfWidth:Float = 20;
	public var columnBoltAccessDepth:Float = 46;
	public var zBearingAnchorOffset:Float = 10;
	public var columnAnchorThread:machinekit.design.ThreadedHolePolicy.ThreadedHoleChoice;
	public var hatchAnchorThread:machinekit.design.ThreadedHolePolicy.ThreadedHoleChoice;

	public function new() {
		columnAnchorThread = machinekit.structural.InsertHoles.metricThread("M12");
		hatchAnchorThread = machinekit.structural.InsertHoles.metricThread("M4");
	}
}
