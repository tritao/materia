package machinekit.milling;

/** Explicit mill casting intent, in mm; no Boolean extension margins. */
class MillCastingDetails {
	public var shoulderClearance:Float = 12;
	public var shoulderDepth:Float = 20;
	public var footAccessDepth:Float = 90;
	public var footAccessHeight:Float = 100;
	public var footAccessFloor:Float = 40;
	public var footInnerClearance:Float = 1;
	public var footStemSize:Float = 20;
	public var footStemTop:Float = 42;
	public var frontBearingSeatWidth:Float = 106;
	public var rearBayDesignClearance:Float = 1;
	public var levelingClearanceDiameter:Float = 18;
	public var columnAnchorThread:String = "M12";
	public var hatchAnchorThread:String = "M4";

	public function new() {}
}
