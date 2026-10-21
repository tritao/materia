package machinekit.milling;

/** Explicit mill casting intent, in mm; no Boolean extension margins. */
class MillCastingDetails {
	/** Source design rule, shared by all base and column inserts. A minimum
	 * pair resists rotation; long seats add stations at four short-face widths.
	 * Geometric minima do not establish compound-specific anchor capacity. */
	public var anchorRule:machinekit.structural.InsertAnchorPattern.InsertAnchorPatternRule = {
		minimumCount: 2, maximumPitchInShortWidths: 4,
		embedmentDiameters: 2, coverDiameters: 1, spacingDiameters: 0.5,
		screwLengthsMm: [10,12,16,20,25,30,35,40,45,50,60,70,80]
	};
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
	public var footStemTop(get, never):Float;
	function get_footStemTop():Float return footAccessFloor;
	public var frontBearingSeatWidth:Float = 106;
	public var rearBayDesignClearance:Float = 1;
	public var levelingClearanceDiameter:Float = 18;
	/** Nominal complete foot in its floor frame; the assembly mates its 90 mm datum to the seat. */
	public var levelingFoot:machinekit.structural.LevelingFootSpec;
	public var selectedShaftClearance:Float = 22;
	/** 30 mm stock accommodates the selected M12 service screw, full thread,
	 * bottoming-tap runout, tip gap and intact policy back wall. */
	public var columnAnchorThickness:Float = 30;
	public var columnBoltLengthIncrement:Float = 5;
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

	/** Joint-pad minimum derived from access, policy edge wall, washer and cover. */
	public function minimumColumnPadDepth(thicknessMm:Float):Float
		return machinekit.structural.InsertAnchorPattern.minimumBandDepth(columnBoltAccessDepth,thicknessMm,C45Bright,anchorRule);

	public function new() {
		levelingFoot = Foot(70, 20, machinekit.standard.FastenerThreads.metric(16, 2),
			20, 115, 90, Nominal, RoundCollar(28, 10, 90));
		columnAnchorThread = machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(12).mating());
		hatchAnchorThread = machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(4).mating());
	}
}
