import materia.casting.CastingInput;

/** Explicit mill casting intent, in mm; no Boolean extension margins. */
class MillCastingDetails {
	/** Source design rule, shared by all base and column inserts. A minimum
	 * pair resists rotation; long seats retain the four-short-width station limit.
	 * Demand is class-8.8 service preload plus a two-load-path lifting share,
	 * multiplied by the named 1.5 design reserve. Mix strengths remain inputs.
	 * Geometric minima do not establish compound-specific anchor capacity. */
	public var anchorRule:materia.casting.InsertAnchorage.InsertAnchorPatternRule = {
		id: "mill-bolted-anchor-demand-v1", demandFactor: 1.5, maximumCount: 64,
		liftingLoadPaths: 2, gravityMps2: 9.80665,
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
	public var hatchAnchorWidth:Float = 44;
	public var hatchAnchorDepth:Float = 20;
	public var hatchAnchorThickness:Float = 12;
	public var hatchAnchorSideOffset:Float = 75;
	/** Existing service face remains 2 mm behind the service-bay datum. */
	public var hatchAnchorServiceFaceOffset:Float = 2;
	public var zStripBearingExtension:Float = 20;
	public var columnBoltAccessHalfWidth:Float = 20;
	public var columnBoltAccessDepth:Float = 46;
	public var zBearingAnchorOffset:Float = 10;
	/** RM0 accuracy allocations are user inputs, not inferred design-hole limits. */
	public var railHoleBudget:machinekit.structural.InsertMountPattern.MountHoleErrorAllocation = {
		placementOffsetMm:Unresolved("RM0 insert placement offset"),placementAngleRad:Unresolved("RM0 insert angular placement"),assemblyAdjustmentMm:Unresolved("RM0 assembly lateral adjustment"),cureBowMm:Unresolved("RM0 cure bow"),
		machiningErrorMm:Unresolved("RM0 drill and guide accuracy"),pitchAccumulationMm:Unresolved("RM0 mounting pitch accumulation"),requiredMarginMm:Unresolved("RM0 required clearance margin")
	};
	public var permittedRailHoleRoutes:Array<materia.casting.InsertManufacturing.InsertHoleRoute> = [
		materia.casting.InsertManufacturing.InsertHoleRoute.BeforeCasting,
		materia.casting.InsertManufacturing.InsertHoleRoute.TransferAfterCasting
	];
	/** Source lifting connections; thread capacity and rigging remain unqualified. */
	public var liftingPlateWidthMm:Float = 40;
	public var liftingPlateDepthMm:Float = 100;
	public var liftingPlateThicknessMm:Float = 30;
	/** Alternative aspect ratio for corners bounded by adjacent interface cuts.
	 * The source builder admits the smallest passing declared blank. */
	public var liftingAlternativeFootprintsMm:Array<{width:Float,depth:Float}> = [{width:71,depth:45}];
	/** Fixed service-hole datums measured inward from the body sides.
	 * Plate growth and anchor margins must not move these connections. */
	public var liftingServiceInsetX:Float = 22;
	public var liftingServiceInsetY:Float = 27;
	public var liftingThread:machinekit.design.ThreadedHolePolicy.ThreadedHoleChoice;
	public var columnAnchorThread:machinekit.design.ThreadedHolePolicy.ThreadedHoleChoice;
	public var hatchAnchorThread:machinekit.design.ThreadedHolePolicy.ThreadedHoleChoice;

	/** A centred clearance bore retains the full policy wall on both sides. */
	public function minimumClearancePadSide(diameterMm:Float,thicknessMm:Float):Float {
		var minimum=machinekit.tooling.InsertAnchorGeometry.minimumClearancePadSide(diameterMm,thicknessMm,C45Bright);
		// Source blanks are specified in whole millimetres, outward from the
		// exact geometric floor; tangent bores are not rounded inward.
		return Math.ceil(minimum);
	}

	/** Absolute joint-pad floor from access, policy edge wall, flange and cover.
	 * The final demand-sized pattern must still pass native containment. */
	public function minimumColumnPadDepth(thicknessMm:Float):Float
		return machinekit.tooling.InsertAnchorGeometry.minimumBandDepth(columnBoltAccessDepth,thicknessMm,C45Bright,anchorRule);

	public function new() {
		levelingFoot = Foot(70, 20, machinekit.standard.FastenerThreads.metric(16, 2),
			20, 115, 90, Nominal, RoundCollar(28, 10, 90));
		liftingThread = machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(12).mating());
		columnAnchorThread = machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(12).mating());
		hatchAnchorThread = machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(4).mating());
	}
}
