import materia.casting.CastingInput;

/** Explicit mill casting intent, in mm; no Boolean extension margins. */
class MillCastingDetails {
  public static inline var COLUMN_JOINT_GAP_MM = 3;
  public var precisionFinishingStockMm:Float = 4;
  /** Source design rule, shared by all base and column inserts. A minimum
	 * pair resists rotation; long seats retain the four-short-width station limit.
	 * Demand is class-8.8 service preload plus a two-load-path lifting share,
	 * multiplied by the named 1.5 design reserve. Mix strengths remain inputs.
	 * Geometric minima do not establish compound-specific anchor capacity. */
  public var anchorRule:materia.casting.InsertAnchorage.InsertAnchorPatternRule =
    materia.casting.InsertAnchorage.InsertAnchorSelection.sourceDemandRule("mill-bolted-anchor-demand-v1");
  public var shoulderClearance:Float = 12;
  public var shoulderDepth:Float = 18;
  public var undersidePocketSetback:Float = 160;
  public var driveBearingSeatWidth:Float = 100;
  public var footEndSetback:Float = 80;
  public var footAccessDepth:Float = 90;
  public var footAccessHeight:Float = 100;
  public var footAccessFloor:Float = 25;
  public var footStemSize:Float = 18;
  /** Removable roof plate carries compression through its upper face. */
  public var footPlateSide:Float = 60;
  public var footPlateThickness:Float = 15;
  public var footSupportNutHeight:Float = 13;
  public var footLockNutHeight:Float = 8;
  public var footNutAcrossFlats:Float = 24;
  public var footGroundClearance:Float = 50;
  public var footStemEndClearance:Float = 2;
  public var footRoof(get, never):Float;
  function get_footRoof():Float return footAccessFloor + footAccessHeight;
  public var footStemTop(get, never):Float;
  function get_footStemTop():Float return footAccessFloor;
  public var frontBearingSeatWidth:Float = 106;
  public var rearBayDesignClearance:Float = 1;
  public var levelingClearanceDiameter:Float = 18;
  /** The complete foot uses the removable plate underside as its mounting datum. */
  public var levelingFoot(get, never):machinekit.structural.LevelingFootSpec;
  public var levelingPadDiameter:Float = 70;
  public var levelingPadHeight:Float = 20;
  function get_levelingFoot():machinekit.structural.LevelingFootSpec return Foot(
    levelingPadDiameter,
    levelingPadHeight,
    levelingThread,
    levelingPadHeight,
    footGroundClearance + footRoof - footStemEndClearance,
    footGroundClearance + footRoof - footPlateThickness,
    Nominal,
    NoCollar
  );
  public var levelingThread:cadkit.threads.ThreadSpec;
  public var selectedShaftClearance:Float = 22;
  /** 30 mm stock accommodates the selected M12 service screw, full thread,
	 * bottoming-tap runout, tip gap and intact policy back wall. */
  public var columnAnchorThickness:Float = 30;
  /** Structural re-entrant fillet, independent of the board-former cutter.
	 * User correction 2026-10-26: 1.5 times the then-declared 8 mm aggregate; retained with the 5 mm mix.
	 * The step is formed by EPS; board-former radii retain their shop rule. */
  public var columnFootCornerRadiusMm:Float = 12;
  public var columnBoltLengthIncrement:Float = 5;
  public var hatchInsertDiameter:Float = 12;
  public var hatchReceiverDiameter:Float = 12.2;
  public var hatchReceiverDepth:Float = 12;
  /** In-house minimum ligament for the lightly loaded hatch receiver.
	 * Compound aggregate/strength and adhesive evidence remain independent Pending. */
  public var hatchReceiverWallDiameters:Float = 0.25;
  public var hatchReceiverBackDiameters:Float = 0.5;
  public var hatchAnchorThickness:Float = 12;
  public var hatchAnchorSideOffset:Float = 75;
  /** Existing service face remains 2 mm behind the service-bay datum. */
  public var hatchAnchorServiceFaceOffset:Float = 2;
  public var zStripBearingExtension:Float = 20;
  public var columnBoltAccessHalfWidth:Float = 20;
  /** Sole blanks meet the existing outer finished faces. Off-centre drilling
  * preserves the four clamp/jack/fixture axes while retaining the 10 mm lands. */
  public var columnSolePadWidthMm:Float = 100;
  public var columnSolePadDepthMm:Float = 125;
  public var jointToolClearanceMm:Float = 2.5;
  public var jointJackThread:machinekit.design.ThreadedHolePolicy.ThreadedHoleChoice;
  public var jointFixtureThread:machinekit.design.ThreadedHolePolicy.ThreadedHoleChoice;
  public var jointHolePitchMm(get, never):Float;
  function get_jointHolePitchMm():Float {
    var clearance = machinekit.design.ThreadedHolePolicy.clearance(columnAnchorThread.clearanceDiameter);
    var jack = machinekit.design.ThreadedHolePolicy.resolve(jointJackThread.thread, C45Bright);
    var fixture = machinekit.design.ThreadedHolePolicy.resolve(jointFixtureThread.thread, C45Bright);
    var separation = columnAnchorThread.clearanceDiameter / 2 + Math.max(
      jointJackThread.clearanceDiameter,
      jointFixtureThread.clearanceDiameter
    ) / 2 + Math.max(
      Math.max(clearance.minWall, clearance.minClearance),
      Math.max(Math.max(jack.minWall, jack.minClearance), Math.max(fixture.minWall, fixture.minClearance))
    );
    return columnBoltLengthIncrement * Math.ceil(separation / columnBoltLengthIncrement);
  }
  public var columnBoltAccessDepth(get, never):Float;
  function get_columnBoltAccessDepth():Float {
    // Reserve a normal M8 head/tool envelope even with headless jacks.
    var head = machinekit.standard.SocketHeadCapScrew.catalog().get(jointJackThread.thread.designation).headDiameter;
    return 2 * (jointHolePitchMm + head / 2 + jointToolClearanceMm);
  }
  public var zBearingAnchorOffset:Float = 10;
  /** RM0 accuracy allocations are user inputs, not inferred design-hole limits. */
  public var railHoleBudget:machinekit.structural.InsertMountPattern.MountHoleErrorAllocation = {
    placementOffsetMm: Unresolved("RM0 insert placement offset"),
    placementAngleRad: Unresolved("RM0 insert angular placement"),
    assemblyAdjustmentMm: Unresolved("RM0 assembly lateral adjustment"),
    cureBowMm: Unresolved("RM0 cure bow"),
    machiningErrorMm: Unresolved("RM0 drill and guide accuracy"),
    pitchAccumulationMm: Unresolved("RM0 mounting pitch accumulation"),
    requiredMarginMm: Unresolved("RM0 required clearance margin")
  };
  public var permittedRailHoleRoutes:Array<materia.casting.InsertManufacturing.InsertHoleRoute> = [
    materia.casting.InsertManufacturing.InsertHoleRoute.BeforeCasting,
    materia.casting.InsertManufacturing.InsertHoleRoute.TransferAfterCasting
  ];
  /** Source lifting connections; thread capacity and rigging remain unqualified. */
  public var liftingPlateWidthMm:Float = 46;
  public var liftingPlateDepthMm:Float = 100;
  public var liftingPlateThicknessMm:Float = 30;
  /** One centred blank per lifting role. Its edge land is resolved before source realization. */
  public var liftingLandStepMm:Float = 5;
  public var liftingMaximumLandMm:Float = 60;
  public var liftingThread:machinekit.design.ThreadedHolePolicy.ThreadedHoleChoice;
  public var columnAnchorThread:machinekit.design.ThreadedHolePolicy.ThreadedHoleChoice;
  public var hatchAnchorThread:machinekit.design.ThreadedHolePolicy.ThreadedHoleChoice;

  /** A centred clearance bore retains the full policy wall on both sides. */
  public function minimumClearancePadSide(diameterMm:Float, thicknessMm:Float):Float {
    var minimum = machinekit.tooling.InsertAnchorGeometry.minimumClearancePadSide(diameterMm, thicknessMm, C45Bright);
    // Source blanks are specified in whole millimetres, outward from the
    // exact geometric floor; tangent bores are not rounded inward.
    return Math.ceil(minimum);
  }

  public function new() {
    anchorRule.allowedNominalDiametersMm = CastingShopRecipe.anchorDiametersMm();
    levelingThread = machinekit.standard.FastenerThreads.metric(16, 2);
    liftingThread =
      machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(12).mating());
    columnAnchorThread =
      machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(12).mating());
    hatchAnchorThread =
      machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(4).mating());
    jointJackThread =
      machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(8).mating());
    jointFixtureThread =
      machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(6).mating());
  }
}
