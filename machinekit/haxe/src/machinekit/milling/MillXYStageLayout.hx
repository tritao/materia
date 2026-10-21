package machinekit.milling;

import machinekit.motion.*;

/** Mill-specific cross-slide datums. Y moves the saddle; X moves the table.
 * Positive work coordinates move the workpiece in the negative world direction.
 * Dimensions and motor patterns are design assumptions, not released vendor drawings.
 */
class MillXYStageLayout {
	public static inline var X_MOTOR_FACE:Float = -216;
 public static inline var X_BELT_PLANE:Float = -234;
	public static inline var SADDLE_HEIGHT:Float = 80;
	public static inline var RISER_HEIGHT:Float = 48;
	public static inline var TABLE_HEIGHT:Float = 40;
	public final frame:MillFrameLayout;
 /** HZMotion-supported Y drive; null when Y uses the same BK12 set as X. */
 public final yDrive:Null<MillScrewDrive>;
 public final supplierY:Bool;
 public final yMotor:LeadshineServo;
 public final yDriveScrew:LeadScrew;
 public final yFixedDepth:Float;
 public final yFloatingDepth:Float;
	public final centreY:Float;
	public final profile:String;
	public final block:LinearRailBlock;
	public final drive:MillXYDrive;
	public final nut:BallNut;
	public final fixed:ScrewSupportUnit;
	public final floating:ScrewSupportUnit;
	public final motor:LeadshineServo;
	public final coupling:ShaftCoupling;
	public final xScrew = new BallScrew(455);
	public final yScrew:BallScrew;
	public final xHalfTravel:Float = 125;
	public final yHalfTravel:Float;
	public final tableDepth:Float;
	public final xRailSpacing:Float = 160;
	public final xRailLength:Float = 440;
	public final saddleDepth:Float;
	public final xBlockSpan:Float;
	public final yBlockSpan:Float;
	public final saddleBottom:Float;
	public final xRailPlane:Float;
	public final tableBottom:Float;
	public final tableTop:Float;
	public final yScrewZ:Float;
	public final xScrewZ:Float;
	public final xMotorZ:Float;
	public final yScrewStart:Float;
	public final yMotorFace:Float;
	public final yServiceRear:Float;
	public final yMountPlane:Float;
	public final yBayTop:Float;
	public final yHatchLowerZ:Float;
	public final yHatchUpperZ:Float;
	public final yBayWidth:Float = 118;
	public final yChannelWidth:Float = 90;
	public final yFixedY:Float;
	public final yFloatingY:Float;
	public final xFixedX:Float = -192.5;
	public final xFloatingX:Float = 212.5;
	public final xScrewStart:Float = -230;
	public final clearance:Float = 3;
	public final xOvertravel:Float;
	public final yOvertravel:Float;

	public static function stackHeight(profile:String):Float return 2 * LinearRailBlock.metric(profile).spec.blockHeight + SADDLE_HEIGHT + RISER_HEIGHT + TABLE_HEIGHT;

	/** `drive` defaults to `MillXYDrive.bk12()`; `yDrive` replaces it on Y with an HZMotion-supported screw. */
	public function new(frame:MillFrameLayout, centreY:Float, profile:String = "HGR15", guideBlock:Null<LinearRailBlock> = null,
		?yDrive:MillScrewDrive, ?drive:MillXYDrive) {
		if (frame == null || !Math.isFinite(centreY)) throw "XY stage needs finite matching frame datums";
		this.frame = frame;
		this.drive = drive ?? MillXYDrive.bk12();
		nut = this.drive.nut; fixed = this.drive.fixed; floating = this.drive.floating; motor = this.drive.motor; coupling = this.drive.coupling;
		yHalfTravel = frame.yTravel.halfTravel;
		tableDepth = frame.yTravel.tableDepth;
		saddleDepth = frame.yTravel.saddleDepth;
  this.yDrive=yDrive;
  supplierY=yDrive!=null;
  yMotor=yDrive==null?motor:yDrive.motor;
  yFixedDepth=yDrive==null?fixed.length:yDrive.fixed.spec.length;
  yFloatingDepth=yDrive==null?floating.length:yDrive.floating.spec.length;
		this.centreY = centreY;
		this.profile = profile;
		if (guideBlock != null && guideBlock.spec.designation != profile)
			throw "Guide block must match the selected rail profile";
		block = guideBlock == null ? LinearRailBlock.metric(profile) : guideBlock;
		xBlockSpan = (block.spec.blockLength + 4) / 2;
		yBlockSpan = (block.spec.blockLength + 40) / 2;
		if (saddleDepth < 2 * yBlockSpan + block.spec.blockLength + clearance)
			throw "Y saddle is too short to support both carriage blocks";
		saddleBottom = frame.railPlane + block.spec.blockHeight;
		xRailPlane = saddleBottom + SADDLE_HEIGHT;
		tableBottom = xRailPlane + block.spec.blockHeight + RISER_HEIGHT;
		tableTop = tableBottom + TABLE_HEIGHT;
		// Put the screw at the saddle underside, between the guides.
		yScrewZ = saddleBottom;
		yMountPlane = yScrewZ - 40;
		yBayTop = yScrewZ + yMotor.spec.connectorHeight + clearance;
		yHatchLowerZ = frame.railPlane + 10; yHatchUpperZ = yBayTop + 10;
		xMotorZ = saddleBottom + SADDLE_HEIGHT / 2;
		xScrewZ = xRailPlane + 40;
		yServiceRear = frame.columnCentreY + frame.columnDepth / 2;
		// The older fabricated cage retains its 130 mm service bay and bearing datums.
		yMotorFace = yServiceRear - (supplierY?yMotor.rating.bodyLength:Math.max(130,yMotor.rating.bodyLength)) - clearance;
		if (yDrive == null) {
			yScrewStart = yMotorFace - yMotor.rating.shaftLength;
			yFixedY = yScrewStart - BallScrew.INPUT_JOURNAL_LENGTH + fixed.length / 2;
			yFloatingY = frame.yRailStart + BallScrew.JOURNAL_LENGTH / 2;
		} else {
			var shoulder = yDrive.fixed.spec.shoulderZ;
			if (shoulder == null) throw "Y drive mount needs a screw shoulder datum";
			yScrewStart = yMotorFace - (shoulder - yDrive.fixedEnd);
			yFixedY = yMotorFace - yDrive.fixed.spec.length / 2;
			yFloatingY = frame.yRailStart + yDrive.floatingEnd;
		}
		yScrew = new BallScrew(yScrewStart - frame.yRailStart);
  yDriveScrew=yDrive==null?yScrew:new HzMotionDrawingScrew(yScrew.totalLength,yDrive.endProfile);
		if (centreY + yHalfTravel + tableDepth / 2 + clearance > frame.columnFront - block.spec.railHeight)
			throw "Y travel/table depth exceeds spindle reach: increase column-to-spindle clearance before increasing Y stroke";
		var xReach = xBlockSpan + block.spec.blockLength / 2 + block.spec.railEndMargin;
		var yReach = yBlockSpan + block.spec.blockLength / 2 + block.spec.railEndMargin;
		var yRailRoom = Math.min(
			centreY - yHalfTravel - yReach - frame.yRailStart,
			frame.yRailStart + frame.yRailLength - centreY - yHalfTravel - yReach
		);
		// Hardware, not merely rail length, limits overrun. Keep 3 mm before the nut bracket or saddle reaches an end bearing.
		var yRoom = Math.min(yRailRoom, Math.min(
			centreY - yHalfTravel - saddleDepth / 2 - 12 - (yFloatingY + yFloatingDepth / 2) - clearance,
			yFixedY - yFixedDepth / 2 - (centreY + yHalfTravel + saddleDepth / 2) - clearance));
		// Retain 1 mm geometric separation at the overrun boundary; nominal separation is 3 mm.
		yOvertravel = Math.min(yRoom, frame.columnFront - block.spec.railHeight - (centreY + yHalfTravel + tableDepth / 2) - 1);
		xOvertravel = Math.min(
			xRailLength / 2 - xHalfTravel - xReach,
			-xHalfTravel - nut.bodyLength - nut.flangeThickness - (xFixedX + fixed.length / 2) - clearance
		);
		if (frame.baseWidth / 2 < 245 + clearance || yMountPlane < frame.insertThickness + frame.wall ||
			yMotorFace + yMotor.rating.bodyLength + clearance > yServiceRear || yBayWidth / 2 + frame.wall > frame.columnWidth / 2 - 1.5 * frame.wall)
			throw "Rear Y drive bay leaves insufficient column support, floor or service space";
		if (yOvertravel + 1e-6 < frame.yTravel.overtravel)
			throw "Y layout cannot retain the configured overtravel reserve";
		if (xOvertravel <= 0 || yOvertravel <= 0 ||
			frame.yRailSpacing / 2 - frame.seatWidth / 2 < yChannelWidth / 2 + clearance ||
			xRailSpacing / 2 - block.spec.blockWidth / 2 < 42 + clearance ||
			centreY + yHalfTravel + tableDepth / 2 + clearance > frame.columnFront - block.spec.railHeight ||
			yFloatingY - yFloatingDepth / 2 < frame.front ||
			xScrewZ + 33 + clearance > tableBottom || block.spec.blockHeight + RISER_HEIGHT < 76)
			throw "XY stage cannot fit its guides, rear direct drive and full table envelope";
		var flangeLow = centreY - yHalfTravel - saddleDepth / 2;
		var flangeHigh = centreY + yHalfTravel - saddleDepth / 2;
		checkRace(yScrew, yScrewStart - flangeHigh - nut.bodyLength - nut.flangeThickness, yScrewStart - flangeLow);
		checkRace(xScrew, -xHalfTravel - nut.bodyLength - nut.flangeThickness - xScrewStart, xHalfTravel - xScrewStart);
	}

 public function guided():MillGuidedXYStage.MillGuidedXYLayout return {
  centreY:centreY,yRailSpacing:frame.yRailSpacing,yRailStart:frame.yRailStart,yRailPlane:frame.railPlane,
  yRail:LinearRail.metric(profile,frame.yRailLength),xRail:LinearRail.metric(profile,xRailLength),block:block,
  xHalfTravel:xHalfTravel,yHalfTravel:yHalfTravel,xOvertravel:xOvertravel,yOvertravel:yOvertravel,
  xBlockSpan:xBlockSpan,yBlockSpan:yBlockSpan,xRailSpacing:xRailSpacing,
  saddleWidth:450,saddleDepth:saddleDepth,saddleHeight:SADDLE_HEIGHT,tableWidth:400,tableDepth:tableDepth,
  tableThickness:TABLE_HEIGHT,riserHeight:RISER_HEIGHT,driveInterfaces:true
 };

	function checkRace(screw:BallScrew, nutStart:Float, nutEnd:Float):Void {
		if (nutStart < BallScrew.INPUT_JOURNAL_LENGTH + clearance ||
			nutEnd > screw.totalLength - BallScrew.JOURNAL_LENGTH - clearance)
			throw "XY nut reaches a screw journal";
	}
}
