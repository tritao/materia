package machinekit.milling;

import machinekit.motion.*;

/** Mill-specific cross-slide datums. Y moves the saddle; X moves the table.
 * Positive work coordinates move the workpiece in the negative world direction.
 * Dimensions and motor patterns are design assumptions, not released vendor drawings.
 */
class MillXYStageLayout {
	public static inline var SADDLE_HEIGHT:Float = 80;
	public static inline var RISER_HEIGHT:Float = 48;
	public static inline var TABLE_HEIGHT:Float = 40;
	public final frame:MillFrameLayout;
	public final centreY:Float;
	public final profile:String;
	public final block:LinearRailBlock;
	public final nut = new BallNut();
	public final fixed = ScrewSupportUnit.bk12();
	public final floating = ScrewSupportUnit.bf12();
	public final motor = ServoMotor.model("GENERIC-SERVO-400W");
	public final coupling = new ShaftCoupling(14, 12, null, 28);
	public final xScrew = new BallScrew(455);
	public final yScrew:BallScrew;
	public final xHalfTravel:Float = 125;
	public final yHalfTravel:Float = 75;
	public final xRailSpacing:Float = 124;
	public final xRailLength:Float = 440;
	public final saddleDepth:Float = 174;
	public final blockSpan:Float;
	public final saddleBottom:Float;
	public final xRailPlane:Float;
	public final tableBottom:Float;
	public final tableTop:Float;
	public final yScrewZ:Float;
	public final xScrewZ:Float;
	public final yFixedY:Float;
	public final yFloatingY:Float;
	public final xFixedX:Float = -192.5;
	public final xFloatingX:Float = 212.5;
	public final xScrewStart:Float = -230;
	public final clearance:Float = 3;
	public final xOvertravel:Float;
	public final yOvertravel:Float;

	public static function stackHeight(profile:String):Float return 2 * LinearRailBlock.metric(profile).spec.blockHeight + SADDLE_HEIGHT + RISER_HEIGHT + TABLE_HEIGHT;

	public function new(frame:MillFrameLayout, centreY:Float, profile:String = "HGR15") {
		if (frame == null || !Math.isFinite(centreY)) throw "XY stage needs finite matching frame datums";
		this.frame = frame;
		this.centreY = centreY;
		this.profile = profile;
		block = LinearRailBlock.metric(profile);
		blockSpan = (block.spec.blockLength + 4) / 2;
		saddleBottom = frame.railPlane + block.spec.blockHeight;
		xRailPlane = saddleBottom + SADDLE_HEIGHT;
		tableBottom = xRailPlane + block.spec.blockHeight + RISER_HEIGHT;
		tableTop = tableBottom + TABLE_HEIGHT;
		yScrewZ = saddleBottom + SADDLE_HEIGHT / 2;
		xScrewZ = xRailPlane + 40;
		yFixedY = frame.yRailStart + BallScrew.INPUT_JOURNAL_LENGTH - fixed.length / 2;
		// The floating journal extends 5 mm beyond the guide length, into the clear space before the column.
		yFloatingY = frame.yRailStart + frame.yRailLength + 5 - BallScrew.JOURNAL_LENGTH / 2;
		yScrew = new BallScrew(frame.yRailLength + 5);
		var reach = blockSpan + block.spec.blockLength / 2 + block.spec.railEndMargin;
		var yRailRoom = Math.min(
			centreY - yHalfTravel - reach - frame.yRailStart,
			frame.yRailStart + frame.yRailLength - centreY - yHalfTravel - reach
		);
		// Hardware, not merely rail length, limits overrun. Keep 3 mm before the nut bracket or saddle reaches an end bearing.
		yOvertravel = Math.min(
			yRailRoom,
			Math.min(
				yFloatingY - floating.length / 2 - (centreY + yHalfTravel + saddleDepth / 2 + 10) - clearance,
				centreY - yHalfTravel - saddleDepth / 2 - (yFixedY + fixed.length / 2) - clearance
			)
		);
		xOvertravel = Math.min(
			xRailLength / 2 - xHalfTravel - reach,
			-xHalfTravel - nut.bodyLength - nut.flangeThickness - (xFixedX + fixed.length / 2) - clearance
		);
		var nutRadius = nut.flangeDiameter / 2;
		if (xOvertravel <= 0 || yOvertravel <= 0 ||
			frame.yRailSpacing / 2 - block.spec.blockWidth / 2 < 50 + clearance ||
			xRailSpacing / 2 - block.spec.blockWidth / 2 < 42 + clearance ||
			centreY + yHalfTravel + 100 + clearance > frame.columnFront ||
			yFloatingY + floating.length / 2 + clearance > frame.columnFront + frame.seatRise ||
			nutRadius + clearance > 27 || xScrewZ + 33 + clearance > tableBottom ||
			yScrewZ - 25 - frame.railPlane < 15 || block.spec.blockHeight + RISER_HEIGHT < 76)
			throw "XY stage cannot fit its guides, central nuts and supported drives";
		checkRace(yScrew, centreY - yHalfTravel + saddleDepth / 2 - nut.bodyLength - nut.flangeThickness - frame.yRailStart,
			centreY + yHalfTravel + saddleDepth / 2 - frame.yRailStart);
		checkRace(xScrew, -xHalfTravel - nut.bodyLength - nut.flangeThickness - xScrewStart, xHalfTravel - xScrewStart);
	}

	function checkRace(screw:BallScrew, nutStart:Float, nutEnd:Float):Void {
		if (nutStart < BallScrew.INPUT_JOURNAL_LENGTH + clearance ||
			nutEnd > screw.totalLength - BallScrew.JOURNAL_LENGTH - clearance)
			throw "XY nut reaches a screw journal";
	}
}
