package machinekit.milling;

import machinekit.motion.BallScrew;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.ScrewSupportUnit;

/** Y working envelope, shared by the frame and cross-slide. Millimetres.
 * Increasing stroke grows the table and front structure; spindle reach must still
 * satisfy the rear table/column clearance checked by MillXYStageLayout.
 */
class MillYTravel {
	public final stroke:Float;
	public final tableEdgeMargin:Float;
	public final saddleDepth:Float;
	public final overtravel:Float;
	public final clearance:Float = 3;
	public final frontExtension:Float = 40;
	public final halfTravel:Float;
	public final tableDepth:Float;

	public function new(stroke:Float = 160, tableEdgeMargin:Float = 25, saddleDepth:Float = 200, overtravel:Float = 2) {
		for (v in [stroke, tableEdgeMargin, saddleDepth, overtravel])
			if (!Math.isFinite(v) || v <= 0) throw "Y travel dimensions must be finite and positive";
		this.stroke = stroke; this.tableEdgeMargin = tableEdgeMargin;
		this.saddleDepth = saddleDepth; this.overtravel = overtravel;
		halfTravel = stroke / 2; tableDepth = stroke + 2 * tableEdgeMargin;
	}

	/** Covers both the reference BF12 and selected HZMotion BF12 drive.
	 * Round outwards to 10 mm, retaining the requested overrun at both ends.
	 */
	public function railStart(centreY:Float, block:LinearRailBlock):Float {
		var reach = block.spec.blockLength + 20 + block.spec.railEndMargin;
		var guide = centreY - halfTravel - reach - overtravel;
		var supportRear = Math.max(BallScrew.JOURNAL_LENGTH / 2, 7) + ScrewSupportUnit.bf12().length / 2;
		var bearing = centreY - halfTravel - saddleDepth / 2 - 12 - clearance - overtravel - supportRear;
		var race = centreY - halfTravel - saddleDepth / 2 - BallScrew.JOURNAL_LENGTH - clearance - overtravel;
		return Math.floor(Math.min(guide, Math.min(bearing, race)) / 10) * 10;
	}

	public function railEnd(centreY:Float, block:LinearRailBlock):Float {
		var reach = block.spec.blockLength + 20 + block.spec.railEndMargin;
		return Math.ceil((centreY + halfTravel + reach + overtravel) / 10) * 10;
	}
}
