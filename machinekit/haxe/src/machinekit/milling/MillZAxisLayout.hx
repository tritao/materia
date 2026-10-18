package machinekit.milling;

import machinekit.motion.*;

/** Mechanical Z datums, mm. Positive Z raises the head; zero is its top stop.
 * Vendor-neutral envelopes and bracket thicknesses remain design assumptions.
 */
class MillZAxisLayout {
	public final frame:MillFrameLayout;
	public final block:LinearRailBlock;
	public final nut = new BallNut();
	public final fixed = ScrewSupportUnit.bk12();
	public final floating = ScrewSupportUnit.bf12();
	public final motor = ServoMotor.model("GENERIC-HV-SERVO-750W");
	public final coupling:ShaftCoupling;
	public final brake:ShaftBrakePart;
	public final screw:BallScrew;
	public final lower:Float;
	public final headBottom:Float;
	public final headHeight:Float;
	public final headRootExtension:Float = 30;
	public final spindleY:Float;
	public final blockSpan:Float;
	public final carriageWidth:Float;
	public final carriageHeight:Float;
	public final carriageThickness:Float = 20;
	public final carriageBack:Float;
	public final headBack:Float;
	public final headCentre:Float;
	public final screwY:Float;
	public final screwTop:Float;
	public final fixedZ:Float;
	public final floatingZ:Float;
	public final upperBottom:Float;
	public final upperHeight:Float;
	public final lowerHeight:Float;
	public final crossDriveZ:Null<Float>;
	public final motorFace:Float;
	public final motorStandBottom:Float;
	public final motorStandHeight:Float;
	public final motorPlateThickness:Float = 8;
	public final mountThickness:Float = 20;
	public final nutPlateThickness:Float = 10;
	public final channelWidth:Float;
	public final channelBack:Float;
	public final bracketWidth:Float;
	public final overtravel:Float;
	public final clearance:Float = 3;
	public final profile:String;

	public function new(frame:MillFrameLayout, headBottom:Float, headHeight:Float, spindleY:Float,
		lower:Float = -250, profile:String = "HGR15", guideBlock:Null<LinearRailBlock> = null, crossDriveZ:Null<Float> = null) {
		if (frame == null) throw "Z axis needs frame datums";
		for (v in [headBottom, headHeight, spindleY, lower]) if (!Math.isFinite(v)) throw "Z datums must be finite";
		if (headHeight <= 0 || lower >= 0) throw "Z needs positive head height and negative lower stop";
		this.crossDriveZ = crossDriveZ;
		this.frame = frame; this.headBottom = headBottom; this.headHeight = headHeight;
		this.spindleY = spindleY; this.lower = lower; this.profile = profile;
		if (guideBlock != null && guideBlock.spec.designation != profile)
			throw "Guide block must match the selected rail profile";
		block = guideBlock == null ? LinearRailBlock.metric(profile) : guideBlock;
		blockSpan = (block.spec.blockLength + 80) / 2;
		carriageWidth = frame.zRailSpacing + block.spec.blockWidth + 20;
		carriageHeight = Math.max(headHeight + 30, 2 * blockSpan + block.spec.blockLength + 20);
		carriageBack = frame.columnFront - block.spec.blockHeight;
		headBack = carriageBack - carriageThickness;
		headCentre = headBottom + headHeight / 2;
		// The bearing feet point forward. Their mounting plane coincides with
		// the finished front-facing steel pads on the column.
		screwY = frame.columnFront - fixed.connector("mount").frame.y;
		bracketWidth = nut.flangeDiameter + 2 * MillZCarriage.SHELF_MARGIN;
		channelWidth = Math.max(bracketWidth, fixed.width) + 2 * clearance;
		channelBack = screwY + bracketWidth / 2 + clearance;
		upperBottom = frame.railPlane + frame.zRailStart + frame.zRailLength;
		lowerHeight = frame.zRailStart;
		floatingZ = frame.railPlane + lowerHeight / 2;
		fixedZ = frame.railPlane + frame.columnHeight + 20;
		upperHeight = 2 * (fixedZ - upperBottom);
		screwTop = fixedZ + BallScrew.INPUT_JOURNAL_LENGTH - fixed.length / 2;
		screw = new BallScrew(screwTop - floatingZ + BallScrew.JOURNAL_LENGTH / 2);
		if (crossDriveZ != null && (!Math.isFinite(crossDriveZ) || floatingZ-BallScrew.JOURNAL_LENGTH/2 <= crossDriveZ+12+clearance))
			throw "Z screw end enters the crossing Y shaft clearance";
		brake = new ShaftBrakePart(motor.rating.shaftDiameter, "shell");
		var couplingLength = 2 * (motor.rating.shaftLength - motorPlateThickness - brake.depth - 1);
		coupling = new ShaftCoupling(motor.rating.shaftDiameter, BallScrew.JOURNAL_DIAMETER, null, couplingLength);
		motorFace = screwTop + motor.rating.shaftLength;
		motorStandBottom = upperBottom + upperHeight;
		motorStandHeight = motorFace - motorStandBottom;
		var railLow = frame.railPlane + frame.zRailStart + block.spec.railEndMargin;
		var railHigh = upperBottom - block.spec.railEndMargin;
		var reach = blockSpan + block.spec.blockLength / 2;
		overtravel = Math.min(headCentre + lower - reach - railLow, railHigh - headCentre - reach);
		if (frame.zRailSpacing - frame.seatWidth <= channelWidth + 2 * clearance ||
			frame.columnWidth < carriageWidth || lowerHeight < 20 ||
			frame.columnHeight - frame.zRailStart - frame.zRailLength < 20 ||
			upperHeight < fixed.length || motorStandHeight <= motorPlateThickness ||
			couplingLength / 2 < motor.rating.shaftDiameter * 0.75 || overtravel <= 0 ||
			headCentre + carriageHeight / 2 + clearance >= upperBottom ||
			headCentre + lower - carriageHeight / 2 <= frame.railPlane + lowerHeight + clearance ||
			headBack - spindleY <= SpindleCartridge.DIAMETER / 2 + 20 ||
			channelBack + frame.wall >= frame.columnCentreY + frame.columnDepth / 2)
			throw "Z layout cannot fit its carriage, central drive and supported end mounts";
		// Both ends of the complete nut must remain on the threaded race.
		var nutTop = headCentre + nut.bodyLength / 2;
		var nutBottom = headCentre + lower - nut.bodyLength / 2 - nut.flangeThickness;
		if (screwTop - nutTop < BallScrew.INPUT_JOURNAL_LENGTH + clearance ||
			screwTop - nutBottom > screw.totalLength - BallScrew.JOURNAL_LENGTH - clearance)
			throw "Z travel reaches a ball screw journal";
	}
}
