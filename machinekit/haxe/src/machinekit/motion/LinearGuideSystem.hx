package machinekit.motion;

import machinekit.standard.BearingFit.BearingHousingFit;
import machinekit.standard.BearingFit.BearingShaftFit;

/** The paired round-rod guide used by a linear axis.
 *
 * This value object keeps the bearing catalog row, rod journal fit, carriage seat fit and
 * resulting guide rods together. It does not add an assembly root by itself; callers place the
 * two rods and mate the two bearings through their own assembly model.
 */
class LinearGuideSystem {
	public final bearingA:LinearBearing;
	public final bearingB:LinearBearing;
	public final rodA:SteppedShaft;
	public final rodB:SteppedShaft;
	public final bearingDesignation:String;
	public final spacing:Float;
	public final rodLength:Float;
	public final rodDiameter:Float;
	public final seatDiameter:Float;
	public final rodFit:BearingShaftFit;
	public final housingFit:BearingHousingFit;

	public function new(designation:String, spacing:Float, rodLength:Float,
			rodFit:BearingShaftFit = Slip, housingFit:BearingHousingFit = Slip) {
		if (!(spacing > 0)) throw "Linear guide spacing must be positive";
		if (!(rodLength > 0)) throw "Linear guide rod length must be positive";
		bearingA = LinearBearing.metric(designation);
		bearingB = LinearBearing.metric(designation);
		if (!(spacing > bearingA.outerDiameter / 2))
			throw "Linear guide spacing must clear the bearing centre radius";
		bearingDesignation = designation;
		this.spacing = spacing;
		this.rodLength = rodLength;
		this.rodFit = rodFit;
		this.housingFit = housingFit;
		rodDiameter = bearingA.guideRodDiameter(rodFit);
		seatDiameter = bearingA.housingSeatDiameter(housingFit);
		rodA = new SteppedShaft([{diameter: rodDiameter, length: rodLength}]);
		rodB = new SteppedShaft([{diameter: rodDiameter, length: rodLength}]);
	}

	/** Check the carriage envelope against both bearing seats and the bearing body length. */
	public function validateCarriage(width:Float, length:Float):Void {
		if (!(width >= 2 * spacing + seatDiameter))
			throw "Linear guide carriage width does not clear the bearing seats";
		if (!(length >= bearingA.length))
			throw "Linear guide carriage length is shorter than its linear bearings";
	}
}
