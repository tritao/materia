package machinekit.robotics;

/** Assumed source-link dimensions in mm, not a manufacturer's specification.
 * These describe the mechanism; OPW parameters must be extracted from its
 * compiled joint axes rather than supplied from this table.
 */
typedef IndustrialArmReferenceRow = {
	var designation:String;
	var nominalReach:Float;
	var turret:Float;
	var upperArm:Float;
	var forearm:Float;
	var wristBody:Float;
	var hand:Float;
	var lowerDegrees:Array<Float>;
	var upperDegrees:Array<Float>;
}

class IndustrialArmReference {
	public static function of(cls:IndustrialArmClass):IndustrialArmReferenceRow {
		var name:String, reach:Float, lengths:Array<Float>;
		switch cls {
			case Reach700: name = "IndustrialReach700"; reach = 700; lengths = [80, 240, 195, 65, 55];
			case Reach900: name = "IndustrialReach900"; reach = 900; lengths = [90, 320, 260, 90, 60];
			case Reach1300: name = "IndustrialReach1300"; reach = 1300; lengths = [110, 520, 440, 120, 70];
		}
		return {designation: name, nominalReach: reach, turret: lengths[0], upperArm: lengths[1],
			forearm: lengths[2], wristBody: lengths[3], hand: lengths[4],
			lowerDegrees: [-170, -100, -70, -190, -125, -360],
			upperDegrees: [170, 135, 200, 190, 125, 360]};
	}
}
