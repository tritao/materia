package machinekit.robotics;

/** Published DH dimensions (mm), payload (kg), and joint speeds (degrees/s).
 * Module construction and drive ratings are separate, assumed engineering values.
 */
typedef CobotReferenceRow = {
	var designation:String;
	var reach:Float;
	var payload:Float;
	var mass:Float;
	var d:Array<Float>;
	var a:Array<Float>;
	var alpha:Array<Float>;
	var speeds:Array<Float>;
	var modules:Array<Int>;
}

/** UR e-series reference geometry. Mass includes the maker's stated cable allowance.
 * Sources: universal-robots.com/developer/hardware-and-motion/robot-motion-dh-parameters/
 * and the collective technical data sheet, November 2023. These describe size classes;
 * the generic module model does not claim to reproduce the manufacturer's internals.
 */
class CobotReference {
	public static function of(cls:CobotClass):CobotReferenceRow {
		var dimensions:Array<Float>, speeds:Array<Float>, modules:Array<Int>;
		var name:String, reach:Float, payload:Float, mass:Float;
		switch cls {
			case Reach500:
				name = "Reach500"; reach = 500; payload = 3; mass = 11.2;
				dimensions = [151.85, 243.55, 213.2, 131.05, 85.35, 92.1];
				speeds = [180, 180, 180, 360, 360, 360]; modules = [2, 2, 2, 0, 0, 0];
			case Reach850:
				name = "Reach850"; reach = 850; payload = 5; mass = 20.6;
				dimensions = [162.5, 425, 392.2, 133.3, 99.7, 99.6];
				speeds = [180, 180, 180, 180, 180, 180]; modules = [3, 3, 3, 1, 1, 1];
			case Reach900:
				name = "Reach900"; reach = 900; payload = 16; mass = 33.1;
				dimensions = [180.7, 478.4, 360, 174.15, 119.85, 116.55];
				speeds = [120, 120, 180, 180, 180, 180]; modules = [4, 4, 3, 2, 2, 2];
			case Reach1300:
				name = "Reach1300"; reach = 1300; payload = 12.5; mass = 33.5;
				dimensions = [180.7, 612.7, 571.55, 174.15, 119.85, 116.55];
				speeds = [120, 120, 180, 180, 180, 180]; modules = [4, 4, 3, 2, 2, 2];
		}
		return {designation: name, reach: reach, payload: payload, mass: mass,
			d: [dimensions[0], 0, 0, dimensions[3], dimensions[4], dimensions[5]],
			a: [0, -dimensions[1], -dimensions[2], 0, 0, 0],
			alpha: [Math.PI / 2, 0, 0, Math.PI / 2, -Math.PI / 2, 0],
			speeds: speeds, modules: modules};
	}
}
