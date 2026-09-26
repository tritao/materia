package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Dimension;
import machinekit.component.Solids;

/** External involute spur gear, standard full-depth teeth (addendum = module, dedendum = 1.25
 * module, no profile shift or backlash), extruded along local +Z from z=0 to z=faceWidth.
 *
 * Tooth flanks are sampled points along the true involute-of-a-circle curve, not exact curves;
 * that is enough fidelity to mesh visually while keeping the outline a single closed polygon.
 * Below the base circle (common for small tooth counts) the flank drops to the root circle on a
 * straight radial step rather than a fillet.
 */
class SpurGear {
	public static inline var STANDARD_PRESSURE_ANGLE:Float = 0.3490658503988659; // 20 degrees
	/** Accepted pressure-angle range, 14.5 to 25 degrees: the standard involute systems. */
	public static inline var MIN_PRESSURE_ANGLE:Float = 0.25307274153917777; // 14.5 degrees
	public static inline var MAX_PRESSURE_ANGLE:Float = 0.4363323129985824; // 25 degrees

	static inline var FLANK_SAMPLES:Int = 6;
	static inline var TIP_SAMPLES:Int = 3;

	public final designation:String;
	public final description:String;
	public final moduleSize:Float;
	public final teeth:Int;
	public final pressureAngle:Float;
	public final faceWidth:Float;

	public final pitchDiameter:Float;
	public final baseDiameter:Float;
	public final outsideDiameter:Float;
	public final rootDiameter:Float;

	public function new(moduleSize:Float, teeth:Int, faceWidth:Float, pressureAngle:Float = STANDARD_PRESSURE_ANGLE) {
		if (!(moduleSize > 0)) throw "Spur gear needs a positive module";
		if (teeth < 6) throw "Spur gear needs at least 6 teeth to avoid severe undercut";
		if (!(faceWidth > 0)) throw "Spur gear needs a positive face width";
		if (!validPressureAngle(pressureAngle)) throw "Spur gear pressure angle must be between 14.5 and 25 degrees";
		this.moduleSize = moduleSize;
		this.teeth = teeth;
		this.pressureAngle = pressureAngle;
		this.faceWidth = faceWidth;
		pitchDiameter = moduleSize * teeth;
		baseDiameter = pitchDiameter * Math.cos(pressureAngle);
		outsideDiameter = pitchDiameter + 2 * moduleSize;
		rootDiameter = pitchDiameter - 2.5 * moduleSize;
		if (!(rootDiameter > 0)) throw "Spur gear root diameter must be positive; use a larger module or more teeth";
		var moduleText = Dimension.format(moduleSize);
		designation = 'SPUR-M$moduleText-${teeth}T';
		description = 'Spur gear module $moduleText, ${teeth} teeth';
	}

	/** True for pressure angles within `MIN_PRESSURE_ANGLE`..`MAX_PRESSURE_ANGLE` (radians). */
	public static function validPressureAngle(angle:Float):Bool
		return angle >= MIN_PRESSURE_ANGLE - 1e-12 && angle <= MAX_PRESSURE_ANGLE + 1e-12;

	/** Centre distance to mesh with `other` at the standard (no profile shift) pitch. Both gears
	 * must share a module and pressure angle.
	 */
	public function centerDistance(other:SpurGear):Float {
		if (moduleSize != other.moduleSize) throw "Meshing gears must share a module";
		if (Math.abs(pressureAngle - other.pressureAngle) > 1e-10)
			throw "Meshing gears must share a pressure angle";
		return (pitchDiameter + other.pitchDiameter) / 2;
	}

	public function geometry():Part
		return Solids.prism(profile(), 0, faceWidth);

	/** Full-gear outline as one closed loop, one tooth centred on each multiple of 2*pi/teeth. */
	function profile():Array<Vector> {
		var rb = baseDiameter / 2, rp = pitchDiameter / 2, ra = outsideDiameter / 2, rf = rootDiameter / 2;
		var rStart = Math.max(rb, rf);
		var toothThickness = Math.PI * moduleSize / 2;
		var halfToothAngle = toothThickness / (2 * rp);
		var tPitch = Math.sqrt(Math.pow(rp / rb, 2) - 1);
		var involuteAnglePitch = tPitch - Math.atan2(tPitch, 1);
		var angleStep = 2 * Math.PI / teeth;
		var points:Array<Vector> = [];
		for (i in 0...teeth) {
			var center = i * angleStep;
			var rightTipAngle = 0.0;
			for (s in 0...FLANK_SAMPLES) {
				var r = rStart + (ra - rStart) * s / (FLANK_SAMPLES - 1);
				var angle = center - halfToothAngle + (involuteRollAngle(r, rb) - involuteAnglePitch);
				if (s == FLANK_SAMPLES - 1) rightTipAngle = angle;
				points.push(new Vector(r * Math.cos(angle), r * Math.sin(angle)));
			}
			var leftTipAngle = 2 * center - rightTipAngle;
			for (s in 1...TIP_SAMPLES - 1) {
				var angle = rightTipAngle + (leftTipAngle - rightTipAngle) * s / (TIP_SAMPLES - 1);
				points.push(new Vector(ra * Math.cos(angle), ra * Math.sin(angle)));
			}
			for (s in 0...FLANK_SAMPLES) {
				var k = FLANK_SAMPLES - 1 - s;
				var r = rStart + (ra - rStart) * k / (FLANK_SAMPLES - 1);
				var angle = center + halfToothAngle - (involuteRollAngle(r, rb) - involuteAnglePitch);
				points.push(new Vector(r * Math.cos(angle), r * Math.sin(angle)));
			}
			if (rf < rStart) {
				var leftFlankRootAngle = center + halfToothAngle + involuteAnglePitch;
				points.push(new Vector(rf * Math.cos(leftFlankRootAngle), rf * Math.sin(leftFlankRootAngle)));
				var nextRightFlankRootAngle = center + angleStep - halfToothAngle - involuteAnglePitch;
				points.push(new Vector(rf * Math.cos(nextRightFlankRootAngle), rf * Math.sin(nextRightFlankRootAngle)));
			}
		}
		return points;
	}

	/** Involute roll angle (t - atan t) of the point at radius `r` on a circle of base radius `rb`. */
	static function involuteRollAngle(r:Float, rb:Float):Float {
		var t = r > rb ? Math.sqrt(Math.pow(r / rb, 2) - 1) : 0;
		return t - Math.atan2(t, 1);
	}
}
