package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Dimension;
import machinekit.component.Solids;

/** External involute spur gear, full-depth teeth (addendum and dedendum are adjusted by the
 * profile-shift coefficient), extruded along local +Z from z=0 to z=faceWidth. Backlash is a
 * tangential allowance at the pitch circle and reduces the generated tooth thickness.
 *
 * Tooth flanks are sampled points along the true involute-of-a-circle curve, not exact curves;
 * that is enough fidelity to mesh visually while keeping the outline a single closed polygon.
 * Below the base circle the flank drops to the root circle on a straight radial step rather
 * than a fillet. Tooth counts whose selected profile shift still requires undercut are rejected.
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
	/** Profile-shift coefficient x, in module units. */
	public final profileShift:Float;
	/** Tangential backlash allowance at the pitch circle, in millimetres. */
	public final backlash:Float;

	public final pitchDiameter:Float;
	public final baseDiameter:Float;
	public final outsideDiameter:Float;
	public final rootDiameter:Float;

	public function new(moduleSize:Float, teeth:Int, faceWidth:Float, pressureAngle:Float = STANDARD_PRESSURE_ANGLE,
			profileShift:Float = 0, backlash:Float = 0) {
		if (!(moduleSize > 0)) throw "Spur gear needs a positive module";
		if (teeth < 6) throw "Spur gear needs at least 6 teeth";
		if (!(faceWidth > 0)) throw "Spur gear needs a positive face width";
		if (!validPressureAngle(pressureAngle)) throw "Spur gear pressure angle must be between 14.5 and 25 degrees";
		if (!Math.isFinite(profileShift) || profileShift <= -1 || profileShift >= 1.25)
			throw "Spur gear profile shift must be finite and between -1 and 1.25";
		if (!Math.isFinite(backlash) || backlash < 0) throw "Spur gear backlash must be finite and non-negative";
		if (profileShift < minimumProfileShift(teeth, pressureAngle) - 1e-10)
			throw profileShift == 0 ? "Unshifted spur gear would require undercut; use a positive profile shift or more teeth" :
				"Spur gear profile shift is insufficient to avoid undercut";
		this.moduleSize = moduleSize;
		this.teeth = teeth;
		this.pressureAngle = pressureAngle;
		this.faceWidth = faceWidth;
		this.profileShift = profileShift;
		this.backlash = backlash;
		pitchDiameter = moduleSize * teeth;
		baseDiameter = pitchDiameter * Math.cos(pressureAngle);
		outsideDiameter = pitchDiameter + 2 * moduleSize * (1 + profileShift);
		rootDiameter = pitchDiameter - 2 * moduleSize * (1.25 - profileShift);
		if (!(rootDiameter > 0)) throw "Spur gear root diameter must be positive; use a larger module or more teeth";
		var toothThickness = pitchToothThickness();
		if (!(toothThickness > 0)) throw "Spur gear backlash leaves no tooth thickness";
		var moduleText = Dimension.format(moduleSize);
		var shiftSuffix = profileShift == 0 ? "" : '-X${Dimension.format(profileShift)}';
		var backlashSuffix = backlash == 0 ? "" : '-B${Dimension.format(backlash)}';
		designation = 'SPUR-M$moduleText-${teeth}T$shiftSuffix$backlashSuffix';
		description = 'Spur gear module $moduleText, ${teeth} teeth${profileShift == 0 ? "" : ", profile shift ${Dimension.format(profileShift)}"}${backlash == 0 ? "" : ", backlash ${Dimension.format(backlash)} mm"}';
	}

	/** Conservative no-undercut full-depth limit: ceil(2 / sin(pressureAngle)^2). */
	public static function minimumUnshiftedTeeth(pressureAngle:Float):Int {
		if (!validPressureAngle(pressureAngle)) throw "Spur gear pressure angle must be between 14.5 and 25 degrees";
		var sine = Math.sin(pressureAngle);
		return Math.ceil(2 / (sine * sine));
	}

	/** Conservative minimum profile shift coefficient for a full-depth gear with this tooth count. */
	public static function minimumProfileShift(teeth:Int, pressureAngle:Float = STANDARD_PRESSURE_ANGLE):Float {
		if (teeth < 1) throw "Spur gear needs a positive tooth count";
		if (!validPressureAngle(pressureAngle)) throw "Spur gear pressure angle must be between 14.5 and 25 degrees";
		return 1 - teeth * Math.pow(Math.sin(pressureAngle), 2) / 2;
	}

	/** True for pressure angles within `MIN_PRESSURE_ANGLE`..`MAX_PRESSURE_ANGLE` (radians). */
	public static function validPressureAngle(angle:Float):Bool
		return angle >= MIN_PRESSURE_ANGLE - 1e-12 && angle <= MAX_PRESSURE_ANGLE + 1e-12;

	/** Centre distance to mesh with `other`, including the sum of both profile shifts. Both gears
	 * must share a module and pressure angle. Backlash changes tooth thickness, not centre distance.
	 */
	public function centerDistance(other:SpurGear):Float {
		if (moduleSize != other.moduleSize) throw "Meshing gears must share a module";
		if (Math.abs(pressureAngle - other.pressureAngle) > 1e-10)
			throw "Meshing gears must share a pressure angle";
		return (pitchDiameter + other.pitchDiameter) / 2 +
			moduleSize * (profileShift + other.profileShift) / Math.sin(pressureAngle);
	}

	/** Tooth thickness measured along the pitch circle after profile shift and backlash. */
	public function pitchToothThickness():Float
		return Math.PI * moduleSize / 2 + 2 * moduleSize * profileShift * Math.tan(pressureAngle) - backlash;

	public function geometry():Part
		return Solids.prism(profile(), 0, faceWidth);

	/** Full-gear outline as one closed loop, one tooth centred on each multiple of 2*pi/teeth. */
	function profile():Array<Vector> {
		var rb = baseDiameter / 2, rp = pitchDiameter / 2, ra = outsideDiameter / 2, rf = rootDiameter / 2;
		var rStart = Math.max(rb, rf);
		var toothThickness = pitchToothThickness();
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
