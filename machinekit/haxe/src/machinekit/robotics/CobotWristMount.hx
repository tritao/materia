package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Solids;

/** Assumed M4 wrist interface. A shallow annular locating shoulder stays outside
 * the installed gearhead; six holes share the remaining housing annulus. The
 * counterbore locates a structural hub, not a separately specified rolling bearing.
 */
class CobotWristMount {
	public static inline var COUNT:Int = 6;
	public static inline var PILOT_DEPTH:Float = 2;
	public static inline var PILOT_CLEARANCE:Float = 0.2;
	public static inline var HOLE_RADIUS:Float = 2.25;
	public static inline var ENGAGEMENT:Float = 7;

	public static function hubRadius(module:CobotJoint):Float return module.diameter / 2 - 2.5;
	public static function pilotRadius(module:CobotJoint):Float return module.pocketDiameter / 2 + 2.5;
	public static function points(module:CobotJoint, clocking:Float = 0):Array<Vector> {
		if (!Math.isFinite(clocking)) throw "Wrist mounting clocking must be finite";
		var radius = (module.pocketDiameter + module.diameter) / 4;
		if (radius - HOLE_RADIUS < pilotRadius(module) + 1 ||
			radius + 3.5 + 1 > hubRadius(module)) throw "Wrist interface has no room for its pilot and M4 mounting pattern";
		return [for (i in 0...COUNT) new Vector(radius * Math.cos(clocking + 2 * Math.PI * i / COUNT),
			radius * Math.sin(clocking + 2 * Math.PI * i / COUNT), 0)];
	}

	/** In the joint's CAD frame; rotor is the +Z end, stator is z=0. */
	public static function housingCuts(module:CobotJoint, rotor:Bool, clocking:Float = 0):Array<Part> {
		var face = rotor ? module.length : 0;
		var low = rotor ? face - PILOT_DEPTH - PILOT_CLEARANCE : -0.1;
		var high = rotor ? face + 0.1 : PILOT_DEPTH + PILOT_CLEARANCE;
		var cuts = [Solids.named(Part.cylinderSpan(pilotRadius(module) + PILOT_CLEARANCE, low, high),
			rotor ? "rotor-pilot-seat" : "stator-pilot-seat")];
		var index = 0;
		// Nominal threaded-hole envelopes. Thread flanks are semantic, as on
		// SocketHeadCapScrew; leave 0.1 mm radial clearance to its major diameter.
		for (point in points(module, clocking)) cuts.push(Solids.named(Part.cylinderSpan(2.1,
			rotor ? face - ENGAGEMENT - 0.5 : -0.1,
			rotor ? face + 0.1 : ENGAGEMENT + 0.5, point.x, point.y),
			(rotor ? "rotor-M4-" : "stator-M4-") + index++));
		return cuts;
	}

	/** Axisymmetric bearing/output recess in the fixed housing. The rotating
	 * flange is a separate member; a bolt pattern must never be cut in a stator
	 * and then expected to follow the driven joint.
	 */
	public static function rotorPocket(module:CobotJoint):Array<Part> {
		var outer = module.diameter / 2 - 2.3;
		return [Solids.named(Part.cylinderSpan(outer, module.length - 4.2, module.length + 0.1), "rotor-face-pocket"),
			Solids.named(Solids.cut(Part.cylinderSpan(outer, module.length - 8.2, module.length + 0.1),
				[Part.cylinderSpan(module.pocketDiameter / 2 + 0.3, module.length - 8.3, module.length + 0.2)]),
				"rotor-bearing-pocket")];
	}

	/** The mating hub's annular locating lip, in a frame pointing into the module. */
	public static function pilot(module:CobotJoint):Part
		return Solids.cut(Part.cylinderSpan(pilotRadius(module), 0, PILOT_DEPTH),
			[Part.cylinderSpan(module.pocketDiameter / 2 + 0.5, -0.1, PILOT_DEPTH + 0.1)]);
}
