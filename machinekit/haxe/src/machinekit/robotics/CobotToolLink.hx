package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Curve;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Rotating terminal housing between the sixth module and its tool flange.
 * Smooth, hollow, and tapered to the flange diameter, with a closed mounting
 * face at either end. Its extent accounts for the module and flange thickness
 * already present in the DH chain. It is assumed construction, not vendor CAD.
 */
class CobotToolLink extends MachineComponent {
	public static inline var WALL:Float = 2.5;
	public static inline var END_DEPTH:Float = 9;
	public static inline var CLOCKING:Float = Math.PI / 12;
	public static inline var ACCESS_RADIUS:Float = 3.75;
	public final length:Float;
	public final module:CobotJoint;
	public final flange:RobotFlange;
	public final inputRadius:Float;
	public final outputRadius:Float;

	public function new(d:Float, module:CobotJoint, flange:RobotFlange) {
		var length = d - module.length - flange.thickness;
		if (!Math.isFinite(length) || length <= CobotWristLink.HUB_DEPTH + END_DEPTH + 2 * WALL)
			throw "Tool housing needs room for its mounting faces and hollow shell";
		super('COBOT-TOOL-HOUSING-${module.size}-${Dimension.format(d)}-${flange.designation}',
			"Tapered rotating tool-flange housing", "aluminium 6061", true);
		this.length = length;
		this.module = module;
		this.flange = flange;
		inputRadius = CobotWristMount.hubRadius(module);
		outputRadius = flange.flangeDiameter / 2;
		if (Math.min(inputRadius, outputRadius) <= WALL + 2)
			throw "Tool housing has no room for a structural shell";
		addConnector("start", Mount, Solids.axial(0, 0, 0));
		addConnector("end", Mount, Solids.axial(0, 0, length));
		// Clock the six M4 access holes 15 degrees from the four M6 flange
		// receivers, retaining at least 1 mm of web on the smallest module.
		var flangeScrew = flange.mountScrewPart(16);
		for (a in CobotWristMount.points(module, CLOCKING)) for (b in flange.boltPattern()) {
			var distance = Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
			if (distance < ACCESS_RADIUS + (flangeScrew.diameter + 0.2) / 2 + 1)
				throw "Tool housing access holes collide with its flange mounting pattern";
		}
		var index = 1;
		for (point in CobotWristMount.points(module, CLOCKING))
			addConnector('inputBolt${index++}', Mount, Solids.axial(point.x, point.y, CobotWristLink.HUB_DEPTH));
	}

	/** Smoothstep gives a straight tangent at both ends of the taper. */
	public function radiusAt(z:Float):Float {
		var t = z / length;
		return inputRadius + (outputRadius - inputRadius) * t * t * (3 - 2 * t);
	}

	function shell(inset:Float):Part {
		var low = inset == 0 ? 0 : CobotWristLink.HUB_DEPTH;
		var high = inset == 0 ? length : length - END_DEPTH;
		var sections:Array<Curve> = [];
		try {
			for (i in 0...7) {
				var z = low + (high - low) * i / 6;
				sections.push(Curve.circle(radiusAt(z) - inset, Plane.XY().offset(z)));
			}
			var result = Part.loft(sections);
			for (section in sections) section.close();
			return result;
		} catch (error:Dynamic) {
			for (section in sections) section.close();
			throw error;
		}
	}

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.named(shell(0), "housing");
		var holes:Array<Part> = [];
		if (detail != Envelope) holes.push(Solids.named(shell(WALL), "cavity"));
		var index = 1;
		for (point in CobotWristMount.points(module, CLOCKING)) {
			holes.push(Solids.named(Part.cylinderSpan(CobotWristMount.HOLE_RADIUS, -0.1,
				CobotWristLink.HUB_DEPTH + 0.1, point.x, point.y), 'input-M4-$index'));
			holes.push(Solids.named(Part.cylinderSpan(ACCESS_RADIUS, CobotWristLink.HUB_DEPTH,
				length + 0.1, point.x, point.y), 'input-access-${index++}'));
		}
		index = 1;
		var screw = flange.mountScrewPart(16);
		for (point in flange.boltPattern()) holes.push(Solids.named(Part.cylinderSpan(
			(screw.diameter + 0.2) / 2, length - 7.5, length + 0.1, point.x, point.y), 'flange-thread-${index++}'));
		var pilot = CobotWristMount.pilot(module);
		var lip = pilot.translated(new Vector(0, 0, -CobotWristMount.PILOT_DEPTH)); pilot.close();
		return Solids.cut(Solids.union([body, Solids.named(lip, "input-pilot")]), holes);
	}
}
