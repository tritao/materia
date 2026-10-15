package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** A hollow structural elbow between perpendicular wrist modules. The section is
 * sized from the modules, independently of the gap between them. A saddle relief
 * follows the next module's envelope; the inner cavity stops one wall before that
 * relief. Both DH mating planes stay fixed. Dimensions are assumed construction,
 * not a bearing or fatigue rating.
 */
class CobotWristLink extends MachineComponent {
	public static inline var CLEARANCE:Float = 1.5;
	public static inline var WALL:Float = 2.5;
	public static inline var HUB_DEPTH:Float = 5;
	public final seat:AssemblyFrame;
	public final radius:Float;
	public final input:CobotJoint;
	public final output:CobotJoint;
	final axis:Vector;
	final route:Array<Vector>;

	public function new(d:Float, alpha:Float, input:CobotJoint, output:CobotJoint) {
		if (!Math.isFinite(d) || !Math.isFinite(alpha) || Math.abs(Math.abs(alpha) - Math.PI / 2) > 1e-9)
			throw "Wrist elbow needs a finite offset and perpendicular module axes";
		var length = d - input.length;
		var gap = length - output.diameter / 2;
		if (gap < HUB_DEPTH + CLEARANCE + 2 * WALL)
			throw "Wrist elbow needs room for its hub, clearance and two structural walls";
		super('COBOT-WRIST-${Dimension.format(d)}-${Dimension.format(alpha)}-${input.size}-${output.size}',
			"Hollow wrist elbow with relieved saddle and locating hubs", "aluminium 6061", true);
		this.input = input;
		this.output = output;
		radius = Math.min(input.diameter, output.diameter) * 0.42;
		seat = {x: 0, y: 0, z: length, qx: Math.sin(alpha / 2), qy: 0, qz: 0, qw: Math.cos(alpha / 2)};
		// The constructor accepts only quarter turns. Use exact CAD axes rather
		// than a residual cos(pi/2) at coincident mounting planes.
		axis = new Vector(0, alpha > 0 ? -1 : 1, 0);
		addConnector("start", Mount, Solids.axial(0, 0, 0));
		addConnector("end", Mount, AssemblyFrames.compose(seat, Solids.axial(0, 0, 0)));
		var index = 1;
		for (point in CobotWristMount.points(input))
			addConnector('inputBolt${index++}', Mount, Solids.axial(point.x, point.y, HUB_DEPTH));
		index = 1;
		for (point in CobotWristMount.points(output))
			addConnector('outputBolt${index++}', Mount, AssemblyFrames.compose(seat,
				AssemblyFrames.alongY(point.x, point.y, -HUB_DEPTH, 0, 0, -1)));
		var reach = HUB_DEPTH + radius / 2;
		route = [new Vector(0, 0, reach), new Vector(-axis.x * reach, -axis.y * reach, reach),
			new Vector(-axis.x * reach, -axis.y * reach, length)];
	}

	override public function hasGeometry():Bool return true;

	/** Broad elbow before the saddle is relieved. The end caps remain solid. */
	function envelope(section:Float, inset:Float):Part {
		var pieces:Array<Part> = [];
		return Solids.building(pieces, tracked -> {
			tracked.push(Part.cylinderSpan(section, inset, route[0].z));
			for (i in 0...route.length) {
				var sphere = Part.sphere(section);
				tracked.push(sphere.translated(route[i]));
				sphere.close();
				if (i < route.length - 1) {
					var delta = route[i + 1].subtract(route[i]);
					if (delta.length() > 1e-6)
						tracked.push(Part.cylinderAlong(section, route[i], delta, delta.length()));
				}
			}
			var start = route[route.length - 1];
			tracked.push(Part.cylinderAlong(section, start, axis, HUB_DEPTH + radius / 2 - inset));
			// The broad bend crosses the input plane. Trim the outside and cavity
			// independently so the housing stays clear and the input hub stays closed.
			var whole = Solids.union(tracked);
			var clip = Part.box(6 * input.diameter, 6 * input.diameter, seat.z + 4 * radius);
			var raised = clip.translated(new Vector(0, 0, inset)); clip.close();
			try {
				var result = whole.intersect(raised); whole.close(); raised.close();
				return result;
			} catch (error:Dynamic) { whole.close(); raised.close(); throw error; }
		});
	}

	/** Keep the entire next housing and its rotating face out of the shell. */
	function relief(extra:Float):Part {
		var clearance = CLEARANCE + extra;
		return Part.cylinderAlong(output.diameter / 2 + clearance,
			new Vector(seat.x - axis.x * clearance, seat.y - axis.y * clearance,
				seat.z - axis.z * clearance), axis, output.length + 2 * radius + 2 * clearance);
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.cut(envelope(radius, 0), [relief(0)]);
		if (detail != Envelope) {
			// Expanding the exclusion as well as shrinking the cavity leaves a wall
			// along the concave saddle, rather than cutting the hollow shell open.
			var cavity = Solids.cut(envelope(radius - WALL, HUB_DEPTH), [relief(WALL)]);
			body = Solids.cut(body, [cavity]);
		}
		var pieces = [Solids.named(body, "shell"),
			Solids.named(softenedHub(CobotWristMount.hubRadius(input)), "input-hub"),
			Solids.named(atOutput(backHub(CobotWristMount.hubRadius(output))), "output-hub")];
		var inputPilot = CobotWristMount.pilot(input);
		pieces.push(Solids.named(inputPilot.translated(new Vector(0, 0, -CobotWristMount.PILOT_DEPTH)), "input-pilot"));
		inputPilot.close();
		pieces.push(Solids.named(atOutput(CobotWristMount.pilot(output)), "output-pilot"));
		var holes:Array<Part> = [];
		var index = 1;
		for (point in CobotWristMount.points(input)) {
			holes.push(Solids.named(Part.cylinderSpan(CobotWristMount.HOLE_RADIUS,
				-0.1, HUB_DEPTH + 0.1, point.x, point.y), 'input-M4-${index}'));
			holes.push(Solids.named(Part.cylinderSpan(4, HUB_DEPTH,
				HUB_DEPTH + 4.5, point.x, point.y), 'input-access-${index++}'));
		}
		index = 1;
		for (point in CobotWristMount.points(output)) {
			holes.push(Solids.named(atOutput(Part.cylinderSpan(CobotWristMount.HOLE_RADIUS,
				-HUB_DEPTH - 0.1, 0.1, point.x, point.y)), 'output-M4-${index}'));
			holes.push(Solids.named(atOutput(Part.cylinderSpan(4,
				-HUB_DEPTH - 4.5, -HUB_DEPTH, point.x, point.y)), 'output-access-${index++}'));
		}
		return Solids.cut(Solids.union(pieces), holes);
	}

	static function softenedHub(radius:Float):Part {
		// A turned edge break remains well conditioned where the shell and
		// screw pockets join the hub; a toroidal fillet there creates sliver faces.
		var bevel = 1.0;
		return Part.revolve([{r: 0, z: 0}, {r: radius - bevel, z: 0},
			{r: radius, z: bevel}, {r: radius, z: HUB_DEPTH - bevel},
			{r: radius - bevel, z: HUB_DEPTH}, {r: 0, z: HUB_DEPTH}]);
	}

	static function backHub(radius:Float):Part {
		var hub = softenedHub(radius);
		try {
			var result = hub.translated(new Vector(0, 0, -HUB_DEPTH)); hub.close(); return result;
		} catch (error:Dynamic) { hub.close(); throw error; }
	}

	/** Consume a solid authored in the next module's frame. */
	function atOutput(part:Part):Part {
		var x = AssemblyFrames.transformVector(seat, 1, 0, 0);
		try {
			var result = part.placed(new Location(new Plane(new Vector(seat.x, seat.y, seat.z),
				new Vector(x.x, x.y, x.z), axis)));
			part.close();
			return result;
		} catch (error:Dynamic) { part.close(); throw error; }
	}
}
