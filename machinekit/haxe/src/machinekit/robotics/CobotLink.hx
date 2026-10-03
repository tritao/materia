package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Tube and end caps between offset joint seats. The outgoing seat removes the
 * preceding housing's axial length from the DH transform; that length is already
 * present in its rotor face. The tube passes outside the housing radii.
 */
class CobotLink extends MachineComponent {
	public final seat:AssemblyFrame;
	public final radius:Float;
	public final wall:Float;
	public final cap:Float;
	final route:Array<Vector>;

	public function new(a:Float, d:Float, alpha:Float, previous:CobotJoint,
			nextRadius:Float) {
		var radius = Math.max(12, previous.diameter * 0.14);
		var cap = 4.0;
		if (Math.abs(a) < 1e-9 && Math.abs(alpha) > 1e-9) {
			var gap = d - previous.length - nextRadius;
			if (!(gap > cap)) throw "Cobot module seats leave no room for their link";
			radius = Math.min(radius, (gap - cap) * 0.8);
		}
		var wall = Math.max(1, radius * 0.12);
		var seat:AssemblyFrame = {x: a, y: 0, z: d - previous.length,
			qx: Math.sin(alpha / 2), qy: 0, qz: 0, qw: Math.cos(alpha / 2)};
		super('COBOT-LINK-${Dimension.format(a)}-${Dimension.format(d)}-${Dimension.format(alpha)}-' +
			'${Dimension.format(previous.diameter)}x${Dimension.format(previous.length)}-${Dimension.format(nextRadius)}',
			"Offset cobot tube link", "aluminium 6061", true);
		this.seat = seat;
		this.radius = radius;
		this.wall = wall;
		this.cap = cap;
		addConnector("start", Mount, Solids.axial(0, 0, 0));
		addConnector("end", Mount, AssemblyFrames.compose(seat, Solids.axial(0, 0, 0)));
		var axis = AssemblyFrames.transformVector(seat, 0, 0, 1);
		var end = new Vector(seat.x - axis.x * (cap + radius),
			seat.y - axis.y * (cap + radius), seat.z - axis.z * (cap + radius));
		var start = new Vector(0, 0, cap + radius);
		if (a < 0 && Math.abs(alpha) < 1e-9) {
			// Parallel shoulder/elbow axes: leave both housings radially before
			// crossing from the rotor side of one to the stator side of the next.
			route = [start, new Vector(-previous.diameter / 2 - radius, 0, start.z),
				new Vector(a + nextRadius + radius, 0, end.z), end];
		} else if (Math.abs(alpha) > 1e-9) {
			// Stay on the outside of the next stator plane before rising
			// alongside it; a diagonal would cut through the housing rim.
			route = [start, new Vector(end.x, end.y, start.z), end];
		} else route = [start, end];
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var axis = AssemblyFrames.transformVector(seat, 0, 0, 1);
		var parts = [Solids.named(Part.cylinderSpan(radius, 0, cap + radius), "input-cap"),
			Solids.named(Part.cylinderAlong(radius,
				new Vector(seat.x - axis.x * (cap + radius), seat.y - axis.y * (cap + radius),
					seat.z - axis.z * (cap + radius)), new Vector(axis.x, axis.y, axis.z), cap + radius), "output-cap")];
		var holes:Array<Part> = [];
		for (i in 0...route.length - 1) {
			var start = route[i], end = route[i + 1];
			var delta = new Vector(end.x - start.x, end.y - start.y, end.z - start.z);
			var length = Math.sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
			if (length <= 1e-6) continue;
			parts.push(Solids.named(Part.cylinderAlong(radius, start, delta, length), 'tube$i'));
			if (detail != Envelope && length > 2 * wall) {
				var fraction = wall / length;
				holes.push(Solids.named(Part.cylinderAlong(radius - wall,
					new Vector(start.x + delta.x * fraction, start.y + delta.y * fraction,
						start.z + delta.z * fraction), delta, length - 2 * wall), 'bore$i'));
			}
		}
		return Solids.cut(Solids.union(parts), holes);
	}
}
