package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.ClearanceFit;
import machinekit.standard.BearingFit.BearingHousingFit;
import machinekit.standard.BearingFit;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.SocketHeadCapScrew;

/** Compact flange-style bearing housing: a square flange with a central bore for `bearing` and
 * four mounting screws around it, all along the bore axis -- the same face/bolt-circle layout as
 * `NemaStepper`, not a classic two-bolt pillow block whose base is perpendicular to the bore.
 * CAD frame: mounting face at z=0, flange toward +Z. Connectors: `bore` (axis, mid-depth) for the
 * bearing, and `bolt1`..`bolt4` (mount), all with +Y along +Z.
 */
class PillowBlockHousing extends MachineComponent {
	public final bearing:DeepGrooveBearing;
	public final face:Float;
	public final depth:Float;
	public final boltSpacing:Float;
	public final mountScrew:String;
	public final fit:BearingHousingFit;
	public final allowance:Float;

	public function new(bearing:DeepGrooveBearing, fit:BearingHousingFit = Slip) {
		var wall = Math.max(6, bearing.outside * 0.2);
		var depth = bearing.width + wall;
		var mountScrew = bearing.bore <= 12 ? "M5" : bearing.bore <= 20 ? "M6" : "M8";
		// Bolts sit mid-wall on the diagonal; the face grows so each screw head keeps 1 mm of edge.
		var boltSpacing = bearing.outside + wall;
		var edge = Math.max(wall / 2, SocketHeadCapScrew.catalog().get(mountScrew).headDiameter / 2 + 1);
		var face = boltSpacing + 2 * edge;
		super('PILLOWBLOCK-${bearing.designation}', 'Flange bearing housing for ${bearing.designation}', "cast iron");
		this.bearing = bearing;
		this.face = face;
		this.depth = depth;
		this.boltSpacing = boltSpacing;
		this.mountScrew = mountScrew;
		this.fit = fit;
		this.allowance = BearingFit.housingAllowance(fit, bearing.outside);
		addConnector("bore", Axis, Solids.axial(0, 0, depth / 2));
		var i = 1;
		for (point in boltPattern()) addConnector('bolt${i++}', Mount, Solids.axial(point.x, point.y, 0));
	}

	/** Bolt centres on the mounting face, counter-clockwise from (+,+). */
	public function boltPattern():Array<{x:Float, y:Float}> {
		var h = boltSpacing / 2;
		return [{x: h, y: h}, {x: -h, y: h}, {x: -h, y: -h}, {x: h, y: -h}];
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Part.box(face, face, depth);
		var boreTool = Solids.cylinder((bearing.outside + allowance) / 2, -0.1, depth + 0.1);
		if (detail == Envelope) return Solids.cut(body, [boreTool]);
		var screw = mountScrewPart(10);
		var tools = [boreTool];
		for (point in boltPattern())
			tools.push(Solids.cylinder(screw.clearanceDiameter(Medium) / 2, -0.1, depth + 0.1, point.x, point.y));
		return Solids.cut(body, tools);
	}

	/** Screw that fits the housing's mounting holes. */
	public function mountScrewPart(length:Float):SocketHeadCapScrew
		return SocketHeadCapScrew.metric(mountScrew, length);
}
