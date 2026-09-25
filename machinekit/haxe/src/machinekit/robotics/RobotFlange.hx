package machinekit.robotics;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.ClearanceFit;
import machinekit.standard.SocketHeadCapScrew;

/** ISO 9409-1 style robot tool flange: a round mounting face with a raised centring pilot boss,
 * a bolt circle, and a locating pin hole, sized proportionally to the flange diameter rather than
 * from a literal standard table.
 * CAD frame: mounting face at z=0, pilot boss toward +Z. Connectors: `face` (the mounting face),
 * `bolt1`..`boltN` and `pin`, all at z=0 with +Y along +Z.
 */
class RobotFlange extends MachineComponent {
	public final flangeDiameter:Float;
	public final thickness:Float;
	public final pilotDiameter:Float;
	public final pilotHeight:Float;
	public final boltCircleDiameter:Float;
	public final boltCount:Int;
	public final mountScrew:String;
	public final pinDiameter:Float;

	public function new(flangeDiameter:Float, boltCount:Int = 4) {
		if (!(flangeDiameter > 0)) throw "Robot flange needs a positive diameter";
		if (boltCount < 3) throw "Robot flange needs at least 3 bolts";
		super('ISO9409-${flangeDiameter}', 'ISO 9409-1 style robot flange, ${flangeDiameter} mm', "steel");
		this.flangeDiameter = flangeDiameter;
		this.boltCount = boltCount;
		thickness = Math.max(6, 0.16 * flangeDiameter);
		pilotDiameter = 0.5 * flangeDiameter;
		pilotHeight = Math.max(2, 0.06 * flangeDiameter);
		boltCircleDiameter = 0.78 * flangeDiameter;
		pinDiameter = Math.max(2.5, 0.05 * flangeDiameter);
		mountScrew = flangeDiameter <= 40 ? "M4" : flangeDiameter <= 63 ? "M5" : flangeDiameter <= 100 ? "M6" : "M8";
		addConnector("face", Face, Solids.axial(0, 0, 0));
		var i = 1;
		for (point in boltPattern()) addConnector('bolt${i++}', Mount, Solids.axial(point.x, point.y, 0));
		var pin = pinPoint();
		addConnector("pin", Mount, Solids.axial(pin.x, pin.y, 0));
	}

	/** Bolt centres on the mounting face, counter-clockwise from angle 0. */
	public function boltPattern():Array<{x:Float, y:Float}> {
		var r = boltCircleDiameter / 2;
		return [for (i in 0...boltCount) {
			var angle = 2 * Math.PI * i / boltCount;
			{x: r * Math.cos(angle), y: r * Math.sin(angle)};
		}];
	}

	/** Locating pin position, offset half a bolt spacing from the first bolt. */
	public function pinPoint():{x:Float, y:Float} {
		var r = boltCircleDiameter / 2, angle = Math.PI / boltCount;
		return {x: r * Math.cos(angle), y: r * Math.sin(angle)};
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var boss = Solids.cylinder(pilotDiameter / 2, thickness, thickness + pilotHeight);
		var body = Solids.union([Solids.cylinder(flangeDiameter / 2, 0, thickness), boss]);
		if (detail == Envelope) return body;
		var screw = mountScrewPart(10);
		var pin = pinPoint();
		var tools = [Solids.cylinder(pinDiameter / 2, -0.1, thickness + 0.1, pin.x, pin.y)];
		for (point in boltPattern())
			tools.push(Solids.cylinder(screw.clearanceDiameter(Medium) / 2, -0.1, thickness + 0.1, point.x, point.y));
		return Solids.cut(body, tools);
	}

	/** Screw that fits the flange's bolt circle. */
	public function mountScrewPart(length:Float):SocketHeadCapScrew
		return SocketHeadCapScrew.metric(mountScrew, length);

	/** Cutting tool for a mating plate in front of the face (z=0..thickness): pilot clearance,
	 * bolt clearances, and the pin clearance.
	 */
	public function mountingCutout(plateThickness:Float, pilotClearance:Float = 0.2, fit:ClearanceFit = Medium):Part {
		if (!(plateThickness > 0)) throw "Robot flange mounting cutout needs a positive thickness";
		var screw = mountScrewPart(10);
		var pin = pinPoint();
		var tools = [
			Solids.cylinder((pilotDiameter + pilotClearance) / 2, 0, plateThickness),
			Solids.cylinder((pinDiameter + pilotClearance) / 2, 0, plateThickness, pin.x, pin.y),
		];
		for (point in boltPattern())
			tools.push(Solids.cylinder(screw.clearanceDiameter(fit) / 2, 0, plateThickness, point.x, point.y));
		return Solids.union(tools);
	}
}
