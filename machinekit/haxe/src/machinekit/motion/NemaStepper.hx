package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.ClearanceFit;
import machinekit.standard.SocketHeadCapScrew;

/** NEMA ICS 16 hybrid stepper frame in millimetres. `mountHoleDepth` is the tapped or flange depth. */
typedef NemaFrameSpec = {
	var frame:Int;
	var face:Float;
	var boltSpacing:Float;
	var mountScrew:String;
	var tappedMount:Bool;
	var mountHoleDepth:Float;
	var pilotDiameter:Float;
	var pilotHeight:Float;
	var shaftDiameter:Float;
	var shaftLength:Float;
	var bodyLength:Float;
}

/** Stepper motor with standard face, pilot, bolt pattern, and output shaft.
 * CAD frame: mounting face at z=0, body toward -Z, shaft along +Z.
 * Connectors: `mountFace`, `shaftAxis` (rotor axis at the face), `shaftTip`, and `bolt1`..`bolt4`,
 * all with +Y along +Z.
 */
class NemaStepper extends MachineComponent {
	static var table:Null<Catalog<NemaFrameSpec>>;

	public final spec:NemaFrameSpec;
	public final bodyLength:Float;

	static function rows():Array<NemaFrameSpec>
		return [
			{frame: 17, face: 42.3, boltSpacing: 31.0, mountScrew: "M3", tappedMount: true,
				mountHoleDepth: 4.5, pilotDiameter: 22.0, pilotHeight: 2.0, shaftDiameter: 5.0,
				shaftLength: 24.0, bodyLength: 48.0},
			{frame: 23, face: 56.4, boltSpacing: 47.14, mountScrew: "M5", tappedMount: false,
				mountHoleDepth: 5.0, pilotDiameter: 38.1, pilotHeight: 1.6, shaftDiameter: 6.35,
				shaftLength: 21.0, bodyLength: 56.0},
			{frame: 34, face: 86.0, boltSpacing: 69.6, mountScrew: "M6", tappedMount: false,
				mountHoleDepth: 8.0, pilotDiameter: 73.0, pilotHeight: 1.6, shaftDiameter: 14.0,
				shaftLength: 32.0, bodyLength: 80.0},
		];

	public static function catalog():Catalog<NemaFrameSpec> {
		if (table == null)
			table = new Catalog("NEMA frame", spec -> Std.string(spec.frame), rows(), _ -> ({source: "MachineKit mixed NEMA interface and representative motor dimensions; source verification pending", standard: null,
				standardEdition: null, dimensionKind: Mixed, conformance: GenericApproximation}));
		return table;
	}

	public static function frame(size:Int, ?bodyLength:Float):NemaStepper
		return new NemaStepper(catalog().get(Std.string(size)), bodyLength);

	public function new(spec:NemaFrameSpec, ?bodyLength:Float) {
		var length = bodyLength == null ? spec.bodyLength : bodyLength;
		if (!(length > spec.mountHoleDepth)) throw 'NEMA ${spec.frame} body is too short';
		if (!(spec.boltSpacing > 0) || !(spec.face > spec.boltSpacing) || !(spec.pilotDiameter > spec.shaftDiameter)
			|| !(spec.pilotDiameter < spec.boltSpacing * Math.sqrt(2)) || !(spec.shaftDiameter > 0) || !(spec.shaftLength > 0))
			throw 'NEMA ${spec.frame} frame has inconsistent dimensions';
		var bodyText = Dimension.format(length);
		super('NEMA${spec.frame}-$bodyText', 'NEMA ${spec.frame} stepper motor, $bodyText mm body', null);
		this.spec = spec;
		this.bodyLength = length;
		addConnector("mountFace", Mount, Solids.axial(0, 0, 0));
		addConnector("shaftAxis", Axis, Solids.axial(0, 0, 0));
		addConnector("shaftTip", Shaft, Solids.axial(0, 0, spec.shaftLength));
		var i = 1;
		for (point in boltPattern()) addConnector('bolt${i++}', Mount, Solids.axial(point.x, point.y, 0));
	}

	/** Bolt centres on the mounting face, counter-clockwise from (+,+). */
	public function boltPattern():Array<{x:Float, y:Float}> {
		var h = spec.boltSpacing / 2;
		return [{x: h, y: h}, {x: -h, y: h}, {x: -h, y: -h}, {x: h, y: -h}];
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var half = spec.face / 2;
		var parts:Array<Part> = [];
		if (detail == Envelope) {
			parts.push(Solids.prism([new Vector(-half, -half), new Vector(half, -half),
				new Vector(half, half), new Vector(-half, half)], -bodyLength, 0));
		} else {
			var c = 0.08 * spec.face;
			var body = Solids.prism([new Vector(-half + c, -half), new Vector(half - c, -half),
				new Vector(half, -half + c), new Vector(half, half - c), new Vector(half - c, half),
				new Vector(-half + c, half), new Vector(-half, half - c), new Vector(-half, -half + c)],
				-bodyLength, 0);
			var screw = mountScrew(10);
			var holeDiameter = spec.tappedMount ? screw.spec.tapDrill : screw.clearanceDiameter(Medium);
			parts.push(Solids.cut(body, [for (point in boltPattern())
				Solids.cylinder(holeDiameter / 2, -spec.mountHoleDepth, 0.1, point.x, point.y)]));
		}
		parts.push(Solids.cylinder(spec.pilotDiameter / 2, 0, spec.pilotHeight));
		parts.push(Solids.cylinder(spec.shaftDiameter / 2, 0, spec.shaftLength));
		return Solids.union(parts);
	}

	/** Screw that fits the motor's mounting holes. */
	public function mountScrew(length:Float):SocketHeadCapScrew
		return SocketHeadCapScrew.metric(spec.mountScrew, length);

	/** Cutting tool for a plate in front of the face (z=0..thickness): pilot and bolt clearances. */
	public function mountingCutout(thickness:Float, pilotClearance:Float = 0.2, fit:ClearanceFit = Medium):Part {
		if (!(thickness > 0)) throw "Motor mounting plate needs a positive thickness";
		var radius = mountScrew(10).clearanceDiameter(fit) / 2;
		var tools = [Solids.cylinder((spec.pilotDiameter + pilotClearance) / 2, 0, thickness)];
		for (point in boltPattern()) tools.push(Solids.cylinder(radius, 0, thickness, point.x, point.y));
		return Solids.union(tools);
	}
}
