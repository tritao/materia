package machinekit.robotics;

import cadkit.modeling.Part;
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

/** ISO 9409-1 mechanical interface size: pitch circle d1, bolt count and thread, centring
 * diameter d2, and locating pin diameter d7, in millimetres.
 */
typedef RobotFlangeSpec = {
	var pitchCircle:Float;
	var boltCount:Int;
	var screw:String;
	var pilotDiameter:Float;
	var pinDiameter:Float;
}

/** Robot tool flange using the ISO 9409-1 bolt pattern: a round plate with a centring pilot, a bolt circle, and a
 * locating pin hole. The flange is sized by its pitch circle (`new RobotFlange(50)` is
 * based on the ISO 9409-1-50-4-M6 pattern): pitch circle, bolt count and thread, pilot and pin diameters come from the
 * standard's table; the outer diameter (pitch circle plus two screw-head diameters), plate
 * thickness (1.5 screw diameters), and pilot height (half a screw diameter) are proportional. The
 * pilot is modelled as a raised boss that engages a recess in the mating part (ISO specifies the
 * robot side's d2 as an H7 recess; the boss/recess roles are swapped here for a simpler mate).
 * Passing a `boltCount` that differs from the table gives a non-standard pattern whose
 * designation drops the ISO pattern prefix.
 *
 * CAD frame: mounting face at z=0, flange plate behind it (z=-thickness..0), pilot boss toward +Z
 * (z=0..pilotHeight). Connectors: `face` (the mounting face), `bolt1`..`boltN` and `pin`, all at
 * z=0 with +Y along +Z, pointing out of the face into the mated part. A mated part's `Mount`
 * connector follows the same rule, so `mate(flange.face, part.mount)` stacks the part in front of
 * the face; `mountingCutout` is the matching tool in that part's frame.
 */
class RobotFlange extends MachineComponent {
	static var table:Null<Catalog<RobotFlangeSpec>>;
	static inline var MIN_WEB:Float = 1.0;

	public final spec:RobotFlangeSpec;
	public final flangeDiameter:Float;
	public final thickness:Float;
	public final pilotDiameter:Float;
	public final pilotHeight:Float;
	public final boltCircleDiameter:Float;
	public final boltCount:Int;
	public final mountScrew:String;
	public final pinDiameter:Float;

	static function rows():Array<RobotFlangeSpec>
		return [
			row(31.5, 4, "M5", 20, 5),
			row(40, 4, "M6", 25, 6),
			row(50, 4, "M6", 31.5, 6),
			row(63, 4, "M6", 40, 6),
			row(80, 6, "M8", 50, 8),
			row(100, 6, "M8", 63, 8),
			row(125, 6, "M10", 80, 10),
			row(160, 6, "M10", 100, 10),
		];

	/** ISO 9409-1 sizes keyed by pitch circle diameter text ("31.5", "50", ...). */
	public static function catalog():Catalog<RobotFlangeSpec> {
		if (table == null)
			table = new Catalog("ISO 9409-1 flange size", spec -> Dimension.format(spec.pitchCircle), rows(), _ -> ({source: "MachineKit ISO 9409-1 pattern table; source verification pending", standard: "ISO 9409-1",
				standardEdition: null, dimensionKind: Unverified, conformance: GenericApproximation}));
		return table;
	}

	/** `pitchCircleDiameter` selects the ISO 9409-1 size; `boltCount` overrides its bolt count. */
	public function new(pitchCircleDiameter:Float, ?boltCount:Int) {
		if (!(pitchCircleDiameter > 0)) throw "Robot flange needs a positive diameter";
		var spec = catalog().get(Dimension.format(pitchCircleDiameter));
		var count:Int = boltCount == null ? spec.boltCount : boltCount;
		if (count < 3) throw "Robot flange needs at least 3 bolts";
		var screw = SocketHeadCapScrew.metric(spec.screw, 10);
		var radius = spec.pitchCircle / 2;
		// The pin sits half a bolt spacing from its neighbours: its hole must clear theirs, and
		// adjacent screw heads must clear each other.
		var pinToBolt = 2 * radius * Math.sin(Math.PI / (2 * count));
		var boltToBolt = 2 * radius * Math.sin(Math.PI / count);
		if (!(pinToBolt >= (screw.clearanceDiameter(Medium) + spec.pinDiameter) / 2 + MIN_WEB) ||
			!(boltToBolt >= screw.spec.headDiameter + MIN_WEB))
			throw 'Robot flange ${Dimension.format(spec.pitchCircle)} has too many bolts for its pitch circle';
		var size = '${Dimension.format(spec.pitchCircle)}-$count-${spec.screw}';
		var standardPattern = count == spec.boltCount;
		super(standardPattern ? 'ISO9409-PATTERN-$size' : 'FLANGE-$size',
			standardPattern ? 'Robot flange with ISO 9409-1-$size bolt pattern and raised pilot' : 'Robot flange $size (non-standard ISO 9409-1 bolt count)',
			"steel");
		this.spec = spec;
		this.boltCount = count;
		mountScrew = spec.screw;
		boltCircleDiameter = spec.pitchCircle;
		pilotDiameter = spec.pilotDiameter;
		pinDiameter = spec.pinDiameter;
		flangeDiameter = spec.pitchCircle + 2 * screw.spec.headDiameter;
		thickness = 1.5 * screw.diameter;
		pilotHeight = 0.5 * screw.diameter;
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

	/** Locating pin position on the pitch circle, half a bolt spacing from the first bolt. */
	public function pinPoint():{x:Float, y:Float} {
		var r = boltCircleDiameter / 2, angle = Math.PI / boltCount;
		return {x: r * Math.cos(angle), y: r * Math.sin(angle)};
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var boss = Solids.cylinder(pilotDiameter / 2, 0, pilotHeight);
		var body = Solids.union([Solids.cylinder(flangeDiameter / 2, -thickness, 0), boss]);
		if (detail == Envelope) return body;
		var screw = mountScrewPart(10);
		var pin = pinPoint();
		var tools = [Solids.cylinder(pinDiameter / 2, -thickness - 0.1, 0.1, pin.x, pin.y)];
		for (point in boltPattern())
			tools.push(Solids.cylinder(screw.clearanceDiameter(Medium) / 2, -thickness - 0.1, 0.1, point.x, point.y));
		return Solids.cut(body, tools);
	}

	/** Screw that fits the flange's bolt circle. */
	public function mountScrewPart(length:Float):SocketHeadCapScrew
		return SocketHeadCapScrew.metric(mountScrew, length);

	/** Depth of the pilot recess `mountingCutout` cuts: the pilot height plus `pilotClearance`. */
	public function pilotRecessDepth(pilotClearance:Float = 0.2):Float
		return pilotHeight + pilotClearance;

	/** Cutting tool for a part mated to `face`, in that part's frame (mating face at z=0, material
	 * toward +Z, `depth` deep): a blind pilot recess `pilotRecessDepth` deep and `pilotClearance`
	 * over the pilot diameter, plus bolt and pin clearance holes through `depth`. Every tool
	 * overshoots the mating face by 0.1, and the holes overshoot `depth` by 0.1, so no cut face is
	 * coplanar with the part's faces.
	 */
	public function mountingCutout(depth:Float, pilotClearance:Float = 0.2, fit:ClearanceFit = Medium):Part {
		if (!(depth > pilotRecessDepth(pilotClearance)))
			throw "Robot flange mounting cutout must be deeper than the pilot recess";
		var screw = mountScrewPart(10);
		var pin = pinPoint();
		var tools = [
			Solids.cylinder((pilotDiameter + pilotClearance) / 2, -0.1, pilotRecessDepth(pilotClearance)),
			Solids.cylinder((pinDiameter + pilotClearance) / 2, -0.1, depth + 0.1, pin.x, pin.y),
		];
		for (point in boltPattern())
			tools.push(Solids.cylinder(screw.clearanceDiameter(fit) / 2, -0.1, depth + 0.1, point.x, point.y));
		return Solids.union(tools);
	}

	static function row(pitchCircle:Float, boltCount:Int, screw:String, pilotDiameter:Float,
			pinDiameter:Float):RobotFlangeSpec
		return {pitchCircle: pitchCircle, boltCount: boltCount, screw: screw, pilotDiameter: pilotDiameter,
			pinDiameter: pinDiameter};
}
