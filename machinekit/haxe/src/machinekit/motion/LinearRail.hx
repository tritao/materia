package machinekit.motion;

import cadkit.InertiaTensor;
import cadkit.modeling.Curve;
import cadkit.modeling.Part;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.LinearRailProfile.LinearRailProfileSpec;
import materia.assembly.AssemblyFrames;

/**
 * A cut length of profile rail for a catalogue carriage. Its drawing is one of two, chosen explicitly and saved with
 * it: `nominal`, the catalogue section with grooves (`ProfileRailShapes`), or `measured`, the running section taken
 * from the pinned HIWIN CAD (HG rails). The drawing changes neither its part number nor its datasheet mass. Mounting
 * holes are counterbored from the rail table and expose named connectors. CAD frame: the cross-section is centred
 * on x=0, spans y=-railHeight..0, and runs along +Z.
 */
class LinearRail extends MachineComponent {
	public static inline var NOMINAL = "nominal";
	public static inline var MEASURED = "measured";

	/** Runs along +Z with its top (+Y) up, whatever its cut length. */
	override public function presentationView():Null<machinekit.component.ComponentView>
		return machinekit.component.ComponentView.lengthAlongZ();

	public final spec:LinearRailProfileSpec;
	public final length:Float;
	public final holePositions:Array<Float>;
	/** `NOMINAL` or `MEASURED`. */
	public final drawing:String;

	/**
	 * The rail for carriage `profile`, drawn `measured` where the pinned CAD has its section and `nominal` otherwise,
	 * unless `drawing` says which.
	 */
	public static function metric(profile:String, length:Float, ?drawing:String):LinearRail {
		var spec = LinearRailSystem.catalog().get(profile);
		return new LinearRail(spec, length, drawing == null ? (hasMeasuredSection(spec) ? MEASURED : NOMINAL) : drawing);
	}

	public static function custom(spec:LinearRailProfileSpec, length:Float):LinearRail
		return new LinearRail(spec, length, NOMINAL, true);

	public static function hasMeasuredSection(spec:LinearRailProfileSpec):Bool
		return HgReferenceData.rails().indexOf(spec.rail) >= 0;

	function new(spec:LinearRailProfileSpec, length:Float, drawing:String, codeOnly:Bool = false) {
		if (!(length > 2 * spec.railEndMargin)) throw 'Linear rail length must exceed twice the end margin';
		if (!(spec.railWidth > 0) || !(spec.railHeight > 0) || !(spec.railHolePitch > 0) || !(spec.railEndMargin > 0))
			throw 'Linear rail profile "${spec.designation}" has invalid rail dimensions';
		if (drawing != NOMINAL && drawing != MEASURED) throw 'Unknown linear rail drawing "$drawing"';
		if (drawing == MEASURED && !hasMeasuredSection(spec)) throw 'No measured section for rail ${spec.rail}';
		var lengthText = Dimension.format(length);
		var customName = '${spec.family}-${spec.rail}-RW${Dimension.format(spec.railWidth)}-RH${Dimension.format(spec.railHeight)}' +
			'-HP${Dimension.format(spec.railHolePitch)}-EM${Dimension.format(spec.railEndMargin)}-L$lengthText';
		super(codeOnly ? customDesignation(customName) : '${spec.family}-${spec.rail}-L$lengthText',
			'${spec.family} ${spec.rail} profile rail, $lengthText mm long', "steel", codeOnly);
		this.spec = spec;
		this.length = length;
		this.drawing = drawing;
		holePositions = [];
		var position = spec.railEndMargin;
		while (position <= length - spec.railEndMargin + 1e-9) {
			holePositions.push(position);
			position += spec.railHolePitch;
		}
		for (z in holePositions)
			if (z < spec.railBore.counterbore / 2 || z > length - spec.railBore.counterbore / 2)
				throw 'Rail ${spec.rail} counterbore crosses its cut end';
		addConnector("axis", Axis, Solids.axial(0, 0, 0));
		// The face the rail is bolted down by, at its start: +Y (a connector's axis) up into the rail, +Z along it.
		addConnector("base", Mount, AssemblyFrames.translation(0, -spec.railHeight, 0));
		for (i in 0...holePositions.length)
			addConnector('mount${i + 1}', Mount,
				AssemblyFrames.alongY(0, 0, holePositions[i], 0, 1, 0));
		// Datasheet mass per metre, centred on the section's bounding box.
		var kg = spec.railMassKgPerMetre * length / 1000, w = spec.railWidth, h = spec.railHeight;
		declareMass(kg, new Vector(0, -h / 2, length / 2),
			new InertiaTensor(kg * (h * h + length * length) / 12, 0, 0, kg * (w * w + length * length) / 12, 0, kg * (w * w + h * h) / 12));
	}

	/** The rail table's counterbored mounting hole. */
	public function mountingHole():{diameter:Float, counterboreDiameter:Float, counterboreDepth:Float, reference:String}
		return {diameter: spec.railBore.through, counterboreDiameter: spec.railBore.counterbore, counterboreDepth: spec.railBore.depth,
			reference: LinearRailSystem.catalog().metadata(spec.designation).source};

	public function mountingConnectors():Array<machinekit.component.Connector>
		return [for (index in 0...holePositions.length) connector('mount${index+1}')];

	public function mountingThread():cadkit.threads.ThreadSpec {
		var parts=spec.railMountScrew.split("x");
		// Profile producers declare either a size or a size plus catalogue length.
		// The placed stack selects the actual length; only its thread is used here.
		if (parts.length<1 || parts.length>2 || (parts.length==2 && !~/^[0-9]+([.][0-9]+)?$/.match(parts[1])))
			throw "Invalid rail mounting screw catalogue token: "+spec.railMountScrew;
		var screw=machinekit.standard.SocketHeadCapScrew.catalog().get(parts[0]);
		return machinekit.standard.FastenerThreads.metric(screw.diameter,screw.pitch).mating();
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		if (detail == Envelope)
			return section();
		if (drawing == NOMINAL)
			return ProfileRailShapes.rail(spec, length, holePositions);
		var body = measuredSection(spec.rail, length), bore = spec.railBore;
		return Solids.cut(body, [for (z in holePositions) for (tool in [
			Part.cylinderAlongY(bore.through / 2, -spec.railHeight - 1, 1, 0, z),
			Part.cylinderAlongY(bore.counterbore / 2, -bore.depth, 1, 0, z)
		]) tool]);
	}

	/**
	 * The undrilled running section of HG rail `rail` from the pinned CAD, `length` long along +Z. Supplier STEP
	 * endpoints differ by up to 0.00000024 mm (HGR45R): segments weld within that tolerance, never across a gap.
	 */
	public static function measuredSection(rail:String, length:Float):Part {
		if (!Math.isFinite(length) || !(length > 0)) throw 'Rail $rail needs a finite positive length';
		var edges:Array<Curve> = [], wire:Null<Curve> = null, face:Null<Sketch> = null;
		try {
			var section = HgReferenceData.section(rail), first = new Vector(section[0][0], section[0][1]), current = first;
			for (i in 0...section.length) {
				var s = section[i], end = i == section.length - 1 ? first : new Vector(s[2], s[3]);
				if (Math.abs(current.x - s[0]) > 1e-6 || Math.abs(current.y - s[1]) > 1e-6) throw 'Disconnected $rail section';
				edges.push(s.length == 4 ? Curve.line(current, end) : Curve.arc(current, new Vector(s[4], s[5]), end));
				current = end;
			}
			wire = Curve.wire(edges);
			face = Sketch.face(wire);
			var result = face.extrude(length);
			face.close();
			wire.close();
			for (edge in edges) edge.close();
			return result;
		} catch (e:Dynamic) {
			if (face != null) face.close();
			if (wire != null) wire.close();
			for (edge in edges) edge.close();
			throw e;
		}
	}

	function section():Part
		return Part.prism([
			new Vector(-spec.railWidth / 2, -spec.railHeight),
			new Vector(spec.railWidth / 2, -spec.railHeight),
			new Vector(spec.railWidth / 2, 0),
			new Vector(-spec.railWidth / 2, 0),
		], 0, length);

	private static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null)
			recipeTypeCache = new ComponentType("machinekit.motion.linear-rail",
				[ComponentRecipeSupport.catalog("profile", LinearRailSystem.catalog(), "MGN12C"), ComponentRecipeSupport.length("length", 100),
					ComponentRecipeSupport.choice("geometry", [NOMINAL, MEASURED], NOMINAL)],
				v -> new LinearRail(LinearRailSystem.catalog().get(v.token("profile")), v.number("length"), v.token("geometry")),
				false, false, ["geometry"]);
		return recipeTypeCache;
	}

	override public function componentType():Null<ComponentType> return codeOnly ? null : recipeType();

	override public function values():ComponentValues {
		return new ComponentValues().setToken("profile", this.spec.designation)
				.setNumber("length", this.length).setToken("geometry", drawing).setToken("material", materialSpec());
	}

}
