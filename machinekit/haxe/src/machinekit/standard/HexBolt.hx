package machinekit.standard;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentValue.*;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ToolSpec;
import machinekit.component.Dimension;
import cadkit.threads.ThreadSpec;
import machinekit.standard.PropertyClass.PropertyClasses;
import machinekit.standard.FastenerTightening.TighteningTableRow;

import cadkit.modeling.Part;
import cadkit.threads.Threads;
import cadkit.threads.ThreadedFace.ThreadStart;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** ISO 4017 hex bolt, threaded to the head. Threads are semantic (diameter, pitch, length), not modelled.
 * CAD frame: head bearing face at z=0, head toward +Z, shank toward -Z, matching
 * `SocketHeadCapScrew`. Connectors: `head` at the bearing face and `tip` at z=-length, both
 * with +Y along +Z. Envelope approximates the head as its across-corners cylinder.
 */
class HexBolt extends MachineComponent implements ScrewFastener {
	static var table:Null<Catalog<HexBoltSpec>>;

	public final spec:HexBoltSpec;
	public final length:Float;
	public final propertyClass:PropertyClass;
	public final drive:ScrewDrive;
	public final thread:ThreadSpec;
	public var diameter(get, never):Float;
	public var pitch(get, never):Float;
	/** Threaded length measured from the tip; ISO 4017 bolts are threaded over their full length. */
	public var threadLength(get, never):Float;
	/** Distance across the head's corners, for clearance around the hex flats. */
	public var acrossCorners(get, never):Float;

	static function rows():Array<HexBoltSpec>
		return [
			{size: "M3", diameter: 3, pitch: 0.5, acrossFlats: 5.5, headHeight: 2,
				tapDrill: 2.5, clearanceFine: 3.2, clearanceMedium: 3.4, clearanceCoarse: 3.6},
			{size: "M4", diameter: 4, pitch: 0.7, acrossFlats: 7, headHeight: 2.8,
				tapDrill: 3.3, clearanceFine: 4.3, clearanceMedium: 4.5, clearanceCoarse: 4.8},
			{size: "M5", diameter: 5, pitch: 0.8, acrossFlats: 8, headHeight: 3.5,
				tapDrill: 4.2, clearanceFine: 5.3, clearanceMedium: 5.5, clearanceCoarse: 5.8},
			{size: "M6", diameter: 6, pitch: 1.0, acrossFlats: 10, headHeight: 4,
				tapDrill: 5.0, clearanceFine: 6.4, clearanceMedium: 6.6, clearanceCoarse: 7},
			{size: "M8", diameter: 8, pitch: 1.25, acrossFlats: 13, headHeight: 5.3,
				tapDrill: 6.8, clearanceFine: 8.4, clearanceMedium: 9, clearanceCoarse: 10},
			{size: "M10", diameter: 10, pitch: 1.5, acrossFlats: 16, headHeight: 6.4,
				tapDrill: 8.5, clearanceFine: 10.5, clearanceMedium: 11, clearanceCoarse: 12},
			{size: "M12", diameter: 12, pitch: 1.75, acrossFlats: 18, headHeight: 7.5,
				tapDrill: 10.2, clearanceFine: 13, clearanceMedium: 13.5, clearanceCoarse: 14.5},
		];

	public static function catalog():Catalog<HexBoltSpec> {
		if (table == null)
			table = new Catalog("hex bolt size", spec -> spec.size, rows(), spec -> spec.size == "M5"
				? ({source: "https://catalogue.wurth.com.au/frontend/catalogs/803200/10/pdf/save/bk_1563.pdf", standard: "ISO 4017",
					standardEdition: null, dimensionKind: Unverified, conformance: NominalEnvelope,
					sources: ["https://norelem.co.uk/medias/Technische-Hinweise-Schrauben-Muttern-EN.pdf"],
					verifiedFields: ["acrossFlats", "headHeight", "tapDrill", "clearanceFine", "clearanceMedium"]})
				: ({source: "MachineKit embedded nominal table; source verification pending", standard: "ISO 4017",
					standardEdition: null, dimensionKind: Unverified, conformance: NominalEnvelope}));
		return table;
	}

	public static function metric(size:String, length:Float, propertyClass:PropertyClass = Steel88):HexBolt
		return new HexBolt(catalog().get(size), length, propertyClass);

	public static function custom(spec:HexBoltSpec, length:Float, propertyClass:PropertyClass = Steel88):HexBolt
		return new HexBolt(spec, length, propertyClass, true);

	private function new(spec:HexBoltSpec, length:Float, propertyClass:PropertyClass = Steel88, codeOnly:Bool = false) {
		if (!(length > 0) || !Math.isFinite(length)) throw 'Bolt ${spec.size} needs a positive length';
		if (propertyClass == null) throw 'Bolt ${spec.size} needs a property class';
		if (!(spec.diameter > 0) || !(spec.acrossFlats > spec.diameter) || !(spec.headHeight > 0)
			|| !(spec.tapDrill < spec.diameter) || !(spec.clearanceFine > spec.diameter))
			throw 'Bolt ${spec.size} has inconsistent dimensions';
		var name = '${spec.size}x${Dimension.format(length)}';
		var classSuffix = propertyClass == Steel88 ? "" : '-' + PropertyClasses.token(propertyClass);
		var designation = 'ISO4017-$name$classSuffix';
		var customName = '${spec.size}-D${Dimension.format(spec.diameter)}-P${Dimension.format(spec.pitch)}-AF${Dimension.format(spec.acrossFlats)}' +
			'-H${Dimension.format(spec.headHeight)}-TD${Dimension.format(spec.tapDrill)}' +
			'-C${Dimension.format(spec.clearanceFine)}x${Dimension.format(spec.clearanceMedium)}x${Dimension.format(spec.clearanceCoarse)}' +
			'-L${Dimension.format(length)}-PC${PropertyClasses.token(propertyClass)}';
		super(codeOnly ? customDesignation(customName) : designation, 'Hex bolt $name$classSuffix', PropertyClasses.material(propertyClass), codeOnly);
		this.spec = spec;
		this.length = length;
		this.propertyClass = propertyClass;
		drive = HexHead(spec.acrossFlats);
		thread = FastenerThreads.metric(spec.diameter, spec.pitch);
		addConnector("head", Face, Solids.axial(0, 0, 0));
		addConnector("tip", Face, Solids.axial(0, 0, -length));
		addFacet(new machinekit.component.CollisionHullFacet([
			machinekit.component.CollisionHullFacet.cylinder(diameter/2,-length,0),
			machinekit.component.CollisionHullFacet.prism(cadkit.modeling.Polygon.regular(6,spec.acrossFlats),0,spec.headHeight)]));
	}

	override public function hasGeometry():Bool return true;

	public function screwDrive():ScrewDrive return drive;
	public function driveFrame():materia.assembly.AssemblyRecord.AssemblyFrame
		return {x:0,y:0,z:spec.headHeight,qx:1,qy:0,qz:0,qw:0};
	public function threadSpec():ThreadSpec return thread;
	public function fastenerClass():PropertyClass return propertyClass;
	public function shankLength():Float return length;
	public function threadedLength():Float return threadLength;
	public function tightening():TighteningTableRow return FastenerTightening.forThread(thread, propertyClass);

	override public function setMaterial(spec:String):Void {
		if (spec != PropertyClasses.material(propertyClass))
			throw 'Fastener $designation material is determined by property class ${PropertyClasses.token(propertyClass)}';
		super.setMaterial(spec);
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var shank = Solids.named(Part.cylinderSpan(diameter / 2, -length, 0), "shank");
		var head = detail == Envelope
			? Part.cylinderSpan(acrossCorners / 2, 0, spec.headHeight)
			: Part.prism(cadkit.modeling.Polygon.regular(6, spec.acrossFlats), 0, spec.headHeight);
		var threaded = new Part(Threads.attach(shank.shape, "shank:cyl.side", thread, AxisMaximum, threadLength));
		shank.close();
		return Solids.union([head, threaded]);
	}

	public function clearanceDiameter(fit:ClearanceFit = Medium):Float {
		return switch (fit) {
			case Fine: spec.clearanceFine;
			case Medium: spec.clearanceMedium;
			case Coarse: spec.clearanceCoarse;
		}
	}

	/** Through-hole tool from z=0 down to z=-depth. */
	public function clearanceHole(depth:Float, fit:ClearanceFit = Medium):Part
		return Part.cylinderSpan(clearanceDiameter(fit) / 2, -depth, 0);

	/** Tap-drill tool from z=0 down to z=-depth; the named bore carries its internal thread. */
	public function tapHole(depth:Float):Part
		return Threads.tappedBore(thread.mating(), spec.tapDrill, depth);

	/** Counterbored through-hole tool, sized with a 0.5 mm diametral clearance around the hex
	 * head's corners and 0.5 mm axial clearance above it. The bolt's `head` sits at z=-headHeight-0.5.
	 */
	public function counterboreHole(depth:Float, fit:ClearanceFit = Medium):Part {
		var seatDepth = spec.headHeight + 0.5;
		if (!(depth > seatDepth)) throw 'Counterbore for ${spec.size} needs depth over $seatDepth';
		return Solids.union([
			Part.cylinderSpan(clearanceDiameter(fit) / 2, -depth, 0),
			Part.cylinderSpan((acrossCorners + 0.5) / 2, -seatDepth, 0),
		]);
	}

	function get_diameter():Float return spec.diameter;
	function get_pitch():Float return spec.pitch;
	function get_threadLength():Float return length;
	function get_acrossCorners():Float return spec.acrossFlats / Math.cos(Math.PI / 6);

	override public function toolSpecs():Array<ToolSpec> return [
		new ToolSpec("clearanceHole", [ComponentRecipeSupport.toolDepth(length),
			ComponentRecipeSupport.choice("fit", ["Fine", "Medium", "Coarse"], "Medium")]),
		new ToolSpec("tapHole", [ComponentRecipeSupport.toolDepth(length)]),
		new ToolSpec("counterboreHole", [ComponentRecipeSupport.toolDepth(Math.max(length, spec.headHeight + 1)),
			ComponentRecipeSupport.choice("fit", ["Fine", "Medium", "Coarse"], "Medium")])
	];

	override function buildTool(name:String, values:ComponentValues):Part {
		var depth = values.number("depth");
		return switch name {
			case "clearanceHole": clearanceHole(depth, ComponentRecipeSupport.clearanceFit(values.token("fit")));
			case "tapHole": tapHole(depth);
			case "counterboreHole": counterboreHole(depth, ComponentRecipeSupport.clearanceFit(values.token("fit")));
			default: super.buildTool(name, values);
		};
	}

	private static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null)
			recipeTypeCache = new ComponentType("machinekit.standard.hex-bolt",
			[ComponentRecipeSupport.catalog("size", HexBolt.catalog(), "M5"), ComponentRecipeSupport.length("length", 20),
				ComponentRecipeSupport.choice("propertyClass", PropertyClasses.tokens(), "8.8")],
			v -> HexBolt.metric(v.token("size"), v.number("length"), PropertyClasses.parse(v.token("propertyClass"))),
			true, true, null, false);
		return recipeTypeCache;
	}

	override public function componentType():Null<ComponentType> return codeOnly ? null : recipeType();

	override public function values():ComponentValues {
		return new ComponentValues().set("size", Token(this.spec.size))
				.set("length", Number(this.length)).setToken("propertyClass", PropertyClasses.token(propertyClass));
	}

}
