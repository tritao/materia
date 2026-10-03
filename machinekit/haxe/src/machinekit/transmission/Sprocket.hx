package machinekit.transmission;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentValue.*;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.Dimension;
import materia.project.MaterialLibrary;

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

/** Roller chain sprocket with a simplified tooth outline (straight flanks between root and tip
 * radii, not the true ANSI B29.1 seating-curve profile), extruded along local +Z.
 * Diameters follow the standard relations: pitch diameter p / sin(pi/z), root (seating) diameter
 * pitch diameter minus the roller diameter, outside diameter p * (0.6 + cot(pi/z)). The roller
 * generic constructor defaults roller diameter to 0.625 p; `forChain` uses tabulated
 * nominal pitch and roller diameter from a specific ANSI chain family.
 * CAD frame: front face at z=0, back face at z=thickness, matching `DeepGrooveBearing`.
 * Connectors: `front`, `back` (faces) and `axis` (mid-thickness), all with +Y along +Z.
 */
class Sprocket extends MachineComponent {
	/** Resolve the coupling from these parts. */
	public static function relation(sprocket:Sprocket, alignment:Float):TransmissionRelation
		return new TransmissionRelation(alignment * 2 / sprocket.pitchDiameter, 0.97, null, null,
			machinekit.assembly.DriveDefaults.BELT_DRAG);

	static var chainTable:Null<Catalog<RollerChainSpec>>;
	public final chain:Null<String>;

	public static function chainCatalog():Catalog<RollerChainSpec> {
		if (chainTable == null)
			chainTable = new Catalog("roller chain", spec -> spec.designation, [
				{designation: "ANSI25", pitch: 6.35, rollerDiameter: 3.30},
				{designation: "ANSI35", pitch: 9.525, rollerDiameter: 5.08},
				{designation: "ANSI40", pitch: 12.7, rollerDiameter: 7.92},
				{designation: "ANSI50", pitch: 15.875, rollerDiameter: 10.16},
			], _ -> ({source: "https://www.renold.com/media/4131298/renold-as-roller-chain-uk.pdf",
				standard: "ANSI B29.1", standardEdition: null, dimensionKind: Nominal,
				conformance: GenericApproximation}));
		return chainTable;
	}

	public static function forChain(chain:String, teeth:Int, boreDiameter:Float, thickness:Float):Sprocket {
		var spec = chainCatalog().get(chain);
		return new Sprocket(spec.pitch, teeth, boreDiameter, thickness, spec.rollerDiameter, chain);
	}

	public final pitch:Float;
	public final rollerDiameter:Float;
	public final teeth:Int;
	public final boreDiameter:Float;
	public final thickness:Float;
	public final pitchDiameter:Float;
	public final outsideDiameter:Float;
	public final rootDiameter:Float;

	public function new(pitch:Float, teeth:Int, boreDiameter:Float, thickness:Float, ?rollerDiameter:Float, ?chain:String) {
		if (chain != null) {
			var spec = chainCatalog().get(chain);
			if (pitch != spec.pitch || rollerDiameter != spec.rollerDiameter)
				throw "Sprocket chain pitch and roller diameter must match its catalog entry";
		}
		if (!(pitch > 0)) throw "Sprocket needs a positive chain pitch";
		if (teeth < 8) throw "Sprocket needs at least 8 teeth";
		if (!(boreDiameter > 0)) throw "Sprocket needs a positive bore diameter";
		if (!(thickness > 0)) throw "Sprocket needs a positive thickness";
		var roller = rollerDiameter == null ? 0.625 * pitch : rollerDiameter;
		if (!(roller > 0) || !(roller < pitch)) throw "Sprocket roller diameter must be positive and less than the pitch";
		var pitchDia = pitch / Math.sin(Math.PI / teeth);
		var outsideDia = pitch * (0.6 + Math.cos(Math.PI / teeth) / Math.sin(Math.PI / teeth));
		var rootDia = pitchDia - roller;
		var rootRadius = rootDia / 2;
		var maxBore = 2 * rootRadius * Math.cos(0.325 * 2 * Math.PI / teeth);
		if (!(maxBore > boreDiameter))
			throw "Sprocket bore must clear the tooth-root land; use more teeth or a smaller bore";
		var pitchText = Dimension.format(pitch);
		super(chain == null ? 'GENERIC-SPROCKET-P$pitchText-${teeth}T' : 'SPROCKET-${chain}-${teeth}T',
			chain == null ? 'Generic sprocket, $pitchText mm pitch, ${teeth} teeth' : '$chain sprocket, ${teeth} teeth', "steel");
		this.chain = chain;
		this.pitch = pitch;
		this.rollerDiameter = roller;
		this.teeth = teeth;
		this.boreDiameter = boreDiameter;
		this.thickness = thickness;
		pitchDiameter = pitchDia;
		outsideDiameter = outsideDia;
		rootDiameter = rootDia;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, thickness / 2));
		addConnector("back", Face, Solids.axial(0, 0, thickness));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Part.prism(profile(), 0, thickness);
		return Solids.cut(body, [Part.cylinderSpan(boreDiameter / 2, -0.1, thickness + 0.1)]);
	}

	function profile():Array<Vector> {
		var rf = rootDiameter / 2, ra = outsideDiameter / 2;
		var angleStep = 2 * Math.PI / teeth;
		var halfRoot = 0.35 * angleStep / 2, halfTip = 0.2 * angleStep / 2;
		var points:Array<Vector> = [];
		for (i in 0...teeth) {
			var center = i * angleStep;
			points.push(new Vector(rf * Math.cos(center - halfRoot), rf * Math.sin(center - halfRoot)));
			points.push(new Vector(ra * Math.cos(center - halfTip), ra * Math.sin(center - halfTip)));
			points.push(new Vector(ra * Math.cos(center + halfTip), ra * Math.sin(center + halfTip)));
			points.push(new Vector(rf * Math.cos(center + halfRoot), rf * Math.sin(center + halfRoot)));
		}
		return points;
	}

	private static var chainRecipeTypeCache:Null<ComponentType>;

	public static function chainRecipeType():ComponentType {
		if (chainRecipeTypeCache == null)
			chainRecipeTypeCache = new ComponentType("machinekit.transmission.sprocket",
			[ComponentRecipeSupport.count("teeth", 20), ComponentRecipeSupport.length("boreDiameter", 8),
				ComponentRecipeSupport.length("thickness", 5), ComponentRecipeSupport.catalog("chain", Sprocket.chainCatalog(), "ANSI25")],
			v -> Sprocket.forChain(v.token("chain"), v.integer("teeth"), v.number("boreDiameter"), v.number("thickness")));
		return chainRecipeTypeCache;
	}

	private static var genericRecipeTypeCache:Null<ComponentType>;

	public static function genericRecipeType():ComponentType {
		if (genericRecipeTypeCache == null)
			genericRecipeTypeCache = new ComponentType("machinekit.transmission.generic-sprocket",
			[ComponentRecipeSupport.length("pitch", 6.35), ComponentRecipeSupport.count("teeth", 20), ComponentRecipeSupport.length("boreDiameter", 8),
				ComponentRecipeSupport.length("thickness", 5), ComponentRecipeSupport.length("rollerDiameter", 3.96875)],
			v -> new Sprocket(v.number("pitch"), v.integer("teeth"), v.number("boreDiameter"),
				v.number("thickness"), v.number("rollerDiameter")));
		return genericRecipeTypeCache;
	}

	override public function componentType():Null<ComponentType>
		return chain == null ? genericRecipeType() : chainRecipeType();

	override public function values():ComponentValues {
		if (chain == null)
		return new ComponentValues().setNumber("pitch", this.pitch)
				.setInteger("teeth", this.teeth)
				.setNumber("boreDiameter", this.boreDiameter)
				.setNumber("thickness", this.thickness)
				.setNumber("rollerDiameter", this.rollerDiameter).setToken("material", materialSpec());
		return new ComponentValues().setInteger("teeth", this.teeth)
				.setNumber("boreDiameter", this.boreDiameter)
				.setNumber("thickness", this.thickness)
				.setToken("chain", this.chain).setToken("material", materialSpec());
	}

}
