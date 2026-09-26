package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.Dimension;
import machinekit.component.Solids;

/** Catalog dimensions for a vendor extrusion profile. Slot dimensions are in the cross-section;
 * the profile is extruded along local +Z from z=0 to z=`length`.
 */
typedef TSlotProfileSpec = {
	var designation:String;
	var family:String;
	var width:Float;
	var height:Float;
	var slotWidth:Float;
	var slotDepth:Float;
	var throatDepth:Float;
	var tWidth:Float;
	var headDepth:Float;
	var boreDiameter:Float;
}

/** T-slot aluminium extrusion profile. The numeric constructor remains a generic proportional
 * approximation; `forProfile()` selects a catalog-backed profile with vendor dimensions.
 */
class TSlotExtrusion implements StructuralProfile {
	static var table:Null<Catalog<TSlotProfileSpec>>;

	public final designation:String;
	public final description:String;
	public final size:Float;
	public final height:Float;
	public final slotWidth:Float;
	public final slotDepth:Float;
	public final throatDepth:Float;
	public final tWidth:Float;
	public final headDepth:Float;
	public final boreDiameter:Float;
	public final family:Null<String>;

	public function new(size:Float, ?profile:TSlotProfileSpec) {
		if (!(size > 0)) throw "T-slot extrusion needs a positive size";
		if (profile != null) {
			if (!(profile.width > 0) || !(profile.height > 0) || !(profile.slotWidth > 0) || !(profile.slotDepth > 0) ||
				!(profile.throatDepth > 0) || !(profile.tWidth > profile.slotWidth) ||
				!(profile.headDepth > 0) || !(profile.boreDiameter > 0))
				throw 'Invalid T-slot profile "${profile.designation}"';
			this.size = profile.width;
			height = profile.height;
			slotWidth = profile.slotWidth;
			slotDepth = profile.slotDepth;
			throatDepth = profile.throatDepth;
			tWidth = profile.tWidth;
			headDepth = profile.headDepth;
			boreDiameter = profile.boreDiameter;
			family = profile.family;
			designation = profile.designation;
			description = '${profile.family} aluminium extrusion ${Dimension.format(profile.width)}x${Dimension.format(profile.height)}';
		} else {
			this.size = size;
			height = size;
			slotWidth = 0.3 * size;
			throatDepth = 0.15 * size;
			tWidth = 0.35 * size;
			headDepth = 0.1 * size;
			slotDepth = throatDepth + headDepth;
			boreDiameter = 0.15 * size;
			family = null;
			var sizeText = Dimension.format(size);
			designation = 'GENERIC-TSLOT-${sizeText}x$sizeText';
			description = 'Generic T-slot extrusion ${sizeText}x$sizeText';
		}
	}

	static function rows():Array<TSlotProfileSpec>
		return [
			{designation: "HFS5-2020", family: "MISUMI HFS5", width: 20, height: 20, slotWidth: 6, slotDepth: 6, throatDepth: 4, tWidth: 12, headDepth: 2, boreDiameter: 4.2},
			{designation: "HFS5-2040", family: "MISUMI HFS5", width: 20, height: 40, slotWidth: 6, slotDepth: 6, throatDepth: 4, tWidth: 12, headDepth: 2, boreDiameter: 4.2},
			{designation: "HFS5-2060", family: "MISUMI HFS5", width: 20, height: 60, slotWidth: 6, slotDepth: 6, throatDepth: 4, tWidth: 12, headDepth: 2, boreDiameter: 4.2},
			{designation: "HFS5-4040", family: "MISUMI HFS5", width: 40, height: 40, slotWidth: 6, slotDepth: 6, throatDepth: 4, tWidth: 12, headDepth: 2, boreDiameter: 4.2},
		];

	/** MISUMI HFS5 profile catalog, with nominal cross-section dimensions in millimetres. */
	public static function catalog():Catalog<TSlotProfileSpec> {
		if (table == null)
			table = new Catalog("T-slot profile", spec -> spec.designation, rows(), _ -> ({
				source: "https://sg.misumi-ec.com/pdf/fa/2013/p1839.pdf", standard: null, standardEdition: null,
				dimensionKind: Nominal, conformance: NominalEnvelope,
				verifiedFields: ["width", "height", "slotWidth", "slotDepth", "tWidth", "boreDiameter"]}));
		return table;
	}

	/** Construct a catalog-backed profile by designation, for example `HFS5-2040`. */
	public static function forProfile(designation:String):TSlotExtrusion
		return new TSlotExtrusion(catalog().get(designation).width, catalog().get(designation));

	public function profileDesignation():String
		return designation;

	public function profileDescription():String
		return description;

	public function geometry(length:Float):Part {
		if (!(length > 0)) throw "T-slot extrusion needs a positive length";
		var half = size / 2, halfHeight = height / 2;
		var body = Solids.prism([
			new Vector(-half, -halfHeight), new Vector(half, -halfHeight), new Vector(half, halfHeight), new Vector(-half, halfHeight),
		], 0, length);
		var tools = [Solids.cylinder(boreDiameter / 2, -0.1, length + 0.1)];
		for (face in 0...4) tools.push(Solids.prism(slotPoints(face), -0.1, length + 0.1));
		return Solids.cut(body, tools);
	}

	/** T-slot notch tool for one face, rotated 90 degrees `face` times from the +X face. */
	function slotPoints(face:Int):Array<Vector> {
		var half = size / 2, halfHeight = height / 2;
		var faceHalf = face % 2 == 0 ? halfHeight : half;
		var outer = (face % 2 == 0 ? half : halfHeight) + 1;
		// The catalog records the nominal 6 mm slot depth. The preview keeps a small corner web
		// so the four simplified straight slot tools do not split the profile into loose solids.
		var effectiveHeadDepth = headDepth;
		var effectiveThroatDepth = throatDepth;
		if (family != null) {
			var maxDepth = faceHalf - tWidth / 2 - 0.5;
			var totalDepth = Math.min(slotDepth, maxDepth);
			effectiveThroatDepth = Math.max(0.1, totalDepth - effectiveHeadDepth);
		}
		var points = [
			new Vector(outer, -slotWidth / 2), new Vector(outer - 1 - effectiveThroatDepth, -slotWidth / 2),
			new Vector(outer - 1 - effectiveThroatDepth, -tWidth / 2), new Vector(outer - 1 - effectiveThroatDepth - effectiveHeadDepth, -tWidth / 2),
			new Vector(outer - 1 - effectiveThroatDepth - effectiveHeadDepth, tWidth / 2), new Vector(outer - 1 - effectiveThroatDepth, tWidth / 2),
			new Vector(outer - 1 - effectiveThroatDepth, slotWidth / 2), new Vector(outer, slotWidth / 2),
		];
		if (tWidth > 2 * faceHalf) throw 'T-slot profile slot is wider than face ${Dimension.format(faceHalf * 2)}';
		return [for (p in points) rotate90(p, face)];
	}

	static function rotate90(p:Vector, times:Int):Vector {
		var x = p.x, y = p.y;
		for (i in 0...times) {
			var nx = -y, ny = x;
			x = nx;
			y = ny;
		}
		return new Vector(x, y);
	}
}
