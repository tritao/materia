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
	/** Optional explicit slot offsets on +X,+Y,-X,-Y; positive follows the rotated face tangent. */
	@:optional var slots:Array<Array<Float>>;
	@:optional var bores:Array<{x:Float,y:Float}>;
	@:optional var systemId:String;
	@:optional var gridPitch:Float;
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
	/** Shared dimensional system; does not assert cross-vendor hardware qualification. */
	public final systemId:Null<String>;
	public final sectionVariantId:Null<String>;
	final sectionVariant:Null<StructuralProfile>;
	final faceSlots:Array<Array<Float>>;
	final borePositions:Array<{x:Float,y:Float}>;
	final cellPitch:Null<Float>;

	public function new(size:Float, ?profile:TSlotProfileSpec, ?sectionVariant:StructuralProfile) {
		if (!(size > 0) || !Math.isFinite(size)) throw "T-slot extrusion needs a positive size";
		this.sectionVariant=sectionVariant;
		sectionVariantId=sectionVariant==null ? null : sectionVariant.profileDesignation();
		systemId=profile==null ? null : profile.systemId;
		cellPitch=profile==null ? null : profile.gridPitch;
		faceSlots=profile!=null && profile.slots!=null ? [for(slots in profile.slots) slots.copy()] : [[0],[0],[0],[0]];
		borePositions=profile!=null && profile.bores!=null ? [for(p in profile.bores) {x:p.x,y:p.y}] : [{x:0,y:0}];
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
			designation = profile.designation + (sectionVariantId==null ? "" : "-SECTION-"+sectionVariantId);
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
			designation = 'GENERIC-TSLOT-${sizeText}x$sizeText' + (sectionVariantId==null ? "" : "-SECTION-"+sectionVariantId);
			description = 'Generic T-slot extrusion ${sizeText}x$sizeText';
		}
		if(faceSlots.length!=4) throw "T-slot layout requires four face lists";
		for(face in 0...4) {
			var span=face%2==0 ? height : this.size;
			for(offset in faceSlots[face]) if(!Math.isFinite(offset) || Math.abs(offset)+tWidth/2>=span/2)
				throw "Slot does not fit on the profile face";
		}
		for(p in borePositions) if(!Math.isFinite(p.x) || !Math.isFinite(p.y) ||
			Math.abs(p.x)+boreDiameter/2>=this.size/2 || Math.abs(p.y)+boreDiameter/2>=height/2)
			throw "Bore does not fit inside the profile";
		if(sectionVariant!=null) {
			var bounds=sectionVariant.sectionBounds();
			if(Math.abs(bounds.minX+this.size/2)>0.02 || Math.abs(bounds.maxX-this.size/2)>0.02 ||
				Math.abs(bounds.minY+height/2)>0.02 || Math.abs(bounds.maxY-height/2)>0.02)
				throw "Section variant bounds differ from the profile family";
		}
	}

	/** Supplier-neutral slot-8 layout candidates. Preview slot/bore dimensions are approximate;
	 * use a measured section variant when internal shape matters. Existing catalog rows are unchanged.
	 */
	public static function forSystem(system:String,width:Float,height:Float,?section:StructuralProfile):TSlotExtrusion {
		var pitch= switch system {
			case "TSLOT-30-SLOT8": 30.0;
			case "TSLOT-40-SLOT8": 40.0;
			default: throw 'Unknown T-slot system "$system"';
		};
		var xs=centres(width,pitch),ys=centres(height,pitch);
		var bores=[for(x in xs) for(y in ys) {x:x,y:y}];
		return new TSlotExtrusion(width,{
			designation:'$system-${Dimension.format(width)}x${Dimension.format(height)}',family:system,
			width:width,height:height,slotWidth:8,slotDepth:pitch==30 ? 8 : 10,
			throatDepth:2,headDepth:pitch==30 ? 6 : 8,tWidth:pitch==30 ? 16.5 : 20,
			boreDiameter:pitch==30 ? 6.8 : 8.5,systemId:system,gridPitch:pitch,
			slots:[ys,xs,ys,xs],bores:bores
		},section);
	}

	static function centres(span:Float,pitch:Float):Array<Float> {
		var count=Math.round(span/pitch);
		if(!Math.isFinite(span) || span<=0 || count<1 || count>128 || Math.abs(span-count*pitch)>1e-8)
			throw "Profile dimension must be a positive multiple of the system pitch";
		return [for(i in 0...count) -span/2+pitch/2+i*pitch];
	}

	public function slotPositions(face:Int):Array<Float> {
		if(face<0 || face>3) throw "Slot face must be between zero and three";
		return faceSlots[face].copy();
	}
	public function boreCentres():Array<{x:Float,y:Float}> return [for(p in borePositions) {x:p.x,y:p.y}];
	/** Portable detailed sections use a self-contained geometry recipe, never a local supplier path. */
	public function drawingSection():Null<DrawingExtrusionProfile>
		return Std.isOfType(sectionVariant,DrawingExtrusionProfile) ? cast sectionVariant : null;

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

	public function sectionBounds():SectionBounds
		return {minX: -size / 2, maxX: size / 2, minY: -height / 2, maxY: height / 2};

	public function geometry(length:Float):Part {
		if (!(length > 0) || !Math.isFinite(length)) throw "T-slot extrusion needs a positive length";
		if(sectionVariant!=null) return sectionVariant.geometry(length);
		return nominalGeometry(length);
	}

	/** Simplified shared layout; source section variants retain their own cavities and curved details. */
	public function nominalGeometry(length:Float):Part {
		if (!(length > 0) || !Math.isFinite(length)) throw "T-slot extrusion needs a positive length";
		var ownedParts:Array<Part> = [];
		return Solids.building(ownedParts, tracked -> {
			var half = size / 2, halfHeight = height / 2;
			var body = Part.prism([
				new Vector(-half, -halfHeight), new Vector(half, -halfHeight), new Vector(half, halfHeight), new Vector(-half, halfHeight),
			], 0, length);
			tracked.push(body);
			var tools:Array<Part> = [];
			for(p in borePositions) {
				var bore = Part.cylinderAlong(boreDiameter/2,new Vector(p.x,p.y,-0.1),Vector.Z(),length+0.2);
				tracked.push(bore);
				tools.push(bore);
			}
			for (face in 0...4) for(offset in faceSlots[face]) {
				var tool = Part.prism(slotPoints(face,offset), -0.1, length + 0.1);
				tracked.push(tool);
				tools.push(tool);
			}
			var result = Solids.cut(body, tools);
			tracked.push(result);
			return result;
		});
	}

	/** Depth actually removed by geometry(), including its corner-web allowance. Hardware fit
	 * must use this value rather than the nominal catalog slotDepth. Faces rotate from +X. */
	public function modeledSlotDepth(face:Int = 0):Float {
		if (face < 0 || face > 3) throw "Slot face must be between zero and three";
		if(sectionVariant!=null) throw "Drawing section slot depth requires measured interface data";
		return nominalSlotDepth(face);
	}

	function nominalSlotDepth(face:Int):Float {
		if (family == null) return slotDepth;
		var faceHalf = cellPitch!=null ? cellPitch/2 : (face % 2 == 0 ? height : size) / 2;
		var total = Math.min(slotDepth,faceHalf-tWidth/2-0.5);
		return Math.max(0.1,total-headDepth)+headDepth;
	}

	/** T-slot notch tool for one face, rotated 90 degrees `face` times from the +X face. */
	function slotPoints(face:Int,offset:Float=0):Array<Vector> {
		var half = size / 2, halfHeight = height / 2;
		var faceHalf = face % 2 == 0 ? halfHeight : half;
		var outer = (face % 2 == 0 ? half : halfHeight) + 1;
		// The catalog records the nominal 6 mm slot depth. The preview keeps a small corner web
		// so the four simplified straight slot tools do not split the profile into loose solids.
		var effectiveHeadDepth = headDepth;
		var effectiveThroatDepth = throatDepth;
		if (family != null) {
			var totalDepth = nominalSlotDepth(face);
			effectiveThroatDepth = Math.max(0.1, totalDepth - effectiveHeadDepth);
		}
		var points = [
			new Vector(outer, -slotWidth / 2), new Vector(outer - 1 - effectiveThroatDepth, -slotWidth / 2),
			new Vector(outer - 1 - effectiveThroatDepth, -tWidth / 2), new Vector(outer - 1 - effectiveThroatDepth - effectiveHeadDepth, -tWidth / 2),
			new Vector(outer - 1 - effectiveThroatDepth - effectiveHeadDepth, tWidth / 2), new Vector(outer - 1 - effectiveThroatDepth, tWidth / 2),
			new Vector(outer - 1 - effectiveThroatDepth, slotWidth / 2), new Vector(outer, slotWidth / 2),
		];
		if (tWidth > 2 * faceHalf) throw 'T-slot profile slot is wider than face ${Dimension.format(faceHalf * 2)}';
		return [for (p in points) rotate90(new Vector(p.x,p.y+offset), face)];
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
