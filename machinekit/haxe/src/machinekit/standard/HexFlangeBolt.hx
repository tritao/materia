package machinekit.standard;

import cadkit.modeling.Part;
import cadkit.modeling.Polygon;
import cadkit.threads.ThreadSpec;
import cadkit.threads.Threads;
import machinekit.catalog.Catalog;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.PropertyClass.PropertyClasses;

/** ISO 4162 Type F, non-serrated, full-form (non-indented) hex flange bolt. The nominal CAD uses a
 * continuous flange and external hex, with no washer bore, socket or Type U
 * undercut. Small edge radii/chamfers are omitted, as on the other standard
 * fastener models. The minimum flat bearing annulus remains a separate fact.
 * Frame: bearing plane z=0, head +Z, shank -Z, matching ScrewFastener.
 */
class HexFlangeBolt extends MachineComponent implements ScrewFastener {
	/** ISO 4162 Figure 1 note 2: incomplete thread u < 2P. Exclude that
	 * upper runout from the full-thread interval used by the hole policy. */
	public static inline var THREAD_RUNOUT_PITCHES=2.0;
	public static inline var DIMENSION_SOURCE="https://www.victoryfasteners.com/hexagon-head/hexagon-head-flange-bolt-iso-4162/";
	/** BIS IS 15582:2005, identical adoption of ISO 4162:1990, Table 1:
	 * c_min, da(Type F)_max, dw_min, b and the M14/M16 rows. The manufacturer's
	 * current ISO 4162 table confirms the M5..M12 head/flange dimensions. */
	public static inline var STANDARD_SOURCE="https://law.resource.org/pub/in/bis/S01/is.15582.2005.pdf";
	static var table:Null<Catalog<HexFlangeBoltSpec>>;
	static var recipe:Null<ComponentType>;
	public final spec:HexFlangeBoltSpec;
	public final length:Float;
	public final propertyClass:PropertyClass;
	public final thread:ThreadSpec;

	public static function catalog():Catalog<HexFlangeBoltSpec> {
		if (table==null) table=new Catalog("ISO 4162 flange bolt",row -> row.size,[
			row("M5",5,0.8,7,5.6,11.4,1,5.7,9.4,16),
			row("M6",6,1,8,6.8,13.6,1.1,6.8,11.6,18),
			row("M8",8,1.25,10,8.5,17,1.2,9.2,14.9,22),
			row("M10",10,1.5,13,9.7,20.8,1.5,11.2,18.7,26),
			row("M12",12,1.75,15,11.9,24.7,1.8,13.7,22.5,30),
			row("M14",14,2,18,12.9,28.6,2.1,15.7,26.4,34),
			row("M16",16,2,21,15.1,32.8,2.4,17.7,30.6,38)
		],s -> ({source:DIMENSION_SOURCE,sources:[STANDARD_SOURCE],standard:"ISO 4162",standardEdition:"2012",
			dimensionKind:machinekit.catalog.CatalogMetadata.DimensionKind.Nominal,
			conformance:machinekit.catalog.CatalogMetadata.Conformance.NominalEnvelope,
			verifiedFields:["diameter","pitch","acrossFlats","headHeight","flangeDiameter","flangeThickness",
				"bearingInnerDiameter","bearingOuterDiameter","threadLength"]}));
		return table;
	}
	static function row(size:String,d:Float,p:Float,s:Float,k:Float,dc:Float,c:Float,da:Float,dw:Float,b:Float):HexFlangeBoltSpec
		return {size:size,diameter:d,pitch:p,acrossFlats:s,headHeight:k,flangeDiameter:dc,flangeThickness:c,
			bearingInnerDiameter:da,bearingOuterDiameter:dw,threadLength:b};

	public static function metric(size:String,length:Float,propertyClass:PropertyClass=Steel88):HexFlangeBolt
		return new HexFlangeBolt(catalog().get(size),length,propertyClass);

	function new(spec:HexFlangeBoltSpec,length:Float,propertyClass:PropertyClass) {
		if (!Math.isFinite(length) || length<=THREAD_RUNOUT_PITCHES*spec.pitch || length>125)
			throw "ISO 4162 component currently requires a positive length up to 125 mm";
		if (propertyClass!=Steel88 && propertyClass!=Steel109)
			throw "ISO 4162 component requires property class 8.8 or 10.9";
		super("ISO4162-F-full-head-"+spec.size+"x"+Dimension.format(length)+"-"+PropertyClasses.token(propertyClass),
			"Full-form hex flange bolt "+spec.size+"x"+Dimension.format(length),PropertyClasses.material(propertyClass),false);
		this.spec=spec;this.length=length;this.propertyClass=propertyClass;
		thread=FastenerThreads.metric(spec.diameter,spec.pitch);
		addConnector("head",Face,Solids.axial(0,0,0));
		addConnector("tip",Face,Solids.axial(0,0,-length));
		addFacet(new machinekit.component.CollisionHullFacet([
			machinekit.component.CollisionHullFacet.cylinder(spec.diameter/2,-length,0),
			machinekit.component.CollisionHullFacet.cylinder(spec.flangeDiameter/2,0,spec.flangeThickness),
			machinekit.component.CollisionHullFacet.prism(Polygon.regular(6,spec.acrossFlats),spec.flangeThickness,spec.headHeight)
		]));
	}
	override public function hasGeometry():Bool return true;
	override public function geometryFidelity():machinekit.component.GeometryFidelity return machinekit.component.GeometryFidelity.Datasheet;
	override public function geometry(detail:ComponentDetail=Preview):Part {
		var shank=Solids.named(Part.cylinderSpan(spec.diameter/2,-length,0),"shank");
		var threaded=new Part(Threads.attach(shank.shape,"shank:cyl.side",thread,
			cadkit.threads.ThreadedFace.ThreadStart.AxisMaximum,threadedLength(),length-threadedLength()));
		shank.close();
		return Solids.union([threaded,Part.cylinderSpan(spec.flangeDiameter/2,0,spec.flangeThickness),
			Part.prism(Polygon.regular(6,spec.acrossFlats),spec.flangeThickness,spec.headHeight)]);
	}
	public function bearingFrame():materia.assembly.AssemblyRecord.AssemblyFrame
		return {x:0,y:0,z:0,qx:1,qy:0,qz:0,qw:0};
	public function headClearanceRadius():Float return spec.flangeDiameter/2;
	public function headBearingRadius():Float return spec.bearingOuterDiameter/2;
	public function screwDrive():machinekit.standard.ScrewDrive return HexHead(spec.acrossFlats);
	public function threadSpec():ThreadSpec return thread;
	public function fastenerClass():PropertyClass return propertyClass;
	public function shankLength():Float return length;
	public function threadedLength():Float return Math.min(length,spec.threadLength)-THREAD_RUNOUT_PITCHES*spec.pitch;
	public function tightening():FastenerTightening.TighteningTableRow return FastenerTightening.forThread(thread,propertyClass);
	public function driveFrame():materia.assembly.AssemblyRecord.AssemblyFrame
		return {x:0,y:0,z:spec.headHeight,qx:1,qy:0,qz:0,qw:0};
	override public function setMaterial(material:String):Void {
		if (material!=PropertyClasses.material(propertyClass)) throw "Flange bolt material follows its property class";
		super.setMaterial(material);
	}
	public static function recipeType():ComponentType {
		if (recipe==null) recipe=new ComponentType("machinekit.standard.hex-flange-bolt",
			[P.catalog("size",catalog(),"M6"),P.length("length",20),P.choice("propertyClass",["8.8","10.9"],"8.8")],
			v -> metric(v.token("size"),v.number("length"),PropertyClasses.parse(v.token("propertyClass"))),true,true,null,false);
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues
		return new ComponentValues().setToken("size",spec.size).setNumber("length",length)
			.setToken("propertyClass",PropertyClasses.token(propertyClass));
}
