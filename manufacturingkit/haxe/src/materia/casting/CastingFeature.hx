package materia.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;

enum abstract CastingFeatureKind(String) to String {
	var Lightening = "lightening";
	var ServiceOpening = "service-opening";
	var InterfaceRecess = "interface-recess";
	var InsertRecess = "insert-recess";
	var Clearance = "clearance";
}

class CastingFeatureKinds {
	public static function values():Array<CastingFeatureKind>
		return [CastingFeatureKind.Lightening, CastingFeatureKind.ServiceOpening, CastingFeatureKind.InterfaceRecess,
			CastingFeatureKind.InsertRecess, CastingFeatureKind.Clearance];
	public static function tokens():Array<String> return [for (kind in values()) (kind:String)];
}

enum abstract CastingDimension(String) to String {
	var Width = "width";
	var Depth = "depth";
	var Height = "height";
	var Draft = "draft";
	var CornerRadius = "cornerRadius";
}

class CastingDimensions {
	public static function values():Array<CastingDimension> return [CastingDimension.Width, CastingDimension.Depth, CastingDimension.Height, CastingDimension.Draft, CastingDimension.CornerRadius];
	public static function tokens():Array<String> return [for (dimension in values()) (dimension:String)];
}

enum abstract CastingOpeningKind(String) to String {
	var BodyFace = "body-face";
	var Feature = "feature";
}

/** Body-face targets are <body ID>.<x-min|x-max|y-min|y-max|z-min|z-max>. */
typedef CastingOpening = {var kind:CastingOpeningKind; var target:String;}

/** Insert recesses preserve their complete source exterior; design cuts stay primitive. */
enum CastingGeometrySolid {
	Cut(solid:CastingSolid);
	Insert(body:InsertSpec.InsertBody);
}

class CastingGeometrySolids {
	public static function snapshot(raw:Dynamic):CastingGeometrySolid {
		if (Std.isOfType(raw, CastingGeometrySolid)) raw = record(cast raw);
		return switch CastingRecordCodec.string(raw, "kind") {
			case "cut":
				CastingFeatureCodec.fields(raw, ["kind", "solid"]);
				Cut(CastingSolid.CastingSolidCodec.snapshot(Reflect.field(raw, "solid")));
			case "insert":
				CastingFeatureCodec.fields(raw, ["kind", "body"]);
				Insert(InsertSpec.InsertBodyCodec.snapshot(Reflect.field(raw, "body")));
			default: throw "Unknown casting geometry kind";
		};
	}
	public static function record(value:CastingGeometrySolid):Dynamic return switch value {
		case Cut(solid): {kind: "cut", solid: CastingSolid.CastingSolidCodec.record(solid)};
		case Insert(body): {kind: "insert", body: InsertSpec.InsertBodyCodec.record(body)};
	};
	public static function isBox(value:CastingGeometrySolid):Bool return switch value {
		case Cut(solid): switch solid {case Box(_, _, _): true; default: false;};
		case Insert(body): switch body {case Primitive(solid): switch solid {case Box(_, _, _): true; default: false;}; default: false;};
	};
	public static function box(value:CastingGeometrySolid, id:String):{width:Float, depth:Float, height:Float} return switch value {
		case Cut(solid): CastingSolid.CastingSolidCodec.box(solid, id);
		case Insert(body): InsertSpec.InsertBodyCodec.box(body, id);
	};
	public static function height(value:CastingGeometrySolid):Float return switch value {
		case Cut(solid): CastingSolid.CastingSolidCodec.height(solid);
		case Insert(body): InsertSpec.InsertBodyCodec.height(body);
	};
	public static function mouth(value:CastingGeometrySolid):CastingSolid return switch value {
		case Cut(solid): solid;
		case Insert(body): InsertSpec.InsertBodyCodec.mouth(body);
	};
}

typedef CastingGeometry = {
	var solid:CastingGeometrySolid;
	var pose:AssemblyFrame;
}

typedef CastingDraft = {var degrees:Float; var axis:{x:Float, y:Float, z:Float};}

/** Design cut plus declared openings; derive the finished cavity through CastingCavities. */
typedef CastingFeature = {
	var id:String;
	var kind:CastingFeatureKind;
	var geometry:CastingGeometry;
	var openings:Array<CastingOpening>;
	var draft:CastingDraft;
	var cornerRadius:Float;
	var owner:Null<String>;
	var adjustable:Array<CastingDimension>;
	var derivedFrom:Null<String>;
}
