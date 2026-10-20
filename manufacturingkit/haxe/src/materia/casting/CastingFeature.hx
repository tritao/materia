package materia.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;

enum abstract CastingFeatureKind(String) to String {
	var Lightening = "lightening";
	var ServiceOpening = "service-opening";
	var InterfaceRecess = "interface-recess";
	var InsertRecess = "insert-recess";
	var Clearance = "clearance";
}

enum abstract CastingGeometryKind(String) to String {
	var Box = "box";
}

enum abstract CastingOpeningKind(String) to String {
	var BodyFace = "body-face";
	var Feature = "feature";
}

/** Body-face targets are <body ID>.<x-min|x-max|y-min|y-max|z-min|z-max>. */
typedef CastingOpening = {var kind:CastingOpeningKind; var target:String;}

typedef CastingGeometry = {
	var kind:CastingGeometryKind;
	var width:Float;
	var depth:Float;
	var height:Float;
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
	var adjustable:Array<String>;
	var derivedFrom:Null<String>;
}
