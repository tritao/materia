import materia.casting.FoundationDatumScheme;
import materia.casting.FoundationDatumScheme.FoundationDatums;
import materia.casting.FoundationDatumScheme.FoundationDatumSources;
import materia.casting.InterfaceRequirement;
import materia.assembly.AssemblyFrames as F;

class FoundationDatumTests {
	static var assertions = 0;
	static function check(ok:Bool, message:String):Void {
		assertions++;
		if (!ok) throw message;
	}
	static function rejects(fn:Void -> Void, expected:String):Void {
		var message = "";
		try fn() catch (error:Dynamic) message = Std.string(error);
		check(message.indexOf(expected) >= 0,'Expected "$expected", got "$message"');
	}
	static function scheme():FoundationDatumScheme return {
		id: "base-datums", foundationId: "base",
		primary: {id: "z-primary", reference: BodyFace("body.z-min"), frame: F.identity()},
		secondary: {id: "x-secondary", reference: FeatureFace("register","side-0"), frame: F.identity()},
		tertiary: {id: "a-tertiary", reference: DatumReference.InsertInterface("seat","edge"), frame: F.identity()}
	};
	public static function run():Void {
		var s=scheme(),saved=haxe.Json.stringify(FoundationDatums.record(s));
		check(haxe.Json.stringify(FoundationDatums.record(FoundationDatums.snapshot(haxe.Json.parse(saved))))==saved,
			"datum scheme round trip");
		check(FoundationDatums.ordered(s)[0].id=="z-primary" && FoundationDatums.ordered(s)[2].id=="a-tertiary",
			"datum precedence remains semantic rather than alphabetical");
		var copy=FoundationDatums.snapshot(s);copy.primary.frame.x=20;
		check(s.primary.frame.x==0,"datum frame is a defensive copy");
		var sources:FoundationDatumSources={bodyFaces:["body.z-min"],featureFaces:["register"=>["side-0"]],
			insertInterfaces:["seat"=>["edge"]],insertHoles:["seat"=>["pin"]],unsupportedFeatures:[]};
		var requirement:InterfaceRequirement={id:"seat-edge",geometry:Line(F.identity()),route:AdjustedAtAssembly,
			allowanceMm:0,finishingProcessErrorMm:null,permittedFinishes:[PushScrews],positionToleranceMm:0.01,datumReference:"x-secondary",relationshipIds:[]};
		FoundationDatums.validateReferences(s,sources,[requirement]);
		check(true,"all source and interface datum references resolve");
		s.tertiary.reference=DatumReference.InsertHole("seat","pin");
		FoundationDatums.validateReferences(s,sources,[requirement]);
		check(FoundationDatums.snapshot(FoundationDatums.record(s)).foundationId=="base","source bore datum remains explicitly typed");
		s.tertiary.reference=DatumReference.InsertHole("seat","invented");
		rejects(() -> FoundationDatums.validateReferences(s,sources,[requirement]),"Unknown foundation datum reference: a-tertiary");
		s=scheme();requirement.datumReference="missing";
		rejects(() -> FoundationDatums.validateReferences(s,sources,[requirement]),"undeclared foundation datum: seat-edge");
		s.secondary.id=s.primary.id;
		rejects(() -> FoundationDatums.snapshot(s),"Duplicate foundation datum");
		s=scheme();s.primary.frame.qw=0;
		rejects(() -> FoundationDatums.snapshot(s),"unit rotation");
		var bad:Dynamic=haxe.Json.parse(saved);Reflect.setField(bad,"oldDatum",true);
		rejects(() -> FoundationDatums.snapshot(bad),"Unexpected casting record fields");
		var envelope = materia.casting.CastingEnvelope.build(100,80,60);
		var feature = materia.casting.CastingPatterns.box("register",materia.casting.CastingFeature.CastingFeatureKind.Lightening,
			[materia.casting.CastingPatterns.face("body.z-min")],20,20,20,0,0,0);
		s=scheme();s.secondary.reference=FeatureFace("register","register-body.x-min");
		s.primary.frame=F.fromRotationMatrix(30,0,0,[1,0,0,0,-1,0,0,0,-1]);
		s.secondary.frame=F.fromRotationMatrix(-10,0,10,[0,0,1,0,1,0,-1,0,0]);
		requirement.datumReference="x-secondary";
		var edge:InterfaceRequirement = {id:"edge",geometry:Line(F.identity()),route:AdjustedAtAssembly,
			allowanceMm:0,finishingProcessErrorMm:null,permittedFinishes:[PushScrews],positionToleranceMm:0.01,datumReference:"z-primary",relationshipIds:[]};
		var inserts:Array<materia.casting.FoundationDatumScheme.FoundationDatumInsert> = [
			{id:"seat",pose:F.identity(),holes:TestInsertHoles.through(5,20),interfaces:[edge]},
			{id:"other-seat",pose:F.identity(),holes:[],interfaces:[edge]}
		];
		check(FoundationDatums.validateSource(s,envelope,[feature],inserts,[requirement]).length==0,
			"source references use exact finished cavity faces and scoped insert interface IDs");
		s.primary.frame.z=1;
		rejects(() -> FoundationDatums.validateSource(s,envelope,[feature],inserts,[requirement]),"does not match referenced face: z-primary");
		s.primary.frame.z=0;s.primary.frame.x=51;
		rejects(() -> FoundationDatums.validateSource(s,envelope,[feature],inserts,[requirement]),"does not match referenced face: z-primary");
		s.primary.frame=F.translation(30,0,0);
		rejects(() -> FoundationDatums.validateSource(s,envelope,[feature],inserts,[requirement]),"does not match referenced face: z-primary");
		var angle=1e-5,c=Math.cos(angle),sn=Math.sin(angle);
		s.primary.frame=F.fromRotationMatrix(30,0,0,[c,0,sn,0,-1,0,sn,0,-c]);
		rejects(() -> FoundationDatums.validateSource(s,envelope,[feature],inserts,[requirement]),"does not match referenced face: z-primary");
		s.primary.frame=F.fromRotationMatrix(30,0,0,[1,0,0,0,-1,0,0,0,-1]);
		s.secondary.frame.x=-9;
		rejects(() -> FoundationDatums.validateSource(s,envelope,[feature],inserts,[requirement]),"does not match referenced face: x-secondary");
		s.secondary.frame.x=-10;
		inserts[0].pose=F.fromRotationMatrix(5,6,7,[0,-1,0,1,0,0,0,0,1]);
		s.tertiary.frame=F.compose(inserts[0].pose,F.translation(4,0,0));
		check(FoundationDatums.validateSource(s,envelope,[feature],inserts,[requirement]).length==0,
			"line datum accepts translation along the source line after composing the insert pose");
		s.tertiary.frame.x+=1;
		rejects(() -> FoundationDatums.validateSource(s,envelope,[feature],inserts,[requirement]),"does not match source interface line: a-tertiary");
		s.tertiary.reference=DatumReference.InsertHole("seat","fixing");
		s.tertiary.frame=F.compose(inserts[0].pose,inserts[0].holes[0].frame);
		check(FoundationDatums.validateSource(s,envelope,[feature],inserts,[requirement]).length==0,
			"hole datum resolves the actual entry and directed axis in foundation coordinates");
		s.tertiary.frame.z+=1;
		rejects(() -> FoundationDatums.validateSource(s,envelope,[feature],inserts,[requirement]),"does not match source hole entry: a-tertiary");
		s.tertiary.reference=DatumReference.InsertInterface("seat","edge");
		edge.geometry=Plane(F.translation(0,0,20));edge.route=InterfaceRoute.Ground;
		edge.permittedFinishes=[InterfaceFinish.Ground];
		s.tertiary.frame=F.compose(inserts[0].pose,F.translation(2,3,20));
		check(FoundationDatums.validateSource(s,envelope,[feature],inserts,[requirement]).length==0,
			"plane datum accepts an in-plane origin without inventing contact extents");
		s.tertiary.frame.z+=1;
		rejects(() -> FoundationDatums.validateSource(s,envelope,[feature],inserts,[requirement]),"does not match source interface plane: a-tertiary");
		s.tertiary.frame.z-=1;
		s.secondary.reference=FeatureFace("register","register-body.missing");
		rejects(() -> FoundationDatums.validateSource(s,envelope,[feature],inserts,[requirement]),"Unknown foundation datum reference: x-secondary");
		s.secondary.reference=FeatureFace("register","register-body.x-min");
		feature.geometry.solid=materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Cylinder(20,20));
		var unresolved=FoundationDatums.validateSource(s,envelope,[feature],inserts,[requirement]);
		check(unresolved.length==1 && unresolved[0].featureId=="register" && unresolved[0].datumId=="x-secondary",
			"round datum boundary stays explicitly unresolved without an invented box face");
		s.secondary.reference=FeatureFace("absent-feature","face");
		rejects(() -> FoundationDatums.validateSource(s,envelope,[feature],inserts,[requirement]),"Unknown foundation datum reference: x-secondary");
		Sys.println('FoundationDatumTests: $assertions assertions passed');
	}
}
