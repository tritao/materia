import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingSolid;
import materia.casting.InsertSpec.InsertBody;
import materia.casting.InsertManufacturing;
import materia.casting.InsertManufacturing.InsertManufacturingCodec as Codec;
import materia.casting.InsertBlankFit;
import materia.casting.InsertBlankFit.InsertBlankFitResult;
import materia.tooling.casting.InsertManufacturingPlan;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;

class InsertManufacturingTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {assertions++;if (!ok) throw message;}
	static function rejects(fn:Void->Void,expected:String):Void {
		var error="";try fn() catch (e:Dynamic) error=Std.string(e);
		check(error.indexOf(expected)>=0,'Expected "$expected", got "$error"');
	}
	public static function run():Void {
		var material=materia.casting.InsertMaterial.InsertMaterials.specification(materia.casting.InsertMaterial.C45Bright);
		var draft=Codec.draft(material),body=Primitive(Box(40,30,20));
		var encoded=haxe.Json.stringify(Codec.record(draft));
		check(haxe.Json.stringify(Codec.record(Codec.snapshot(haxe.Json.parse(encoded))))==encoded,"manufacturing choices round trip");
		check(draft.holeRoutes.length==0 && draft.inspections.length==0 && draft.processReference==null,
			"a draft invents neither a route nor an inspection limit or process evidence");
		rejects(() -> materia.casting.InsertOrigin.InsertOriginCodec.snapshot({kind:"fabricated"},body,[]),"Unexpected casting record fields");
		check(InsertBlankFit.assess(body,draft.blank)==InsertBlankFitResult.Unselected,"unselected stock remains explicit");
		draft.blank=Stock("synthetic-flat-bar",Box(40,30,20),F.identity());
		check(InsertBlankFit.assess(body,draft.blank)==Fits,"exact nominal flat stock contains insert");
		check(switch InsertBlankFit.assess(body,Stock("small",Box(39,30,20),F.identity())) {case Invalid(_):true;default:false;},"undersize stock rejected");
		var rotated=F.fromRotationMatrix(0,0,0,[0,-1,0,1,0,0,0,0,1]);
		check(InsertBlankFit.assess(body,Stock("rotated",Box(30,40,20),rotated))==Fits,"part-to-stock orientation is used");
		check(switch InsertBlankFit.assess(body,Stock("offset",Box(40,30,20),F.translation(1,0,0))) {case Invalid(_):true;default:false;},"offset stock cannot use only nominal dimensions");
		check(InsertBlankFit.assess(Primitive(Cylinder(8,20)),Stock("round",Cylinder(8,20),F.identity()))==Fits,"round stock exact fit");
		check(InsertBlankFit.assess(Primitive(Cylinder(8,20)),Stock("square",Box(8,8,20),F.identity()))==Fits,"analytic cylinder supports fit square stock");
		check(InsertBlankFit.assess(Primitive(Box(6,8,20)),Stock("round",Cylinder(10,20),F.identity()))==Fits,"convex vertex check fits a rectangular part in round stock");
		check(switch InsertBlankFit.assess(Primitive(Box(8,8,20)),Stock("round",Cylinder(10,20),F.identity())) {case Invalid(_):true;default:false;},
			"a square with matching bounds does not fit inside smaller round stock");
		var tilted=F.fromRotationMatrix(0,0,0,[0,0,1,0,1,0,-1,0,0]);
		check(switch InsertBlankFit.assess(Primitive(Cylinder(8,20)),Stock("oblique",Cylinder(30,30),tilted)) {case Unsupported(_):true;default:false;},
			"oblique curved containment is explicit, never a box approximation");
		check(switch InsertBlankFit.assess(Primitive(Cylinder(8,1000)),Stock("small-angle",Cylinder(8,1000),F.turnY(1e-9))) {
			case Unsupported(_):true;default:false;
		},"parallel classification accounts for accumulated angular error over the insert length");
		draft.inspections=[BlankDimensions(0)];
		rejects(() -> Codec.record(draft),"tolerance must be positive");
		draft.inspections=[BlankDimensions(0.1),BlankDimensions(0.2)];
		rejects(() -> Codec.record(draft),"Duplicate insert manufacturing inspection");
		draft.inspections=[MaterialIdentity,BlankDimensions(0.1),BondSurface];
		var copy=Codec.snapshot(Codec.record(draft));copy.inspections.pop();
		check(draft.inspections.length==3,"inspection choices are copied defensively");
		draft.processReference="synthetic-shop-process";
		var holes=TestInsertHoles.through(5,20);
		var insert:materia.tooling.casting.CastingToolingInput.CastingToolingInsert={id:"synthetic-seat",body:body,holes:holes,
			origin:materia.casting.InsertOrigin.Fabricated(draft),interfaces:[],material:material,pose:F.identity(),
			role:materia.casting.InsertSpec.InsertRole.GenericMount,label:"synthetic inspection fixture",positionTolerance:0.01,
			finish:materia.casting.InsertSpec.InsertFinish.AsSupplied};
		var pending=InsertManufacturingPlan.build(insert);
		check(pending.operations.filter(op -> op.kind==Drill || op.kind==TransferDrill).length==0,
			"unassigned source hole generates no guessed drilling operation");
		check(pending.validation.filter(v -> v.status==Pending && v.message.indexOf("hole route fixing")>=0).length==1,
			"missing hole route blocks manufacturing readiness with source hole ID");
		draft.holeRoutes=[{holeId:"missing",route:InsertHoleRoute.BeforeCasting}];
		rejects(() -> Codec.references(draft,holes,[]),"unknown insert hole");
		draft.holeRoutes=[{holeId:"fixing",route:InsertHoleRoute.BeforeCasting}];
		draft.inspections.push(HolePosition("fixing",0.05));
		var before=InsertManufacturingPlan.build(insert);
		check(before.operations.filter(op -> op.kind==Drill && op.stage==InsertOperationStage.BeforeCasting).length==1,
			"declared pre-cast source hole receives one drilling operation");
		check(before.validation.filter(v -> v.rule=="fabrication.insert.plan.synthetic-seat" && v.status==Pass).length==1,
			"declared process and inspection limits clear the plan check");
		check(before.validation.filter(v -> v.rule=="fabrication.insert.inspection.synthetic-seat" && v.status==Pending).length==1,
			"an inspection plan never becomes a physical inspection result");
		draft.holeRoutes[0].route=InsertHoleRoute.TransferAfterCasting;
		var after=InsertManufacturingPlan.build(insert);
		check(materia.casting.InsertStages.holes(
			{body:insert.body,holes:insert.holes,origin:insert.origin,interfaces:insert.interfaces},materia.casting.InsertStages.InsertStage.BeforeCasting).length==0
			&& materia.casting.InsertStages.holes(
			{body:insert.body,holes:insert.holes,origin:insert.origin,interfaces:insert.interfaces},materia.casting.InsertStages.InsertStage.Service).length==1,
			"one source pattern produces the correct pre-cast and service hole sets");
		check(after.operations.filter(op -> op.kind==TransferDrill && op.stage==InsertOperationStage.AfterCasting).length==1,
			"transfer route moves the hole operation after casting");
		check(after.validation.filter(v -> v.rule=="fabrication.insert.hole-geometry.synthetic-seat" && v.status==Pass).length==1,
			"resolved source routes produce distinct pre-cast and service hole sets");
		check(after.inspectionRequirements.filter(row -> row.kind==InsertInspectionKind.HolePosition
			&& row.stage==InsertInspectionStage.AfterCasting && row.declared && row.toleranceMm==0.05).length==1,
			"inspection schedule follows the declared hole route and tolerance");
		insert.interfaces=[{id:"ground-face",geometry:materia.casting.InterfaceRequirement.PrecisionGeometry.Plane(F.translation(0,0,19)),
			route:materia.casting.InterfaceRequirement.InterfaceRoute.Ground,allowanceMm:1,finishingProcessErrorMm:null,
			permittedFinishes:[materia.casting.InterfaceRequirement.InterfaceFinish.Ground],positionToleranceMm:0.01,
			datumReference:"primary",relationshipIds:[]}];
		holes[0].frame.z=19;holes[0].depth=19;
		var finishing=InsertManufacturingPlan.build(insert),grind=-1,transfer=-1;
		for (index in 0...finishing.operations.length) {
			if (finishing.operations[index].kind==GrindInterface) grind=index;
			if (finishing.operations[index].kind==TransferDrill) transfer=index;
		}
		check(grind>=0 && transfer>grind,"finishing precedes transfer drilling through an aligned bought part");
		check(finishing.validation.filter(v -> v.rule=="fabrication.insert.finish-geometry.synthetic-seat"
			&& v.status==Pass).length==1,"finished steel geometry and service drilling share the source stock realization");
		check(finishing.operations[finishing.operations.length-1].kind==Inspect
			&& finishing.operations[finishing.operations.length-1].stage==InsertOperationStage.AfterCasting,
			"post-cast work ends with inspection");
		check(finishing.inspectionRequirements.filter(row -> row.kind==InsertInspectionKind.FaceFlatness
			&& !row.declared && row.toleranceMm==null).length==1,
			"missing pre-cast face-flatness specification is visible without a placeholder limit");
		draft.blank=Stock("finished-size-only",Box(40,30,19),F.identity());
		check(InsertManufacturingPlan.build(insert).validation.filter(v -> v.rule=="fabrication.insert.blank.synthetic-seat"
			&& v.status==Fail).length==1,"stock must contain the raw insert's finishing layer, not only the finished dimensions");
		draft.blank=Stock("synthetic-flat-bar",Box(40,30,20),F.identity());
		draft.inspections.push(ThreadGauge("fixing"));
		rejects(() -> Codec.references(draft,holes,[]),"source threaded hole");
		Sys.println('InsertManufacturingTests: $assertions assertions passed');
	}
}
