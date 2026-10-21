import materia.assembly.AssemblyFrames as F;
import materia.casting.InterfaceRequirement;
import materia.casting.InterfaceRelationship;
import materia.casting.CastingInterfaceContext;
import materia.tooling.casting.ReplicationJig;
import materia.tooling.casting.ReplicationRelationships;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingCodec;

class ReplicationRelationshipTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {assertions++;if (!ok) throw message;}
	static function requirement(id:String,x:Float):InterfaceRequirement return {
		id:id,geometry:Plane(F.translation(x,0,0)),route:InterfaceRoute.Replicated,allowanceMm:1,finishingProcessErrorMm:null,
		permittedFinishes:[InterfaceFinish.JigMaster],positionToleranceMm:0.01,datumReference:"primary",relationshipIds:["pair"]};
	static function master(id:String,x:Float):ReplicationMaster return {
		id:id,kind:ReplicationMasterKind.GroundPlate,body:materia.casting.CastingSolid.Box(40,20,4),pose:F.translation(x,0,0),reference:F.identity(),
		members:[ComponentInterface("casting",id)],finish:InterfaceFinish.JigMaster,jacks:[],releaseSpec:null,flatnessMm:null,inspectionReference:null};
	public static function run():Void {
		var a=requirement("a",0),b=requirement("b",50);
		var r:InterfaceRelationship={id:"pair",kind:Coplanar,members:[ComponentInterface("casting","a"),ComponentInterface("casting","b")],
			tolerance:Linear(0.01),metrology:{method:MetrologyMethod.IndicatorReversal,instrumentReference:null,uncertaintyMm:null}};
		var context:CastingInterfaceContext={foundationJoints:new Array<materia.casting.FoundationJointSources.FoundationJointOccurrence>(),jointSources:materia.casting.FoundationJointSources.FoundationJointSourceCodec.empty(),bodyMember:"casting",inserts:[],relationships:[{id:"instance",relationship:r}],
			sources:[Precision("casting",a,F.identity()),Precision("casting",b,F.identity())]};
		var j:ReplicationJig={id:"frame",datumSchemeId:"datums",frameReference:"synthetic-frame",pose:F.identity(),masters:[master("a",0),master("b",50)]};
		function evaluate():materia.tooling.ToolingValidation return ReplicationRelationships.check(j,"instance",r,context);
		check(evaluate().status==ToolingCheckStatus.Pass,"aligned nominal planes are coplanar");
		j.masters[1].pose.z=0.02;
		check(evaluate().status==ToolingCheckStatus.Fail,"common frame cannot hide a plane-height error");
		r.kind=Parallel;r.tolerance=PerLength(0.01,100);
		check(evaluate().status==ToolingCheckStatus.Pass,"direction-only parallelism does not constrain separation");
		j.masters[1].pose=F.compose(F.translation(50,0,0),F.turnY(0.001));
		check(evaluate().status==ToolingCheckStatus.Fail,"nominal masters must satisfy the source angular span budget");
		r.tolerance=PerLength(1,100);
		check(evaluate().status==ToolingCheckStatus.Pass,"a changed source budget changes angular acceptance");
		r.kind=Perpendicular;r.tolerance=PerLength(0.01,100);j.masters[1].pose=F.turnY(Math.PI/2);
		check(evaluate().status==ToolingCheckStatus.Pass,"orthogonal master planes satisfy perpendicularity");
		j.masters[1].pose=F.turnY(Math.PI/2-0.001);
		check(evaluate().status==ToolingCheckStatus.Fail,"squareness error is a rise over the declared span");
		r.kind=Distance(10);r.tolerance=Linear(0.01);j.masters[1].pose=F.translation(50,0,10);
		check(evaluate().status==ToolingCheckStatus.Pass,"separated parallel planes satisfy their nominal distance");
		j.masters[1].pose.z=10.02;
		check(evaluate().status==ToolingCheckStatus.Fail,"wrong plane distance fails with its source relationship ID");
		j.masters[1].pose=F.fromRotationMatrix(50,0,10,[1,0,0,0,-1,0,0,0,-1]);
		check(evaluate().status==ToolingCheckStatus.Pass,"unsigned plane separation permits reversed surface normals");
		var original=ToolingCodec.canonical(evaluate());
		r.members.reverse();j.masters.reverse();context.sources.reverse();
		check(ToolingCodec.canonical(evaluate())==original,"relationship results and affected IDs are order independent");
		var common=F.compose(F.translation(100,-30,70),F.turnY(0.4));j.pose=common;
		check(evaluate().status==ToolingCheckStatus.Pass,"pairwise nominal relation is invariant under a common rigid transform");
		j.pose=F.identity();
		a.geometry=Line(F.identity());a.route=InterfaceRoute.AdjustedAtAssembly;a.allowanceMm=0;a.permittedFinishes=[InterfaceFinish.PushScrews];
		b.geometry=Line(F.translation(0,10,0));b.route=InterfaceRoute.AdjustedAtAssembly;b.allowanceMm=0;b.permittedFinishes=[InterfaceFinish.PushScrews];
		context.sources=[Precision("casting",a,F.identity()),Precision("casting",b,F.identity())];
		j.masters=[master("a",0),master("b",0)];j.masters[1].pose=F.translation(0,10,0);
		check(evaluate().status==ToolingCheckStatus.Pass,"parallel reference edges have their exact nominal line distance");
		j.masters[1].pose=F.fromRotationMatrix(0,10,0,[-1,0,0,0,-1,0,0,0,1]);
		check(evaluate().status==ToolingCheckStatus.Pass,"antiparallel line distance does not collapse through floating-point cross products");
		j.masters[1].pose=F.fromRotationMatrix(0,10,0,[Math.cos(0.001),-Math.sin(0.001),0,Math.sin(0.001),Math.cos(0.001),0,0,0,1]);
		check(evaluate().status==ToolingCheckStatus.Unsupported,"skew edge distance does not invent a gage plane or use a distant intersection");
		r.kind=Coplanar;
		check(evaluate().status==ToolingCheckStatus.Unsupported,"reference edges are not substituted for plane interfaces");
		r.kind=Straightness;r.members=[ComponentInterface("casting","a")];context.sources=[Precision("casting",a,F.identity())];
		check(evaluate().status==ToolingCheckStatus.Unsupported,"an ideal straight line cannot certify physical straightness");
		r.kind=InPlaneParallelism;r.members=[ComponentInterface("casting","a"),ComponentInterface("casting","b")];r.tolerance=PerLength(0.01,100);
		context.sources=[Precision("casting",a,F.identity()),Precision("casting",b,F.identity())];
		check(evaluate().status==ToolingCheckStatus.Unsupported,"in-plane alignment does not silently infer a normal from a line frame");
		r.kind=Parallel;r.members=[ComponentInterface("casting","a"),MachineAxis("C")];
		context.sources=[Precision("casting",a,F.identity()),Axis("C",F.turnY(Math.PI/2),{x:0,y:0,z:1})];
		j.masters[1].members=[MachineAxis("C")];j.masters[1].kind=ReplicationMasterKind.AxisMaster;
		j.masters[1].pose=F.turnY(Math.PI/2);
		check(evaluate().status==ToolingCheckStatus.Pass,"reference edge and machine-axis master use their distinct local directions");
		j.masters[1].pose=F.identity();
		check(evaluate().status==ToolingCheckStatus.Fail,"wrong axis-master direction cannot pass a shared-frame relationship");
		a.geometry=Plane(F.identity());a.route=InterfaceRoute.Replicated;a.allowanceMm=1;a.permittedFinishes=[InterfaceFinish.JigMaster];
		context.sources=[Precision("casting",a,F.identity()),Axis("C",F.identity(),{x:0,y:0,z:1})];
		r.kind=Perpendicular;
		check(evaluate().status==ToolingCheckStatus.Pass,"a table plane is perpendicular to an axis parallel to its normal");
		r.kind=Parallel;
		check(evaluate().status==ToolingCheckStatus.Fail,"a plane normal parallel to an axis does not make the plane parallel to it");
		j.masters[1].pose=F.turnY(Math.PI/2);
		check(evaluate().status==ToolingCheckStatus.Pass,"a vertical rail plane can be parallel to an axis lying along that plane");
		Sys.println('ReplicationRelationshipTests: $assertions assertions passed');
	}
}
