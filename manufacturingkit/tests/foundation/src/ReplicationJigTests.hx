import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingSolid;
import materia.casting.InterfaceRequirement;
import materia.casting.InterfaceRelationship;
import materia.casting.InterfaceRelationship.InterfaceRelationships as R;
import materia.casting.CastingInterfaceContext;
import materia.tooling.casting.ReplicationJig;
import materia.tooling.casting.ReplicationJig.ReplicationJigCodec as Codec;
import materia.tooling.casting.ReplicationJigs;
import materia.tooling.casting.CastingToolingInput;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;

class ReplicationJigTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {assertions++;if (!ok) throw message;}
	static function rejects(fn:Void->Void,expected:String):Void {
		var message="";try fn() catch (e:Dynamic) message=Std.string(e);
		check(message.indexOf(expected)>=0,'Expected "$expected", got "$message"');
	}
	static function requirement(id:String,x:Float):InterfaceRequirement return {
		id:id,geometry:Plane(F.translation(x,0,20)),route:InterfaceRoute.Replicated,allowanceMm:1,finishingProcessErrorMm:null,
		permittedFinishes:[InterfaceFinish.JigMaster],positionToleranceMm:0.01,datumReference:"primary",relationshipIds:["rail-planes"]};
	static function input():CastingToolingInput {
		var a=requirement("a",-10),b=requirement("b",10);
		var relationship:InterfaceRelationship={id:"rail-planes",kind:Coplanar,
			members:[ComponentInterface("casting","a"),ComponentInterface("casting","b")],tolerance:Linear(0.01),
			metrology:{method:MetrologyMethod.IndicatorReversal,instrumentReference:null,uncertaintyMm:null}};
		var context:CastingInterfaceContext={bodyMember:"casting",inserts:[],relationships:[{id:"rail-planes",relationship:relationship}],
			sources:[Precision("casting",a,F.identity()),Precision("casting",b,F.identity())]};
		return {id:"jig-fixture",revision:1,lengthUnit:materia.tooling.ToolingEnums.ToolingLengthUnit.Millimetres,
			datumScheme:null,interfaceContext:CastingInterfaceContexts.snapshot(context),interfaces:[a,b],
			envelope:materia.casting.CastingEnvelope.build(100,80,40),features:[],inserts:[]};
	}
	static function jig():ReplicationJig return {
		id:"common-frame",datumSchemeId:"foundation-datums",frameReference:"synthetic-frame",pose:F.identity(),masters:[{
			id:"ground-bar",kind:ReplicationMasterKind.GroundBar,body:Box(60,20,10),pose:F.translation(0,0,20),reference:F.identity(),
			members:[ComponentInterface("casting","a"),ComponentInterface("casting","b")],finish:InterfaceFinish.JigMaster,
			jacks:[{id:"jack-a",position:{x:-20,y:-5,z:0},lockReference:"lock-a"},
				{id:"jack-b",position:{x:20,y:-5,z:0},lockReference:"lock-b"},
				{id:"jack-c",position:{x:0,y:5,z:0},lockReference:"lock-c"}],releaseSpec:null,flatnessMm:null,inspectionReference:null
		}]};
	static function status(checks:Array<materia.tooling.ToolingValidation>,suffix:String,value:ToolingCheckStatus):Bool
		return checks.filter(check -> StringTools.endsWith(check.rule,suffix) && check.status==value).length==1;
	static function placement():Void {
		var source=input(),j=jig(),context=source.interfaceContext;
		if (context==null) throw "Expected source context";
		var target=context.sources[0];
		function measure():materia.tooling.ToolingValidation
			return materia.tooling.casting.ReplicationPlacement.check(j,j.masters[0],target,context);
		check(measure().status==ToolingCheckStatus.Pass,"nominal master plane matches the source plane");
		j.pose.z=0.02;
		check(measure().status==ToolingCheckStatus.Fail,"normal translation exceeds the source position tolerance");
		j.pose.z=0;j.pose.x=100;
		check(measure().status==ToolingCheckStatus.Pass,"tangential plane placement is separate from finite contact coverage");
		j=jig();j.masters[0].pose=F.compose(F.translation(0,0,20),F.turnY(0.001));
		check(measure().status==ToolingCheckStatus.Fail,"angular error is bounded over the finite master stock");
		j.masters[0].pose=F.compose(F.translation(0,0,20),F.turnY(0.00001));
		check(measure().status==ToolingCheckStatus.Pass,"source tolerance accepts a smaller nominal angular bound");
		j=jig();j.masters[0].reference=F.translation(0,0,10);j.masters[0].pose=F.translation(0,0,10);
		check(measure().status==ToolingCheckStatus.Pass,"contact-reference placement is composed with the blank placement");
		j=jig();var common=F.compose(F.translation(100,-200,300),F.turnY(0.4));j.pose=common;
		target=Precision("external",requirement("a",-10),common);
		check(measure().status==ToolingCheckStatus.Pass,"nominal plane checks are invariant under a common rigid transform");
		j=jig();var edge=requirement("edge",0);edge.geometry=Line(F.translation(0,0,20));
		edge.route=InterfaceRoute.AdjustedAtAssembly;edge.allowanceMm=0;edge.permittedFinishes=[InterfaceFinish.PushScrews];
		target=Precision("external",edge,F.identity());
		check(measure().status==ToolingCheckStatus.Pass,"line references use their declared positive-X direction");
		j.pose.x=100;
		check(measure().status==ToolingCheckStatus.Pass,"nominal line placement permits translation along the reference edge");
		j.pose.x=0;j.pose.y=0.02;
		check(measure().status==ToolingCheckStatus.Fail,"line lateral placement is checked against its source tolerance");
		j=jig();j.masters[0].pose=F.fromRotationMatrix(0,0,20,[Math.cos(0.001),-Math.sin(0.001),0,Math.sin(0.001),Math.cos(0.001),0,0,0,1]);
		check(measure().status==ToolingCheckStatus.Fail,"reference-edge angular error is not mistaken for its plane normal");
		var r=materia.tooling.casting.ReplicationPlacement.stockRadius(Cylinder(10,80),F.translation(3,4,0));
		check(Math.abs(r-Math.sqrt(6500))<1e-10,"cylinder stock radius uses exact radial support and cap height");
		context.relationships[0].relationship.members=[ComponentInterface("casting","a"),MachineAxis("Z")];
		context.relationships[0].relationship.kind=Parallel;context.relationships[0].relationship.tolerance=PerLength(0.01,100);
		j=jig();j.masters[0].kind=ReplicationMasterKind.AxisMaster;j.masters[0].body=Cylinder(10,80);
		j.masters[0].pose=F.translation(0,0,50);target=Axis("Z",F.identity(),{x:0,y:0,z:1});
		check(measure().status==ToolingCheckStatus.Pass,"axis placement permits translation along the same line");
		j.masters[0].pose.x=0.02;
		check(measure().status==ToolingCheckStatus.Fail,"axis lateral displacement violates nominal coaxiality");
		j.masters[0].pose=F.turnY(0.001);
		check(measure().status==ToolingCheckStatus.Fail,"axis angular error uses the declared relationship span");
		context.relationships[0].relationship.tolerance=PerLength(1,100);
		check(measure().status==ToolingCheckStatus.Pass,"axis acceptance changes with the declared source tolerance");
		j.masters[0].pose=F.compose(F.translation(0,0,50),F.turnY(0.00005));
		j.masters[0].body=Cylinder(10,1000);context.relationships[0].relationship.tolerance=PerLength(0.01,100);
		check(measure().status==ToolingCheckStatus.Pass,"axis angular budget retains its declared span for longer stock");
		j.masters[0].pose=F.turnY(Math.PI/4);context.relationships[0].relationship.tolerance=PerLength(80,100);
		check(measure().status==ToolingCheckStatus.Fail,"axis angular rise uses the source reference span rather than the tilted length");
		j.masters[0].pose=F.identity();context.relationships[0].relationship.kind=Distance(0);
		context.relationships[0].relationship.tolerance=Linear(0.01);
		check(measure().status==ToolingCheckStatus.Unsupported,"a linear distance tolerance supplies no angular manufacturing budget");
		context.relationships=[];
		check(measure().status==ToolingCheckStatus.Unsupported,"axis placement invents no missing design limit");
	}

	static function contactBoundary():Void {
		var source=input(),j=jig();
		function contact(value:ToolingCheckStatus):Bool
			return status(ReplicationJigs.analyze(source,[j],[]),".support-contact",value);
		check(contact(ToolingCheckStatus.Pass),"box underside contacts lie on the actual master boundary");
		j.masters[0].jacks[0].position.x=31;
		check(contact(ToolingCheckStatus.Fail),"a coplanar point outside box stock is not a support contact");
		j=jig();j.masters[0].jacks[0].position.z=1;
		check(contact(ToolingCheckStatus.Fail),"a jack point buried in solid stock fails");
		j=jig();j.masters[0].jacks[0].position={x:30,y:0,z:5};
		check(contact(ToolingCheckStatus.Pass),"a declared side contact may locate the master without claiming a support force direction");
		j.pose=F.compose(F.translation(120,-40,200),F.turnY(0.7));j.masters[0].pose=F.turnY(-0.2);
		check(contact(ToolingCheckStatus.Pass),"contact membership is unchanged by arbitrary jig and master placement");
		j=jig();j.masters[0].body=Cylinder(10,20);
		j.masters[0].jacks[0].position={x:3,y:4,z:10};
		j.masters[0].jacks[1].position={x:0,y:0,z:0};
		j.masters[0].jacks[2].position={x:0,y:0,z:20};
		check(contact(ToolingCheckStatus.Pass),"round master caps and curved boundary are exact contact candidates");
		j.masters[0].jacks[0].position={x:4,y:4,z:0};
		check(contact(ToolingCheckStatus.Fail),"a point inside the cylinder bounding box but outside the circle fails");
		j.masters[0].jacks[0].position={x:0,y:0,z:10};
		check(contact(ToolingCheckStatus.Fail),"an internal cylinder point is not a boundary contact");
		j.masters[0].jacks=[];
		check(contact(ToolingCheckStatus.Pending),"no contacts cannot vacuously pass stock contact qualification");
	}

	static function selectedFrame():Void {
		var source=input(),material=materia.casting.InsertMaterial.InsertMaterials.specification(materia.casting.InsertMaterial.C45Bright);
		var a=source.interfaces[0],b=source.interfaces[1];source.interfaces=[];
		source.inserts=[for (r in [a,b]) {id:r.id,body:materia.casting.InsertSpec.InsertBody.Primitive(Box(40,30,20)),
			holes:new Array<materia.casting.InsertHole>(),interfaces:[r],
			origin:materia.casting.InsertOrigin.Fabricated(materia.casting.InsertManufacturing.InsertManufacturingCodec.draft(material)),
			material:material,pose:F.identity(),role:materia.casting.InsertSpec.InsertRole.GuideSeat,label:"synthetic jig selection",
			positionTolerance:0.01,finish:materia.casting.InsertSpec.InsertFinish.ReplicatedAfterCasting}];
		var relation:InterfaceRelationship={id:"rail-planes",kind:Coplanar,
			members:[ComponentInterface("left","a"),ComponentInterface("right","b")],tolerance:Linear(0.01),
			metrology:{method:MetrologyMethod.IndicatorReversal,instrumentReference:null,uncertaintyMm:null}};
		source.interfaceContext=CastingInterfaceContexts.snapshot({bodyMember:"casting",inserts:[{id:"a",member:"left"},{id:"b",member:"right"}],
			relationships:[{id:"rail-planes",relationship:relation}],sources:[Precision("left",a,F.identity()),Precision("right",b,F.identity())]});
		var common=jig();common.masters[0].members=relation.members.copy();
		var independent=Codec.snapshot([common])[0];independent.id="other-frame";
		var process:Array<materia.tooling.casting.CastingMoldPlan.PrecisionInterface> = [for (r in [a,b]) {
			insertId:r.id,masterReference:null,jigReference:"common-frame",masterFlatnessMm:null,releaseSpec:null,holeProtectionReference:null,evidence:[]}];
		check(ReplicationJigs.analyze(source,[common,independent],process).filter(c ->
			StringTools.startsWith(c.rule,"tooling.replication.relationship-frame.") && c.status==ToolingCheckStatus.Pass).length==1,
			"both inserts select their common jig frame");
		process[1].jigReference="other-frame";
		check(ReplicationJigs.analyze(source,[common,independent],process).filter(c ->
			StringTools.startsWith(c.rule,"tooling.replication.relationship-frame.") && c.status==ToolingCheckStatus.Pending).length==1,
			"unused complete inventory cannot hide inserts selecting different frames");
		process[1].jigReference=null;
		check(ReplicationJigs.analyze(source,[common,independent],process).filter(c ->
			StringTools.startsWith(c.rule,"tooling.replication.relationship-frame.") && c.status==ToolingCheckStatus.Pending).length==1,
			"unselected jig does not establish the relationship frame");
	}

	public static function run():Void {
		var source=input(),j=jig(),encoded=ToolingCodec.canonical(Codec.records([j]));
		check(ToolingCodec.canonical(Codec.records(Codec.snapshot(haxe.Json.parse(encoded))))==encoded,"strict jig records round trip");
		j.masters[0].members.reverse();j.masters[0].jacks.reverse();
		check(ToolingCodec.canonical(Codec.records([j]))==encoded,"member and jack order does not change jig encoding");
		var copy=Codec.snapshot([j]);copy[0].masters[0].pose.z=999;
		check(j.masters[0].pose.z==20,"jig poses are defensive copies");
		rejects(() -> Codec.snapshot([j,j]),"Duplicate replication jig");
		var raw:Array<Dynamic> = cast haxe.Json.parse(encoded),masters:Array<Dynamic> = cast Reflect.field(raw[0],"masters");
		Reflect.setField(masters[0],"kind","invented-master");
		rejects(() -> Codec.snapshot(raw),"Unknown replication master kind");
		j=jig();j.masters[0].finish=InterfaceFinish.SelfLevelled;
		rejects(() -> Codec.snapshot([j]),"physical master finish");
		j=jig();j.masters[0].kind=ReplicationMasterKind.Rail;
		rejects(() -> Codec.snapshot([j]),"kind conflicts with its finish");
		j=jig();var checks=ReplicationJigs.analyze(source,[j],[]);
		check(status(checks,".supports",ToolingCheckStatus.Pass),"three non-collinear jack contacts and lock references satisfy only layout prerequisites");
		check(status(checks,".datum",ToolingCheckStatus.Pending),"missing source datum scheme stays pending");
		check(status(checks,".contact-geometry",ToolingCheckStatus.Unsupported),"stock records cannot claim native contact geometry qualification");
		check(status(checks,".rigidity",ToolingCheckStatus.Pending) && status(checks,".inspection",ToolingCheckStatus.Pending),
			"master geometry never proves rigidity or inspection evidence");
		check(checks.filter(c -> StringTools.startsWith(c.rule,"tooling.replication.relationship-frame.") && c.status==ToolingCheckStatus.Pass).length==1,
			"both source faces are assigned to one declared frame");
		check(!materia.tooling.ToolingValidation.ToolingReadiness.evaluate(checks).ready,
			"jig prerequisite checks have valid severity/status pairs and cannot fabricate readiness");
		ReplicationJigs.validate(source,[j],[],checks);
		checks=checks.filter(c -> !StringTools.endsWith(c.rule,".contact-geometry"));
		rejects(() -> ReplicationJigs.validate(source,[j],[],checks),"Replication jig validation differs");
		j.masters[0].jacks[2].position={x:0,y:-5,z:0};
		check(status(ReplicationJigs.analyze(source,[j],[]),".supports",ToolingCheckStatus.Fail),"collinear jack contacts fail");
		j=jig();j.masters[0].jacks[0].lockReference=null;
		check(status(ReplicationJigs.analyze(source,[j],[]),".supports",ToolingCheckStatus.Pending),"missing lock identification stays pending");
		j=jig();j.masters[0].jacks.pop();
		check(status(ReplicationJigs.analyze(source,[j],[]),".supports",ToolingCheckStatus.Fail),"two contacts fail the three-point support contract");
		j=jig();source.interfaces[0].permittedFinishes=[InterfaceFinish.RailMaster];
		var permittedContext=source.interfaceContext;
		if (permittedContext==null) throw "Expected fixture context";
		permittedContext.sources[0]=Precision("casting",source.interfaces[0],F.identity());
		check(status(ReplicationJigs.analyze(source,[j],[]),".source-members",ToolingCheckStatus.Fail),"master finishing method must be permitted by the source interface");
		source=input();j=jig();j.masters[0].members=[ComponentInterface("casting","a")];
		var other=jig();other.id="independent-frame";other.masters[0].members=[ComponentInterface("casting","b")];
		check(ReplicationJigs.analyze(source,[j,other],[]).filter(c -> StringTools.startsWith(c.rule,"tooling.replication.relationship-frame.")
			&& c.status==ToolingCheckStatus.Pending).length==1,"two unrelated frames do not establish a shared relationship");
		j=jig();j.masters[0].members.push(ComponentInterface("casting","unknown"));
		check(status(ReplicationJigs.analyze(source,[j],[]),".source-members",ToolingCheckStatus.Fail),"unknown source interfaces fail explicitly");
		j=jig();source.interfaceContext=null;
		check(status(ReplicationJigs.analyze(source,[j],[]),".source-members",ToolingCheckStatus.Pending),"missing assembly context never infers master source members");
		source=input();var context=source.interfaceContext;
		if (context==null) throw "Expected fixture context";
		context.sources=[Precision("casting",source.interfaces[0],F.identity()),Axis("Z",F.identity(),{x:0,y:0,z:1})];
		source.interfaces=[source.interfaces[0]];
		context.relationships[0].relationship.members=[ComponentInterface("casting","a"),MachineAxis("Z")];
		context.relationships[0].relationship.kind=Parallel;
		context.relationships[0].relationship.tolerance=PerLength(0.01,100);
		source.interfaceContext=CastingInterfaceContexts.snapshot(context);
		j=jig();j.masters[0].members=[ComponentInterface("casting","a")];
		var axis=jig().masters[0];axis.id="axis-master";axis.kind=ReplicationMasterKind.AxisMaster;axis.body=Cylinder(10,80);axis.members=[MachineAxis("Z")];
		j.masters.push(axis);
		check(ReplicationJigs.analyze(source,[j],[]).filter(c -> StringTools.startsWith(c.rule,"tooling.replication.relationship-frame.")
			&& c.status==ToolingCheckStatus.Pass).length==1,"axis master and replicated plane share the declared frame");
		axis.kind=ReplicationMasterKind.GroundBar;
		check(status(ReplicationJigs.analyze(source,[j],[]),".source-members",ToolingCheckStatus.Fail),"machine axis cannot silently use a plane master");
		placement();
		selectedFrame();
		contactBoundary();
		Sys.println('ReplicationJigTests: $assertions assertions passed');
	}
}
