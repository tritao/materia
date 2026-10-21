import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import machinekit.structural.CastInsert;
import machinekit.structural.MineralFoundation;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import machinekit.tooling.FoundationAssemblyContext;
import machinekit.tooling.FoundationToolingAdapter;
import materia.casting.CastingInterfaceContext.InterfaceContextSource;
import materia.casting.InterfaceRequirement;
import materia.casting.InterfaceRelationship;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.InterfaceQualification;
import materia.assembly.AssemblyFrames as F;

/** Synthetic assembly evidence; production accuracy and compound choices remain undeclared. */
class FoundationContextTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {
		assertions++;
		if (!ok) throw message;
	}
	static function rejects(fn:Void->Void,expected:String):Void {
		var error="";
		try fn() catch(value:Dynamic) error=Std.string(value);
		check(error.indexOf(expected)>=0,'Expected "$expected", got "$error"');
	}
	public static function main():Void run();

	public static function fabricationSpec():materia.tooling.export.FabricationSpec return {revision:1,sheetMaterialId:"tooling-coated-board",
			solvent:materia.tooling.export.FabricationSpec.ResinSolvent.Unknown,adhesiveContainsEPS:null,
			resinReference:"synthetic-context-binder",compatibilityEvidence:[],gravityMps2:9.80665,loadFactor:1.5,
			compactionPressurePa:null,compactionEvidence:[],plates:[],braces:[],fasteners:[],coreRestraints:[],rigidRestraints:[],
			blankMarginMm:12,blankKerfMm:3.2,couponClearancesMm:[0.1,0.2,0.3]};


	public static function run():Void {
		var requirements:Array<InterfaceRequirement> = [{id:"seat-plane",geometry:Plane(F.translation(0,0,20)),
			route:InterfaceRoute.AsSupplied,allowanceMm:0,finishingProcessErrorMm:null,permittedFinishes:[InterfaceFinish.AsSupplied],
			positionToleranceMm:0.01,datumReference:"primary",relationshipIds:["seat-pair"]}];
		var values=CastInsert.boxValues(40,30,20).setToken("role",materia.casting.InsertSpec.InsertRole.GuideSeat)
			.setToken("finish",materia.casting.InsertSpec.InsertFinish.AsSupplied)
			.setToken("interfaces",haxe.Json.stringify(materia.casting.InterfaceRequirement.InterfaceRequirements.records(requirements)));
		var placements:Array<CastInsertPlacement> = [for (side in [-1,1]) {
			var declared=materia.casting.InterfaceRequirement.InterfaceRequirements.snapshot(requirements);
			declared[0].relationshipIds.push(side<0 ? "seat-axis-left" : "seat-axis-right");
			{id:side<0 ? "left" : "right",part:new CastInsert(values.copy().setToken("interfaces",
				haxe.Json.stringify(materia.casting.InterfaceRequirement.InterfaceRequirements.records(declared)))),pose:F.translation(side*25,0,40),
				openings:[materia.casting.CastingPatterns.face("body.z-max")]};
		}];
		var foundation=new MineralFoundation(new ComponentValues().setNumber("width",100).setNumber("depth",80).setNumber("height",60),placements);
		var assembly=new MachineAssembly();
		assembly.include("base",foundation,F.fromRotationMatrix(300,-200,50,[0,0,1,0,1,0,-1,0,0]));
		var witness=new CastInsert(CastInsert.boxValues(10,10,10).setToken("role",materia.casting.InsertSpec.InsertRole.GenericMount));
		assembly.addComponent("axisWitness",witness);
		assembly.addMateOnAxis("C",materia.assembly.AssemblyDefinition.AssemblyJointType.Revolute,
			"base/casting","mount","axisWitness","mount",{x:0,y:1,z:0});
		var relationship:InterfaceRelationship={id:"seat-pair",kind:Parallel,
			members:[ComponentInterface("base/left","seat-plane"),ComponentInterface("base/right","seat-plane")],
			tolerance:PerLength(0.005,300),metrology:{method:IndicatorReversal,instrumentReference:"synthetic-instrument",uncertaintyMm:0.006}};
		assembly.addDesignRequirement(machinekit.design.DesignRequirement.InterfaceAlignment("seat-alignment",relationship));
		for (side in ["left","right"]) assembly.addDesignRequirement(machinekit.design.DesignRequirement.InterfaceAlignment("seat-axis-"+side+"-alignment",{
			id:"seat-axis-"+side,kind:Perpendicular,members:[ComponentInterface("base/"+side,"seat-plane"),MachineAxis("C")],
			tolerance:PerLength(0.005,300),metrology:{method:AxisMasterIndicator,instrumentReference:"synthetic-instrument",uncertaintyMm:0.006}}));
		var bindings=[{id:"left",member:"base/left"},{id:"right",member:"base/right"}];
		var context=FoundationAssemblyContext.snapshot(assembly,"base/casting",bindings);
		check(context.relationships.length==3 && context.sources.length==3,"source assembly supplies seat parallelism and both plane-axis squareness relationships");
		var input=FoundationToolingAdapter.snapshot(foundation,"context-fixture",1,context);
		var encoded=ToolingCodec.encodeInput(input);
		check(ToolingCodec.encodeInput(ToolingCodec.decodeInput(encoded))==encoded,"adapter and source codec preserve assembly context");
		bindings.reverse();
		check(ToolingCodec.inputFingerprint(FoundationToolingAdapter.snapshot(foundation,"context-fixture",1,
			FoundationAssemblyContext.snapshot(assembly,"base/casting",bindings)))==ToolingCodec.inputFingerprint(input),
			"binding order does not change the context fingerprint");
		for (source in context.sources) switch source {
			case Precision(member,_,pose):
				check(Math.abs(pose.x-(member=="base/left" ? -25 : 25))<1e-8 && Math.abs(pose.z-40)<1e-8,
					"insert poses use foundation coordinates despite the service assembly rotation");
			case Axis(_,frame,direction):
				var global=F.transformVector(frame,direction.x,direction.y,direction.z);
				check(Math.abs(frame.x)+Math.abs(frame.y)+Math.abs(frame.z)<1e-8
					&& Math.abs(global.x)+Math.abs(global.y)+Math.abs(global.z-1)<1e-8,
					"machine axis composes its connector frame with the local joint direction into foundation coordinates");
		}
		var checks=InterfaceQualification.relationships(input);
		check(checks.length==6 && checks.filter(row -> row.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending).length==6,
			"source metrology uncertainty reaches tooling and remains pending");
		check(checks[0].affectedIds.indexOf("left")>=0 && checks[0].affectedIds.indexOf("right")>=0,
			"relationship validation identifies the affected source inserts");
		var altered=ToolingCodec.inputSnapshot(input);
		var alteredContext=altered.interfaceContext;
		if (alteredContext==null) throw "Expected a source context";
		for (source in alteredContext.sources) switch source {
			case Precision(member,_,pose): if (member=="base/left") pose.x+=1;
			case Axis(_,_,_):
		}
		rejects(() -> ToolingCodec.inputSnapshot(altered),"Interface context differs from source requirement or pose");
		var missing:Dynamic=haxe.Json.parse(encoded);
		Reflect.deleteField(missing.input,"datumScheme");
		rejects(() -> ToolingCodec.decodeInput(haxe.Json.stringify(missing)),"Missing input field: datumScheme");
		var budgeted=ToolingCodec.inputSnapshot(input);budgeted.interfaceContext=null;
		var budgetFingerprint=ToolingCodec.inputFingerprint(budgeted);
		budgeted.inserts[0].interfaces[0].finishingProcessErrorMm=0.002;
		check(ToolingCodec.inputFingerprint(budgeted)!=budgetFingerprint,"source error allocations enter the fingerprint");
		context.relationships[0].relationship.metrology.uncertaintyMm=0.001;
		check(ToolingCodec.encodeInput(input)==encoded,"context snapshots cannot be mutated through the caller's records");
		var nested=new MachineAssembly();nested.include("station",MachineAssembly.decode(assembly.encode()),F.translation(100,100,100));
		var nestedContext=FoundationAssemblyContext.snapshot(nested,"station/base/casting",
			[{id:"left",member:"station/base/left"},{id:"right",member:"station/base/right"}]);
		var nestedInput=FoundationToolingAdapter.snapshot(foundation,"context-fixture",1,nestedContext);
		check(nestedContext.relationships[0].id=="station/seat-alignment" && nestedContext.relationships[0].relationship.id=="seat-pair",
			"saved nested assemblies retain source definitions and qualify physical paths");
		check(ToolingCodec.inputFingerprint(nestedInput)!=ToolingCodec.inputFingerprint(input),"assembly occurrence context enters the source fingerprint");
		rejects(() -> FoundationAssemblyContext.snapshot(assembly,"base/left",bindings),"body binding needs a source casting");
		var wrong=FoundationAssemblyContext.snapshot(assembly,"base/casting",[{id:"left",member:"axisWitness"},{id:"right",member:"base/right"}]);
		rejects(() -> FoundationToolingAdapter.snapshot(foundation,"context-fixture",1,wrong),"Interface context omits source requirements");
		Sys.println('FoundationContextTests: $assertions assertions passed');
	}
}
