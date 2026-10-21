import materia.casting.InterfaceRequirement;
import materia.casting.InterfaceRequirement.InterfaceRequirements;
import materia.casting.InterfaceRelationship;
import materia.casting.InterfaceRelationship.InterfaceRelationships;
import materia.assembly.AssemblyFrames as F;

/** Synthetic dimensions exercise contracts; none chooses a production replication layer. */
class FoundationRequirementTests {
	static var assertions = 0;
	static function check(ok:Bool, message:String):Void {
		assertions++;
		if (!ok) throw message;
	}
	static function rejects(fn:Void -> Void, expected:String):Void {
		var error = "";
		try fn() catch (e:Dynamic) error = Std.string(e);
		check(error.indexOf(expected) >= 0, 'Expected "$expected", got "$error"');
	}
	static function requirement():InterfaceRequirement return {
		id: "rail-plane", geometry: Plane(F.translation(0,0,20)), route: Replicated, allowanceMm: 1,finishingProcessErrorMm:null,
		permittedFinishes: [JigMaster,SurfacePlateMaster], positionToleranceMm: 0.01,
		datumReference: "primary", relationshipIds: ["rail-coplanarity"]
	};
	static function relationship():InterfaceRelationship return {
		id: "rail-coplanarity", kind: Coplanar,
		members: [ComponentInterface("base/railLeft","rail-plane"),ComponentInterface("base/railRight","rail-plane")],
		tolerance: Linear(0.005),
		metrology: {method: IndicatorReversal,instrumentReference: "synthetic-instrument",uncertaintyMm: 0.002}
	};
	public static function main():Void {
		var r = requirement();
		check(InterfaceRequirements.fixtureBudgetMm(r)==null,"undeclared finishing allocation has no invented budget");
		r.finishingProcessErrorMm=0.003;
		var firstBudget=InterfaceRequirements.fixtureBudgetMm(r);
		check(firstBudget!=null && Math.abs(firstBudget-0.007)<1e-12,"error share subtracts from tolerance");
		var frameBefore=InterfaceRequirements.geometryFrame(r.geometry);
		r.finishingProcessErrorMm=0.004;
		check(InterfaceRequirements.geometryFrame(r.geometry).z==frameBefore.z && r.allowanceMm==1,
			"error share changes neither source geometry nor finishing stock");
		r.allowanceMm=2;
		var secondBudget=InterfaceRequirements.fixtureBudgetMm(r);
		check(secondBudget!=null && Math.abs(secondBudget-0.006)<1e-12,"geometric layer never consumes error budget");
		r.finishingProcessErrorMm=0.01;
		r.finishingProcessErrorMm=-1;
		rejects(() -> InterfaceRequirements.snapshot([r]),"Invalid finishing process error allocation");
		r=requirement();
		var record = InterfaceRequirements.records([r]);
		var encoded = haxe.Json.stringify(record);
		check(haxe.Json.stringify(InterfaceRequirements.records(InterfaceRequirements.snapshot(haxe.Json.parse(encoded)))) == encoded,
			"requirement round trip");
		var copy = InterfaceRequirements.snapshot([r])[0];
		copy.permittedFinishes.pop();copy.relationshipIds.push("other");
		check(r.permittedFinishes.length == 2 && r.relationshipIds.length == 1,"source lists are defensive copies");
		switch copy.geometry {case Plane(frame): frame.z=99; default: throw "plane expected";}
		check(InterfaceRequirements.geometryFrame(r.geometry).z == 20,"source frame is a defensive copy");
		r.allowanceMm=0;
		check(InterfaceRequirements.lacksDeclaredAllowance(InterfaceRequirements.snapshot([r])[0]),"absent layer remains an unsatisfied design input");
		r.allowanceMm=1;
		check(!InterfaceRequirements.lacksDeclaredAllowance(r),"declared layer is represented numerically");
		rejects(() -> InterfaceRequirements.snapshot([r,r]),"Duplicate interface requirement");
		r.permittedFinishes=[];
		rejects(() -> InterfaceRequirements.snapshot([r]),"requires permitted finishes");
		r.permittedFinishes=[InterfaceFinish.Ground];
		rejects(() -> InterfaceRequirements.snapshot([r]),"conflicts with its route");
		r=requirement();r.positionToleranceMm=0;
		rejects(() -> InterfaceRequirements.snapshot([r]),"allowance or tolerance");
		r=requirement();r.geometry=Line(F.identity());
		rejects(() -> InterfaceRequirements.snapshot([r]),"assembly adjustment route");
		r.route=AdjustedAtAssembly;r.allowanceMm=0;r.permittedFinishes=[PushScrews,TaperedGib,ReplicatedEdge];
		check(InterfaceRequirements.snapshot([r]).length==1,"in-plane adjustment has an explicit permitted finish list");
		var bad:Array<Dynamic> = cast haxe.Json.parse(encoded);Reflect.setField(bad[0],"compat",true);
		rejects(() -> InterfaceRequirements.snapshot(bad),"Unexpected casting record fields");

		var rel=relationship(),first=haxe.Json.stringify(InterfaceRelationships.records([rel]));
		rel.members.reverse();
		check(haxe.Json.stringify(InterfaceRelationships.records([rel]))==first,"member order is canonical");
		check(haxe.Json.stringify(InterfaceRelationships.records(InterfaceRelationships.snapshot(haxe.Json.parse(first))))==first,
			"relationship round trip");
		var margin = InterfaceRelationships.uncertaintyMargin(rel);
		if (margin == null) throw "Expected a declared uncertainty margin";
		var numericMargin:Float = margin;
		check(Math.abs(numericMargin-0.003)<1e-12,"measurement budget margin");
		rel.metrology.uncertaintyMm=0.006;
		var excessMargin = InterfaceRelationships.uncertaintyMargin(rel);
		check(excessMargin != null && excessMargin < 0,"excess measurement uncertainty has negative budget margin");
		rel.metrology.uncertaintyMm=null;
		check(InterfaceRelationships.uncertaintyMargin(rel)==null,"missing evidence has no invented margin");
		rel=relationship();rel.metrology.uncertaintyMm=0;
		rejects(() -> InterfaceRelationships.snapshot([rel]),"uncertainty must be positive");
		rel=relationship();rel.members=[MachineAxis("C"),ComponentInterface("column/railLeft","rail-plane")];
		rel.kind=Parallel;rel.tolerance=PerLength(0.005,300);
		var axisRecords=InterfaceRelationships.records([rel]);
		check(InterfaceRelationships.snapshot(axisRecords).length==1,"machine axis participates without a fabricated insert reference");
		var scoped=InterfaceRelationships.remapMembers(rel,id -> "frame/"+id);
		check(scoped.id==rel.id && scoped.members.filter(member -> switch member {
			case MachineAxis(id): id=="frame/C";
			default:false;
		}).length==1,"nesting remaps physical axis paths while preserving the source relationship definition");
		var badPath=relationship();badPath.members[0]=ComponentInterface("../left","rail-plane");
		rejects(() -> InterfaceRelationships.snapshot([badPath]),"Invalid casting reference path");
		rel.tolerance=Linear(0.005);
		rejects(() -> InterfaceRelationships.snapshot([rel]),"linear/angular basis");
		rel.tolerance=PerLength(0.005,0);
		rejects(() -> InterfaceRelationships.snapshot([rel]),"span must be positive");
		rel=relationship();rel.members=[rel.members[0],rel.members[0]];
		rejects(() -> InterfaceRelationships.snapshot([rel]),"Duplicate relationship member");
		rel=relationship();rel.kind=Straightness;
		rejects(() -> InterfaceRelationships.snapshot([rel]),"member count");
		rel.members=[ComponentInterface("base/casting","master-edge")];
		check(InterfaceRelationships.snapshot([rel]).length==1,"single reference straightness declaration");
		rel=relationship();rel.kind=Distance(-1);
		rejects(() -> InterfaceRelationships.snapshot([rel]),"separation cannot be negative");
		rel=relationship();rel.kind=Distance(5);
		check(InterfaceRelationships.snapshot([rel]).length==1,"distance carries an explicit target");
		rel=relationship();
		var sources:Array<materia.casting.InterfaceRelationship.ScopedInterfaceRequirement> = [
			{member: rel.members[0], requirement: requirement()},
			{member: rel.members[1], requirement: requirement()}
		];
		InterfaceRelationships.validateReferences([rel],sources,[]);
		check(true,"same local interface ID resolves independently on two inserts");
		rejects(() -> InterfaceRelationships.validateReferences([rel],[sources[0]],[]),"Unknown relationship interface");
		sources[0].requirement.relationshipIds=[];
		rejects(() -> InterfaceRelationships.validateReferences([rel],sources,[]),"omits participating relationship");
		sources[0].requirement=requirement();
		sources[0].requirement.relationshipIds.push("missing");
		rejects(() -> InterfaceRelationships.validateReferences([rel],sources,[]),"Unknown interface relationship");
		sources[0].requirement=requirement();
		rejects(() -> InterfaceRelationships.validateReferences([rel],[sources[0],sources[0]],[]),"Duplicate scoped interface");
		rel.members=[sources[0].member,MachineAxis("Y")];
		rejects(() -> InterfaceRelationships.validateReferences([rel],[sources[0]],[]),"Unknown relationship machine axis");
		InterfaceRelationships.validateReferences([rel],[sources[0]],["Y"]);
		check(true,"axis members resolve against the assembly's declared axes");
		rejects(() -> InterfaceRelationships.validateReferences([rel],sources,["Y"]),"references unrelated relationship");
		FixtureConstraintTests.run();
		FoundationDatumTests.run();
		Sys.println('FoundationRequirementTests: $assertions assertions passed');
	}
}
