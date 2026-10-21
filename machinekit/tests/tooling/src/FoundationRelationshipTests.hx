import machinekit.assembly.MachineAssembly;
import machinekit.design.DesignRequirement;
import machinekit.design.AssemblyDesignChecks;
import machinekit.structural.CastInsert;
import materia.casting.InterfaceRequirement;
import materia.casting.InterfaceRelationship;
import materia.assembly.AssemblyFrames as F;

/** Synthetic requirements exercise persistence and assembly scope, not mill accuracy choices. */
class FoundationRelationshipTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {
		assertions++;
		if (!ok) throw message;
	}
	static function relationship():InterfaceRelationship return {
		id:"pair-plane",kind:Coplanar,members:[ComponentInterface("left","mounting-plane"),ComponentInterface("right","mounting-plane")],
		tolerance:Linear(0.005),metrology:{method:IndicatorReversal,instrumentReference:"synthetic-instrument",uncertaintyMm:0.002}
	};
	static function fixture(relationship:InterfaceRelationship):MachineAssembly {
		var requirements:Array<InterfaceRequirement> = [{
			id:"mounting-plane",geometry:Plane(F.translation(0,0,20)),route:InterfaceRoute.AsSupplied,allowanceMm:0,finishingProcessErrorMm:null,
			permittedFinishes:[InterfaceFinish.AsSupplied],positionToleranceMm:0.01,datumReference:"primary",relationshipIds:["pair-plane"]
		}];
		var values=CastInsert.boxValues(40,30,20).setToken("role",materia.casting.InsertSpec.InsertRole.GuideSeat)
			.setToken("finish",materia.casting.InsertSpec.InsertFinish.AsSupplied)
			.setToken("interfaces",haxe.Json.stringify(materia.casting.InterfaceRequirement.InterfaceRequirements.records(requirements)));
		var result=new MachineAssembly();
		result.addComponent("left",new CastInsert(values));
		result.addComponent("right",new CastInsert(values),F.translation(60,0,0));
		result.addDesignRequirement(InterfaceAlignment("plane-instance",relationship));
		return result;
	}
	public static function main():Void run();

	public static function run():Void {
		var source=fixture(relationship()),saved=source.encode(),restored=MachineAssembly.decode(saved);
		check(restored.encode()==saved,"typed relationship assembly round trip");
		var requirements=restored.designRequirements();
		switch requirements[0] {
			case InterfaceAlignment(id,relationship):
				check(id=="plane-instance" && relationship.id=="pair-plane","assembly occurrence and source definition IDs remain distinct");
				relationship.metrology.uncertaintyMm=1;
			default: throw "Expected relationship requirement";
		}
		check(restored.encode()==saved,"returned relationship evidence is a defensive copy");
		var root=new MachineAssembly();
		root.include("first",source);
		root.include("second",restored,F.translation(200,0,0));
		var nested=MachineAssembly.decode(root.encode()).designRequirements();
		check(nested.length==2,"two independent occurrences retain the shared design definition");
		for (requirement in nested) switch requirement {
			case InterfaceAlignment(id,relationship):
				check(relationship.id=="pair-plane","nesting preserves source relationship definition references");
				var prefix=id.split("/")[0];
				for (member in relationship.members) switch member {
					case ComponentInterface(part,face): check(part.indexOf(prefix+"/")==0 && face=="mounting-plane","nested interface member path is remapped");
					default: throw "Expected component reference";
				}
			default: throw "Expected relationship requirement";
		}
		var diagnostics=AssemblyDesignChecks.check(root);
		check(!diagnostics.hasErrors(),"nested source references resolve independently of shared part definitions");
		check(diagnostics.items.filter(item -> item.code=="design.interface.qualification").length==2,
			"resolved metadata does not qualify actual alignment or replication jigs");
		var excessive=relationship();excessive.metrology.uncertaintyMm=0.006;
		diagnostics=AssemblyDesignChecks.check(fixture(excessive));
		check(diagnostics.items.filter(item -> item.code=="design.interface.metrology").length==1,
			"uncertainty above tolerance remains a metrology warning");
		var missing=relationship();missing.members[1]=ComponentInterface("right","absent");
		diagnostics=AssemblyDesignChecks.check(fixture(missing));
		check(diagnostics.hasErrors() && diagnostics.items.filter(item -> item.message.indexOf("Unknown relationship precision interface")>=0).length==1,
			"missing precision source interface is a design error");
		check(diagnostics.items.filter(item -> item.code=="design.interface.relationship-reference").length==1,
			"source interface cannot reference a relationship it does not participate in");
		var axis=relationship();axis.kind=Parallel;axis.tolerance=PerLength(0.005,300);axis.members.push(MachineAxis("C"));
		diagnostics=AssemblyDesignChecks.check(fixture(axis));
		check(diagnostics.hasErrors() && diagnostics.items.filter(item -> item.message.indexOf("Unknown relationship machine axis: C")>=0).length==1,
			"axis labels cannot substitute for an existing machine joint");
		var rotating=fixture(axis);
		rotating.addMateOnAxis("C",materia.assembly.AssemblyDefinition.AssemblyJointType.Revolute,
			"left","mount","right","mount",{x:0,y:0,z:1});
		check(!AssemblyDesignChecks.check(rotating).hasErrors(),"machine axis resolves to an explicitly named directed joint");
		var station=new MachineAssembly();station.include("station",rotating);
		check(!AssemblyDesignChecks.check(MachineAssembly.decode(station.encode())).hasErrors(),
			"nested saved relationships remap both machine joints and interface member paths");
		var fixed=fixture(axis);
		fixed.addMate("C",materia.assembly.AssemblyDefinition.AssemblyJointType.Fixed,"left","mount","right","mount");
		diagnostics=AssemblyDesignChecks.check(fixed);
		check(diagnostics.hasErrors() && diagnostics.items.filter(item -> item.message.indexOf("single directed joint")>=0).length==1,
			"fixed connections do not become machine axes");
		var version=source.describe();version.schemaVersion=20;
		var rejected=false;
		try MachineAssembly.fromDescription(version) catch(error:Dynamic) rejected=Std.string(error).indexOf("unsupported")>=0;
		check(rejected,"assembly schema rejects previous version without a compatibility reader");
		Sys.println('FoundationRelationshipTests: $assertions assertions passed');
	}
}
