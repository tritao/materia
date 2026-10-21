import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.tooling.ReplicationContactCad;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;

/** Exact boundary contacts: coincident bounding boxes and sampled corners are insufficient. */
class ReplicationContactTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {assertions++;if (!ok) throw message;}
	static function near(actual:Float,expected:Float,message:String):Void
		check(Math.abs(actual-expected)<1e-5,message+": "+actual+" vs "+expected);
	static function areas(report:machinekit.tooling.PlanarContactCad.PlanarCoverage):{target:Float,uncovered:Float}
		return switch report.areas {
			case Evaluated(target,uncovered):{target:target,uncovered:uncovered};
			case NotEvaluated:throw "Expected evaluated contact areas";
		};
	static function source(insert:machinekit.structural.CastInsert):materia.tooling.casting.CastingToolingInput {
		var foundation=new machinekit.structural.MineralFoundation(new machinekit.component.ComponentValues()
			.setNumber("width",100).setNumber("depth",80).setNumber("height",40),
			[{id:"seat",part:insert,pose:F.translation(0,0,20),openings:[materia.casting.CastingPatterns.face("body.z-max")]}]);
		var assembly=new machinekit.assembly.MachineAssembly();assembly.include("base",foundation,F.identity());
		var context=machinekit.tooling.FoundationAssemblyContext.snapshot(assembly,"base/casting",[{id:"seat",member:"base/seat"}]);
		return machinekit.tooling.FoundationToolingAdapter.snapshot(foundation,"replication-contact",1,context);
	}
	static function replicatedLayer():Void {
		var requirement:materia.casting.InterfaceRequirement={id:"finished-plane",
			geometry:materia.casting.InterfaceRequirement.PrecisionGeometry.Plane(F.translation(0,0,21)),
			route:materia.casting.InterfaceRequirement.InterfaceRoute.Replicated,allowanceMm:1,finishingProcessErrorMm:null,
			permittedFinishes:[materia.casting.InterfaceRequirement.InterfaceFinish.JigMaster],positionToleranceMm:0.01,
			datumReference:"primary",relationshipIds:[]};
		var values=machinekit.structural.CastInsert.boxValues(40,30,20)
			.setToken("role",materia.casting.InsertSpec.InsertRole.GuideSeat)
			.setToken("finish",materia.casting.InsertSpec.InsertFinish.ReplicatedAfterCasting)
			.setToken("interfaces",haxe.Json.stringify(materia.casting.InterfaceRequirement.InterfaceRequirements.records([requirement])));
		var insert=new machinekit.structural.CastInsert(values);
		var input=source(insert);
		var jigs:Array<materia.tooling.casting.ReplicationJig> = [{id:"layer-jig",datumSchemeId:"pending-datums",frameReference:"synthetic-frame",
			pose:F.identity(),masters:[{id:"master",kind:materia.tooling.casting.ReplicationJig.ReplicationMasterKind.GroundPlate,
				body:materia.casting.CastingSolid.Box(50,40,2),pose:F.translation(0,0,41),reference:F.identity(),
				members:[materia.casting.InterfaceRelationship.InterfaceMember.ComponentInterface("base/seat","finished-plane")],
				finish:materia.casting.InterfaceRequirement.InterfaceFinish.JigMaster,jacks:[],releaseSpec:null,flatnessMm:null,inspectionReference:null}]}];
		var checks=ReplicationContactCad.review(input,jigs);
		check(checks.length==1 && checks[0].status==ToolingCheckStatus.Pass,"source-derived replication layer has exact finite CAD contact coverage");
		var steel=insert.geometry();
		try near(steel.volume(),40*30*20,"replication layer is not silently added to the steel")
		catch (error:Dynamic) {steel.close();throw error;}
		steel.close();
		jigs[0].masters[0].body=materia.casting.CastingSolid.Box(25,40,2);
		check(ReplicationContactCad.review(input,jigs)[0].status==ToolingCheckStatus.Fail,"undersized master fails coverage of the actual layer region");
		input.inserts[0].holes=machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.through(6,20));
		check(ReplicationContactCad.review(input,jigs)[0].status==ToolingCheckStatus.Unsupported,
			"an undecided hole route cannot become a qualified layer footprint");
		jigs[0].masters[0].body=materia.casting.CastingSolid.Box(50,40,2);
		var back=machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.tappedBlind(
			machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(6).mating()),20))[0];
		back.frame=F.identity();input.inserts[0].holes=[back];
		switch input.inserts[0].origin {
			case Fabricated(manufacturing):manufacturing.holeRoutes=[{holeId:back.id,route:materia.casting.InsertManufacturing.InsertHoleRoute.BeforeCasting}];
			case Catalogue(_):throw "Expected manufactured source";
		}
		check(ReplicationContactCad.review(input,jigs)[0].status==ToolingCheckStatus.Pass,
			"blind back-face anchorage does not need front-layer bore protection");
		back.frame=F.fromRotationMatrix(0,0,20,[1,0,0,0,-1,0,0,0,-1]);
		check(ReplicationContactCad.review(input,jigs)[0].status==ToolingCheckStatus.Unsupported,
			"a pre-cast front hole requires its actual layer-protection geometry");
		switch input.inserts[0].origin {
			case Fabricated(manufacturing):manufacturing.holeRoutes[0].route=materia.casting.InsertManufacturing.InsertHoleRoute.TransferAfterCasting;
			case Catalogue(_):throw "Expected manufactured source";
		}
		check(ReplicationContactCad.review(input,jigs)[0].status==ToolingCheckStatus.Pass,
			"a hole drilled after replication does not remove the as-replicated contact footprint");
		var ground=materia.casting.InterfaceRequirement.InterfaceRequirements.snapshot([requirement])[0];
		ground.geometry=materia.casting.InterfaceRequirement.PrecisionGeometry.Plane(F.translation(0,0,19));
		ground.route=materia.casting.InterfaceRequirement.InterfaceRoute.Ground;
		ground.permittedFinishes=[materia.casting.InterfaceRequirement.InterfaceFinish.Ground];
		var roundValues=values.copy().setToken("body",materia.casting.InsertSpec.InsertBodyCodec.encode(
			materia.casting.InsertSpec.InsertBody.Primitive(materia.casting.CastingSolid.Cylinder(40,20))))
			.setToken("interfaces",haxe.Json.stringify(materia.casting.InterfaceRequirement.InterfaceRequirements.records([ground])))
			.setToken("finish",materia.casting.InsertSpec.InsertFinish.GroundAfterCasting);
		check(ReplicationContactCad.review(source(new machinekit.structural.CastInsert(roundValues)),jigs)[0].status==ToolingCheckStatus.Unsupported,
			"unsupported source finishing does not become a failed contact against an unfinished preview");
	}

	public static function main():Void run();

	public static function run():Void {
		var parts:Array<Part> = [];
		function own(part:Part):Part {parts.push(part);return part;}
		try {
			var master=own(Part.box(10,10,2));
			var small=own(Part.box(8,8,2)),target=own(small.translated(new Vector(0,0,-2)));
			var report=machinekit.tooling.PlanarContactCad.coverage(master,F.identity(),target,F.identity());
			check(report.status==ToolingCheckStatus.Pass,"larger master covers the actual target boundary");
			near(areas(report).target,64,"exact rectangular contact area");
			near(areas(report).uncovered,0,"no missing contact area");
			var narrow=own(Part.box(7,12,2));
			report=machinekit.tooling.PlanarContactCad.coverage(narrow,F.identity(),target,F.identity());
			check(report.status==ToolingCheckStatus.Fail,"narrow master fails finite face coverage");
			near(areas(report).uncovered,8,"missing edge strips have exact area");
			var disk=own(Part.cylinder(5,2)),square=own(master.translated(new Vector(0,0,-2)));
			report=machinekit.tooling.PlanarContactCad.coverage(disk,F.identity(),square,F.identity());
			check(report.status==ToolingCheckStatus.Fail,"identical bounding boxes do not make a circular master cover square corners");
			near(areas(report).uncovered,100-25*Math.PI,"circular boundary subtraction retains exact arcs");
			var roundTarget=own(disk.translated(new Vector(0,0,-2)));
			report=machinekit.tooling.PlanarContactCad.coverage(master,F.identity(),roundTarget,F.identity());
			check(report.status==ToolingCheckStatus.Pass,"square master covers an exact circular target");
			near(areas(report).target,25*Math.PI,"circular target area");
			var core=own(Part.cylinder(2,2)),ring=own(disk.subtractAll([core]));
			var smallerDisk=own(Part.cylinder(4,2)),solidTarget=own(smallerDisk.translated(new Vector(0,0,-2)));
			report=machinekit.tooling.PlanarContactCad.coverage(ring,F.identity(),solidTarget,F.identity());
			check(report.status==ToolingCheckStatus.Fail,"a master hole leaves uncovered target despite enclosing its perimeter");
			near(areas(report).uncovered,4*Math.PI,"missing central region is the exact bore area");
			var smallerRing=own(smallerDisk.subtractAll([core])),ringTarget=own(smallerRing.translated(new Vector(0,0,-2)));
			report=machinekit.tooling.PlanarContactCad.coverage(ring,F.identity(),ringTarget,F.identity());
			check(report.status==ToolingCheckStatus.Pass,"actual target holes are retained rather than filled by a bounding outline");
			near(areas(report).target,12*Math.PI,"annular contact area excludes its hole");
			var common=F.compose(F.translation(30,20,-10),F.turnY(0.45));
			var rotatedMaster=own(PosedParts.moved(master,common)),rotatedTarget=own(PosedParts.moved(target,common));
			report=machinekit.tooling.PlanarContactCad.coverage(rotatedMaster,common,rotatedTarget,common);
			check(report.status==ToolingCheckStatus.Pass,"exact contact coverage is invariant under a common oblique rigid transform");
			near(areas(report).target,64,"oblique planar boolean area");
			report=machinekit.tooling.PlanarContactCad.coverage(master,F.translation(0,0,1),target,F.identity());
			check(report.status==ToolingCheckStatus.Fail && report.message.indexOf("actual planar stock boundary")>=0,
				"interior volume section is rejected as a master boundary");
			check(switch report.areas {case NotEvaluated:true;default:false;},"unevaluated coverage areas are not reported as zero");
			var penetrating=own(master.translated(new Vector(0,0,-2)));
			report=machinekit.tooling.PlanarContactCad.coverage(penetrating,F.identity(),target,F.identity());
			check(report.status==ToolingCheckStatus.Fail && report.message.indexOf("overlaps")>=0,
				"coplanar faces with solid interpenetration are rejected");
			var separated=own(target.translated(new Vector(0,0,-0.005)));
			report=machinekit.tooling.PlanarContactCad.coverage(master,F.identity(),separated,F.translation(0,0,-0.005));
			check(report.status==ToolingCheckStatus.Unsupported,"noncoincident planes do not become an invented tolerance-aware layer projection");
			var invalid=F.identity(),error="";invalid.qw=0;
			try machinekit.tooling.PlanarContactCad.coverage(master,F.identity(),target,invalid) catch (failure:Dynamic) error=Std.string(failure);
			check(error.indexOf("finite position and unit rotation")>=0,"invalid reference frames are rejected before native contact operations");
			check(master.valid() && target.valid(),"coverage borrows and preserves its source solids");
			near(master.volume(),200,"borrowed master remains usable after repeated checks");
		} catch (error:Dynamic) {for (part in parts) part.close();throw error;}
		for (part in parts) part.close();
		replicatedLayer();
		Sys.println('ReplicationContactTests: $assertions assertions passed');
	}
}
