import machinekit.assembly.MachineAssembly;
import machinekit.design.AssemblyDesignChecks;
import machinekit.design.DesignRequirement;
import machinekit.milling.MillRailPlacement;
import machinekit.milling.MillRailFasteners;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertStages;
import materia.casting.InsertStages.InsertStage;

class RailSeatTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {assertions++;if (!ok) throw message;}

	public static function main():Void {
		var larger=new machinekit.motion.HgRail("HGR25R",400);
		var drilling=larger.mountingHole();
		check(drilling!=null && drilling.diameter==7 && drilling.counterboreDiameter==11 && drilling.counterboreDepth==9,
			"current HG family exposes its existing pinned drilling geometry");
		check(larger.mountingThread().designation=="M6" && larger.mountingThread().internal,
			"size-only mounting screw catalogue tokens produce a typed source thread");
		var frame=MillMoldSetup.frame();
		var assembly=new MachineAssembly();assembly.include("frame",frame);
		var groups=[
			{name:"xyStage",foundation:frame.base,prefix:"frame/base",rails:MillRailPlacement.y(frame.xyAxis.guided())},
			{name:"zAxis",foundation:frame.column,prefix:"frame/column",rails:MillRailPlacement.z(frame.zAxis)}
		];
		for (group in groups) {
			var rails=new MachineAssembly();
			for (placed in group.rails) rails.addComponent(placed.memberId,placed.rail,placed.pose);
			assembly.include(group.name,rails);
			MillRailFasteners.install(assembly,group.foundation,group.prefix,frame.layout.casting.anchorRule.screwLengthsMm);
		}
		var poses=assembly.solvedPoses(),count=0;
		for (group in groups) for (location in group.foundation.insertLocations()) {
			var spec=location.part.insertSpec();
			var manufacturing=switch spec.origin {case Fabricated(plan):plan;case Catalogue(_):continue;};
			var raw=InsertStages.holes(spec,InsertStage.BeforeCasting);
			for (requirement in manufacturing.mountingRequirements) {
				count++;
				check(raw.filter(hole -> hole.id==requirement.holeId).length==0,
					"unresolved pre-cast budget schedules transfer drilling without a raw bore");
				var members=assembly.components().filter(member -> member.id==requirement.componentPath);
				var rail:machinekit.motion.LinearRail=cast members[0].component;
				var id=MillRailFasteners.idFor(location.id,requirement.holeId);
				var local=F.compose(F.inverse(poses.get(requirement.componentPath)),poses.get(id));
				var geometry=rail.mountingHole(),connector=rail.connector(requirement.connectorId).frame;
				check(geometry!=null && Math.abs(local.y+geometry.counterboreDepth)<1e-7
					&& Math.abs(local.x-connector.x)<1e-7 && Math.abs(local.z-connector.z)<1e-7,
					"every real screw seats at its actual rail counterbore");
				var report=materia.tooling.casting.InsertManufacturingPlan.build({id:location.id,body:spec.body,holes:spec.holes,
					origin:spec.origin,anchorage:spec.anchorage,interfaces:spec.interfaces,pose:location.pose,
					role:cast location.part.parameterToken("role"),label:location.part.parameterToken("label"),
					finish:cast location.part.parameterToken("finish"),positionTolerance:location.part.parameterNumber("positionTolerance"),
					material:location.part.insertMaterial()});
				check(report.holeBudgets.filter(budget -> budget.holeId==requirement.holeId && budget.status=="pending"
					&& budget.route==materia.casting.InsertManufacturing.InsertHoleRoute.TransferAfterCasting).length==1,
					"each hole reports its pending budget and explicit transfer route");
			}
		}
		check(count>0,"production guide seats derive real mounting patterns");
		var declarations=assembly.designRequirements().filter(requirement -> switch requirement {case Coaxial(_,_,_,_):true;default:false;});
		check(declarations.length==2*count,"every screw has rail-axis and source-seat coaxial requirements");
		var diagnostics=AssemblyDesignChecks.check(assembly);
		var axisFailures=diagnostics.items.filter(item -> item.code=="design.fastener-axis");
		check(axisFailures.length==0,"actual rail and steel axes: "+axisFailures.map(item -> item.message).join("; "));
		for (finding in diagnostics.items) if (finding.code!="design.fastener-axis")
			Sys.println("Other source diagnostic: "+finding.code+" "+finding.subject+" "+finding.message);
		var restored=MachineAssembly.decode(assembly.encode());
		check(restored.encode()==assembly.encode(),"coaxial requirements and source routing round trip in assembly schema 23");

		var location=frame.base.insertLocations().filter(row -> switch row.part.insertSpec().origin {
			case Fabricated(plan):plan.mountingRequirements.length>0;case Catalogue(_):false;
		})[0];
		var spec=location.part.insertSpec(),manufacturing=switch spec.origin {case Fabricated(plan):plan;default:throw "Made seat expected";};
		var requirement=manufacturing.mountingRequirements[0];
		var boltId=MillRailFasteners.idFor(location.id,requirement.holeId);
		var bolt=assembly.components().filter(member -> member.id==boltId)[0].component;
		var relative=F.compose(F.inverse(poses.get("frame/base/"+location.id)),poses.get(boltId));
		for (tilt in [false,true]) {
			var bad=new MachineAssembly();bad.addComponent("steel",location.part);
			var rotation=F.identity();rotation.qx=Math.sin(0.00005);rotation.qw=Math.cos(0.00005);
			var changed=tilt ? F.compose(relative,rotation) : F.compose(F.translation(1,0,0),relative);
			bad.addComponent("screw",bolt,changed);
			bad.addDesignRequirement(Coaxial("deliberate-misalignment",{member:"screw",feature:"head"},
				{member:"steel",feature:requirement.holeId},1e-6));
			var failed=AssemblyDesignChecks.check(bad);
			check(failed.items.filter(item -> item.code=="design.fastener-axis" && item.subject=="deliberate-misalignment").length==1,
				tilt ? "small angular mismatch is caught independently" : "real screw displacement is caught independently");
		}
		Sys.println('RailSeatTests: $assertions assertions passed; $count connector-derived service screws');
	}
}
