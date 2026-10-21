package machinekit.design;

import machinekit.component.MachineComponent;
import machinekit.structural.MineralCasting;
import machinekit.structural.CastInsert;
import machinekit.standard.SocketHeadCapScrew;
import materia.casting.FoundationJointSources;
import materia.casting.FoundationJointSources.FoundationJointSourceCodec;
import materia.casting.FoundationJointAnalysis;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

/** Copy source components, never infer hole patterns or support dimensions. */
class FoundationJointSourceSupport {
	public static function capture(instances:Array<FoundationJointOccurrence>,members:Map<String,MachineComponent>,
		state:AssemblyState,worldToFoundation:AssemblyFrame):FoundationJointSources {
		var result=FoundationJointSourceCodec.empty(),seen:Map<String,Bool> = [];
		function pose(member:String):AssemblyFrame return F.compose(worldToFoundation,state.worldPose(member));
		function body(member:String):Void {
			if (seen.exists(member)) return;
			var part=members.get(member);
			if (!Std.isOfType(part,MineralCasting)) throw "Foundation joint body is not a source casting: "+member;
			var casting:MineralCasting=cast part,slash=member.lastIndexOf("/"),prefix=slash<0 ? "" : member.substr(0,slash+1);
			var owners:Array<String> = [];
			for (feature in casting.castingFeatures()) if (feature.kind==materia.casting.CastingFeature.CastingFeatureKind.InsertRecess
				&& feature.owner!=null && owners.indexOf(prefix+feature.owner)<0) owners.push(prefix+feature.owner);
			seen.set(member,true);result.bodies.push({member:member,pose:pose(member),interfaces:casting.precisionRequirements(),insertMembers:owners});
		}
		function insert(member:String):Void {
			if (seen.exists(member)) return;
			var part=members.get(member);
			if (!Std.isOfType(part,CastInsert)) throw "Foundation joint hole/seat requires a source insert: "+member;
			var casting:CastInsert=cast part;
			if (casting.buildStage()!=materia.casting.InsertStages.InsertStage.Service)
				throw "Foundation joint design requires a service-stage source insert: "+member;
			seen.set(member,true);result.inserts.push({member:member,pose:pose(member),spec:casting.insertSpec()});
		}
		function screw(member:String):Void {
			if (seen.exists(member)) return;
			var part=members.get(member);
			if (!Std.isOfType(part,SocketHeadCapScrew)) throw "Foundation joint screw is not an actual fastener: "+member;
			var fastener:SocketHeadCapScrew=cast part;
			if (fastener.componentType()==null) throw "Unsupported custom foundation joint screw: "+member;
			if (fastener.propertyClass!=machinekit.standard.PropertyClass.Steel88)
				throw "Foundation joint source policy requires ISO 4762 class 8.8: "+member;
			seen.set(member,true);result.screws.push({member:member,kind:FoundationJointScrewKind.Iso4762Steel88,partNumber:fastener.bom.partNumber,
				thread:machinekit.structural.InsertHoles.threadRecord(fastener.threadSpec()),shankLength:fastener.shankLength(),
				threadedLength:fastener.threadedLength(),bearingFrame:F.compose(pose(member),fastener.bearingFrame())});
		}
		for (instance in instances) {body(instance.joint.lowerBody);body(instance.joint.upperBody);}
		for (instance in instances) {
			for (seat in instance.joint.seats) {insert(seat.lower.member);insert(seat.upper.member);}
			for (jack in instance.joint.jacks) {screw(jack.screw);insert(jack.receiver.member);insert(jack.bearing.member);}
			for (anchor in instance.joint.anchors) {
				screw(anchor.screw);insert(anchor.target.member);for (hole in anchor.through) insert(hole.member);
			}
		}
		var snapshot=FoundationJointSourceCodec.snapshot(result);FoundationJointAnalysis.validate(snapshot,instances);return snapshot;
	}
}
