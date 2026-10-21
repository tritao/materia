package machinekit.tooling;

import cadkit.modeling.Part;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.FoundationJoint;
import materia.casting.FoundationJointAnalysis as J;
import materia.casting.FoundationJointSources;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;
import machinekit.assembly.PosedParts;

/** Finite tip-bearing coverage from actual source components. This checks nominal
 * contact only: stability, strength, tightening and measured fit remain separate.
 */
class FoundationJointCad {
	public static function contacts(source:FoundationJointSources,joint:FoundationJoint,
		local:String->Part,pose:String->AssemblyFrame):Array<{id:String,report:PlanarContactCad.PlanarCoverage}> {
		var result:Array<{id:String,report:PlanarContactCad.PlanarCoverage}> = [];
		var locals:Map<String,Part> = [];
		function at(id:String):Part {
			var part=locals.get(id);
			if (part==null) {part=local(id);locals.set(id,part);}
			return PosedParts.moved(part,pose(id));
		}
		try {
			for (jack in joint.jacks) {
				var fastener=J.screw(source,jack.screw),frame=J.plane(source,jack.bearing);
				var tip=F.compose(fastener.bearingFrame,F.translation(0,0,fastener.shankLength));
				var parts:Array<Part> = [];
				try {
					var support=at(jack.bearing.member);parts.push(support);
					var screw=at(jack.screw);parts.push(screw);
					result.push({id:jack.id,report:PlanarContactCad.coverage(support,frame,screw,tip)});
				} catch (error:Dynamic) {
					for (part in parts) part.close();throw error;
				}
				for (part in parts) part.close();
			}
		} catch (error:Dynamic) {for (part in locals) part.close();throw error;}
		for (part in locals) part.close();return result;
	}

	/** Reconstruct only standard/source geometry recorded in the fingerprint. */
	public static function review(input:materia.tooling.casting.CastingToolingInput):Array<ToolingValidation> {
		var result:Array<ToolingValidation> = [],context=input.interfaceContext;
		if (context==null) return result;
		var source=context.jointSources;
		var screws:Map<String,machinekit.standard.SocketHeadCapScrew> = [],poses:Map<String,AssemblyFrame> = [];
		for (row in source.inserts) poses.set(row.member,row.pose);
		for (row in source.screws) {
			var screw=machinekit.standard.SocketHeadCapScrew.metric(row.thread.designation,row.shankLength,machinekit.standard.PropertyClass.Steel88);
			if (screw.bom.partNumber!=row.partNumber || Math.abs(screw.threadedLength()-row.threadedLength)>1e-8
				|| materia.tooling.ToolingCodec.canonical(machinekit.structural.InsertHoles.threadRecord(screw.threadSpec()))
					!=materia.tooling.ToolingCodec.canonical(row.thread))
				throw "Joint hardware differs from source standard component: "+row.member;
			screws.set(row.member,screw);
			poses.set(row.member,F.compose(row.bearingFrame,F.inverse(screw.bearingFrame())));
		}
		for (instance in context.foundationJoints) {
			var prefix="tooling.foundation-joint.cad."+haxe.crypto.Sha256.encode(instance.id);
			for (contact in contacts(source,instance.joint,id -> {
				var screw=screws.get(id);
				if (screw!=null) return screw.geometry();
				return machinekit.structural.InsertGeometry.part(J.insert(source,id).spec,materia.casting.InsertStages.InsertStage.Service);
			},id -> {
				var pose=poses.get(id);if (pose==null) throw "Missing source joint pose: "+id;return pose;
			})) result.push(check(prefix+".jack-contact."+contact.id,input.id,contact.report.status,
				instance.id+"/"+contact.id+": "+contact.report.message));
		}
		result.sort((a,b) -> Reflect.compare(a.rule,b.rule));return result;
	}

	static function check(rule:String,id:String,status:ToolingCheckStatus,message:String):ToolingValidation
		return {rule:rule,affectedIds:[id],status:status,severity:status==ToolingCheckStatus.Pass ? ToolingSeverity.Info : ToolingSeverity.Error,
			message:message,evidence:[]};
}
