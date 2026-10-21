package machinekit.design;

import machinekit.assembly.Diagnostics;
import machinekit.component.MachineComponent;
import cadkit.modeling.AssemblyState;
import materia.casting.FoundationJoint;
import materia.casting.FoundationJoint.FoundationJoints;
import materia.casting.FoundationJointAnalysis;
import materia.assembly.AssemblyFrames as F;

/** Native source resolution followed by the same nominal checks as tooling. */
class FoundationJointDesignChecks {
	public static function check(out:Diagnostics,id:String,authored:FoundationJoint,
		members:Map<String,MachineComponent>,state:AssemblyState):Void {
		var joint=FoundationJoints.snapshot(authored);
		var source=FoundationJointSourceSupport.capture([{id:id,joint:joint}],members,state,F.identity());
		for (finding in FoundationJointAnalysis.check(source,joint)) if (!finding.passed)
			out.error("design.foundation-joint."+finding.code,id+"/"+finding.subject,finding.message);
		for (contact in machinekit.tooling.FoundationJointCad.contacts(source,joint,member -> {
			var part=members.get(member);if (part==null) throw "Missing joint component: "+member;return part.geometry();
		},member -> state.worldPose(member))) if (contact.report.status!=materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass)
			out.error("design.foundation-joint.jack-finite-contact",id+"/"+contact.id,contact.report.message);
		out.warning("design.foundation-joint.qualification",id,
			"Source joint references and nominal geometry and jack-tip coverage checked; joint-seat coverage, jack restraint, full anchor stack, grout and cure-before-torque evidence remain unqualified.");
	}
}
