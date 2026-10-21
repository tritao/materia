package materia.tooling.casting;

import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;
import materia.casting.FoundationJointAnalysis;

/** Source intent and recomputed nominal checks are separate from joint process
 * evidence. No material name or cure-reference string alone releases a joint.
 */
class FoundationJointQualification {
	public static function missingSource(input:CastingToolingInput):Null<ToolingValidation> {
		var missing:Array<String> = [],context=input.interfaceContext;
		for (insert in input.inserts) if (insert.role==materia.casting.InsertSpec.InsertRole.ColumnMount) {
			var found=false;
			if (context!=null) {
				var bindings=context.inserts.filter(binding -> binding.id==insert.id);
				if (bindings.length==1) for (instance in context.foundationJoints) for (seat in instance.joint.seats)
					if (seat.lower.member==bindings[0].member || seat.upper.member==bindings[0].member) found=true;
			}
			if (!found) missing.push(insert.id);
		}
		if (missing.length==0) return null;
		missing.sort(Reflect.compare);
		return ToolingReadiness.pending("tooling.foundation-joint.source",missing,
			"Declare the assembly foundation joint for these source mounting interfaces, including gap, jacks, anchor holes and cure-before-final-torque: "+missing.join(", "));
	}
	public static function analyze(input:CastingToolingInput,processes:Array<CastingMoldPlan.FoundationJointProcess>):Array<ToolingValidation> {
		var context=input.interfaceContext,result:Array<ToolingValidation> = [];
		for (process in processes) if (context==null || context.foundationJoints.filter(j -> j.id==process.jointId).length!=1)
			throw "Unknown source foundation joint process: "+process.jointId;
		if (context==null) return result;
		for (instance in context.foundationJoints) {
			var prefix="tooling.foundation-joint."+haxe.crypto.Sha256.encode(instance.id),ids=[input.id];
			for (finding in FoundationJointAnalysis.check(context.jointSources,instance.joint)) {
				var status=finding.passed ? ToolingCheckStatus.Pass : ToolingCheckStatus.Fail;
				result.push({rule:prefix+"."+finding.code+"."+finding.subject,affectedIds:ids.copy(),status:status,
					severity:finding.passed ? ToolingSeverity.Info : ToolingSeverity.Error,message:instance.id+": "+finding.message,evidence:[]});
			}
			result.push({rule:prefix+".contact-and-restraint",affectedIds:ids.copy(),status:ToolingCheckStatus.Unsupported,severity:ToolingSeverity.Error,
				message:instance.id+": finite joint contact, jack restraint and the complete anchor/grout stack are not geometrically qualified.",evidence:[]});
			var choices=processes.filter(process -> process.jointId==instance.id);
			result.push(ToolingReadiness.pending(prefix+".process",ids,instance.id+": "+(choices.length==0
				? "Declare the joint compound and bind its cure-before-final-torque evidence to the actual joint."
				: "Joint process references are recorded; qualify the compound, cure and final torque against the actual joint geometry.")));
		}
		result.sort((a,b) -> Reflect.compare(a.rule,b.rule));return result;
	}
	public static function source(input:CastingToolingInput):Array<ToolingValidation> {
		var result=analyze(input,[]),missing=missingSource(input);
		if (missing!=null) result.push(missing);
		result.sort((a,b) -> Reflect.compare(a.rule,b.rule));return result;
	}
	public static function validate(input:CastingToolingInput,processes:Array<CastingMoldPlan.FoundationJointProcess>,actual:Array<ToolingValidation>):Void {
		var expected=analyze(input,processes),missing=missingSource(input);
		if (missing!=null) expected.push(missing);
		expected.sort((a,b) -> Reflect.compare(a.rule,b.rule));
		var stored=actual.filter(check -> StringTools.startsWith(check.rule,"tooling.foundation-joint."));
		stored.sort((a,b) -> Reflect.compare(a.rule,b.rule));
		if (materia.tooling.ToolingCodec.canonical(expected)!=materia.tooling.ToolingCodec.canonical(stored))
			throw "Foundation joint validation differs from source and process choices";
	}
}
