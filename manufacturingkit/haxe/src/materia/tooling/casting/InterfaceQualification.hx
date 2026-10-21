package materia.tooling.casting;

import materia.casting.InterfaceRelationship;
import materia.casting.InterfaceRelationship.InterfaceRelationships;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingValidation.ToolingReadiness;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;

/** A measurement-chain budget check, independent of measured geometry and
 * instrument acceptance. Passing this rule does not qualify a precision interface.
 */
class InterfaceQualification {
	/** Geometry-only decision. A pass does not qualify hole finishing, the selected
	 * machining process, bonding, a replication dam or physical inspection.
	 */
	public static function finishingStock(input:CastingToolingInput,insert:CastingToolingInput.CastingToolingInsert):ToolingValidation {
		var rule="tooling.precision.allowance."+insert.id;
		if (insert.interfaces.length==0) {
			if (insert.finish==materia.casting.InsertSpec.InsertFinish.AsSupplied
				|| insert.finish==materia.casting.InsertSpec.InsertFinish.SourceAsCast)
				return stockCheck(rule,insert.id,ToolingCheckStatus.Pass,"The source finish declares no added or removed finishing layer.");
			return ToolingReadiness.pending(rule,[insert.id],"Declare source interface requirements and include their finishing stock or replication gap in source geometry.");
		}
		var failures:Array<String> = [],unsupported:Array<String> = [];
		var layers:Array<{interfaceId:String,region:materia.casting.Box3}> = [];
		switch materia.casting.InsertFinishing.assess(insert.body,insert.interfaces) {
			case Invalid(issues):for (issue in issues) failures.push(issue.interfaceIds.join(", ")+": "+issue.message);
			case Unsupported(issues):for (issue in issues) unsupported.push(issue.interfaceIds.join(", ")+": "+issue.message);
			case Resolved(steel):layers=steel.replicatedLayers;
		}
		for (entry in layers) {
			var layer=entry.region,requirementId=entry.interfaceId;
			var support=CastingBoxSupport.check(input,"tooling.precision.gap-geometry");
			var unavailable=support.filter(check -> check.status==ToolingCheckStatus.Unsupported);
			if (unavailable.length>0) {
				unsupported.push(requirementId+": "+[for (check in unavailable) check.message].join("; "));
				continue;
			}
			var region=materia.casting.CastingBounds.box({
				solid:materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(layer.width,layer.depth,layer.height)),
				pose:materia.assembly.AssemblyFrames.compose(insert.pose,materia.assembly.AssemblyFrames.translation(
					layer.x+layer.width/2,layer.y+layer.depth/2,layer.z))
			});
			var material=materia.casting.BoxAlgebra.material(input.envelope,input.features);
			if (material.filter(box -> materia.casting.BoxAlgebra.intersection(box,region)!=null).length>0)
				failures.push(requirementId+": replication gap is occupied by casting material; reserve it in the source geometry");
			for (other in input.inserts) if (other.id!=insert.id) {
				var bounds=materia.casting.CastingBounds.box({
					solid:materia.casting.CastingFeature.CastingGeometrySolid.Insert(other.body),pose:other.pose});
				if (materia.casting.BoxAlgebra.intersection(bounds,region)!=null) {
					if (InsertFixturePlanner.holes(other).length==0) failures.push(requirementId+": replication gap is occupied by insert "+other.id);
					else unsupported.push(requirementId+": exact replication-gap contact with perforated insert "+other.id+" requires native qualification");
				}
			}
		}
		if (failures.length>0) return stockCheck(rule,insert.id,ToolingCheckStatus.Fail,failures.join("; "));
		if (unsupported.length>0) return stockCheck(rule,insert.id,ToolingCheckStatus.Unsupported,unsupported.join("; "));
		return stockCheck(rule,insert.id,ToolingCheckStatus.Pass,
			"Declared nominal finishing layers match the insert geometry and replication gaps are reserved; process, hole-route and inspection qualification are separate.");
	}

	/** Process prerequisites follow the source interface routes. An identified
	 * master/release does not establish material, metrology or physical evidence.
	 */
	public static function replicationRequirement(source:CastingToolingInput.CastingToolingInsert,
		process:CastingMoldPlan.PrecisionInterface):Null<ToolingValidation> {
		var required=source.interfaces.length==0
			? source.finish==materia.casting.InsertSpec.InsertFinish.ReplicatedAfterCasting
			: source.interfaces.filter(requirement -> requirement.route==materia.casting.InterfaceRequirement.InterfaceRoute.Replicated).length>0;
		if (!required) return null;
		if (process.masterReference!=null && StringTools.trim(process.masterReference).length>0
			&& process.releaseSpec!=null && StringTools.trim(process.releaseSpec).length>0) return null;
		return ToolingReadiness.pending("tooling.precision.replication."+source.id,[source.id],
			"The source replication route requires an identified master and release specification.");
	}

	static function stockCheck(rule:String,id:String,status:ToolingCheckStatus,message:String):ToolingValidation
		return {rule:rule,affectedIds:[id],status:status,
			severity:status==ToolingCheckStatus.Pass ? ToolingSeverity.Info : ToolingSeverity.Error,message:message,evidence:[]};

	public static function fixtureBudget(requirement:materia.casting.InterfaceRequirement,owner:String):ToolingValidation {
		var remaining=materia.casting.InterfaceRequirement.InterfaceRequirements.fixtureBudgetMm(requirement);
		var rule="tooling.fixture.budget."+owner+"."+requirement.id;
		if (remaining==null) return ToolingReadiness.pending(rule,[owner],
			"Declare the source finishing-process error allocation; geometric finishing stock is not an error budget.");
		if (remaining<=0) return stockCheck(rule,owner,ToolingCheckStatus.Fail,
			"Finishing-process error exhausts the interface tolerance; no fixture/casting/metrology budget remains.");
		return ToolingReadiness.pending(rule,[owner],"Remaining combined fixture/casting/metrology budget: "+remaining
			+" mm. Allocate and qualify these contributions; finishing stock changes geometry only.");
	}

	public static function fixtureBudgets(input:CastingToolingInput):Array<ToolingValidation> {
		var checks:Array<ToolingValidation> = [];
		for (requirement in input.interfaces) checks.push(fixtureBudget(requirement,input.id));
		for (insert in input.inserts) for (requirement in insert.interfaces)
			checks.push(fixtureBudget(requirement,insert.id));
		return checks;
	}

	public static function relationships(input:CastingToolingInput):Array<ToolingValidation> {
		var context=input.interfaceContext;
		if (context==null) {
			var required=input.interfaces.filter(requirement -> requirement.relationshipIds.length>0).length>0;
			for (insert in input.inserts)
				if (insert.interfaces.filter(requirement -> requirement.relationshipIds.length>0).length>0) required=true;
			return required ? [ToolingReadiness.pending("tooling.relationship.context."+input.id,[input.id],
				"Source interfaces reference machine relationships, but their assembly context has not been supplied.")] : [];
		}
		var checks:Array<ToolingValidation> = [];
		for (instance in context.relationships) {
			var key=haxe.crypto.Sha256.encode(instance.id);
			var affected=[input.id];
			for (reference in instance.relationship.members) switch reference {
				case ComponentInterface(part,_):
					for (binding in context.inserts)
						if (binding.member==part && affected.indexOf(binding.id)<0) affected.push(binding.id);
				case MachineAxis(_):
			}
			affected.sort(Reflect.compare);
			var budget=metrologyBudget(instance.relationship,affected);
			budget.rule="tooling.relationship.metrology-budget."+key;
			budget.message=instance.id+": "+budget.message;
			checks.push(budget);
			checks.push(ToolingReadiness.pending("tooling.relationship.qualification."+key,affected,
				instance.id+": nominal relationship geometry, the chosen jig/masters and measured alignment require qualification."));
		}
		return checks;
	}

	public static function foundationDatums(input:CastingToolingInput):Array<ToolingValidation> {
		var scheme = input.datumScheme;
		if (scheme == null) return [ToolingReadiness.pending("tooling.datum.scheme." + input.id,[input.id],
			"The source has no primary/secondary/tertiary datum scheme; fixtures and masters cannot be qualified.")];
		var unresolved = materia.casting.FoundationDatumScheme.FoundationDatums.validateSource(scheme,input.envelope,input.features,
			[for (insert in input.inserts) {id:insert.id,pose:insert.pose,holes:insert.holes,interfaces:insert.interfaces}],input.interfaces);
		var checks:Array<ToolingValidation> = [];
		for (datum in unresolved) checks.push({
			rule: "tooling.datum.reference." + datum.datumId,
			affectedIds: [input.id,datum.featureId],status: ToolingCheckStatus.Unsupported,severity: ToolingSeverity.Error,
			message: datum.reason + ": " + datum.featureId,evidence: []
		});
		if (unresolved.length == 0) checks.push({
			rule:"tooling.datum.references." + input.id,affectedIds:[input.id],
			status:ToolingCheckStatus.Pass,severity:ToolingSeverity.Info,
			message:"Declared datum and interface IDs resolve to supported source boundary records; contact geometry and accuracy are separate checks.",evidence:[]
		});
		checks.push(ToolingReadiness.pending("tooling.datum.contacts." + input.id,[input.id],
			"Datum contact patches, source frames, fixture constraints and master references require native qualification."));
		return checks;
	}

	public static function metrologyBudget(relationship:InterfaceRelationship, affectedIds:Array<String>):ToolingValidation {
		var r = InterfaceRelationships.snapshot([relationship])[0];
		if (affectedIds == null || affectedIds.length == 0) throw "Metrology check requires affected source IDs";
		for (id in affectedIds) materia.tooling.ToolingIds.validate(id);
		var margin = InterfaceRelationships.uncertaintyMargin(r);
		var fitsBudget = margin != null && margin >= 0 && r.metrology.instrumentReference != null;
		var message = if (r.metrology.instrumentReference == null)
			"Select the relationship's measurement instrument; uncertainty budget remains pending."
		else if (margin == null)
			"The instrument's measurement uncertainty has not been declared."
		else if (margin < 0)
			"Declared measurement uncertainty exceeds the relationship tolerance; measurement remains pending."
		else
			"Declared measurement uncertainty fits the relationship tolerance; instrument acceptance and measured geometry are separate checks.";
		return {
			rule: "tooling.relationship.metrology-budget." + r.id,
			affectedIds: affectedIds.copy(),
			status: fitsBudget ? ToolingCheckStatus.Pass : ToolingCheckStatus.Pending,
			severity: fitsBudget ? ToolingSeverity.Info : ToolingSeverity.Warning,
			message: message,
			evidence: []
		};
	}
	public static function source(input:CastingToolingInput):Array<ToolingValidation>
		return foundationDatums(input).concat(relationships(input)).concat(fixtureBudgets(input));

	public static function validate(input:CastingToolingInput,checks:Array<ToolingValidation>):Void {
		for (expected in source(input)) {
			var found=checks.filter(check -> check.rule==expected.rule);
			if (found.length!=1 || materia.tooling.ToolingCodec.canonical(found[0])!=materia.tooling.ToolingCodec.canonical(expected))
				throw "Source interface validation differs from source: "+expected.rule;
		}
	}

}
