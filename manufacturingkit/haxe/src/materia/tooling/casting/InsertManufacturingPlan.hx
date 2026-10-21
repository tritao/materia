package materia.tooling.casting;

import materia.casting.InsertManufacturing;
import materia.casting.InsertManufacturing.InsertManufacturingCodec;
import materia.casting.InsertManufacturing.InsertInspection;
import materia.casting.InsertManufacturing.InsertHoleRoute;
import materia.casting.InterfaceRequirement.InterfaceRoute;
import materia.tooling.casting.CastingToolingInput.CastingToolingInsert;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;

enum abstract InsertOperationKind(String) to String {
	var CutStock = "cut-stock";
	var MachineBlank = "machine-blank";
	var Drill = "drill";
	var TransferDrill = "transfer-drill";
	var Tap = "tap";
	var Deburr = "deburr";
	var Clean = "clean";
	var PrepareBondSurface = "prepare-bond-surface";
	var Inspect = "inspect";
	var MachineInterface = "machine-interface";
	var GrindInterface = "grind-interface";
	var ReplicateInterface = "replicate-interface";
	var AdjustInterface = "adjust-interface";
}

enum abstract InsertOperationStage(String) to String {
	var BeforeCasting = "before-casting";
	var AfterCasting = "after-casting";
}

enum abstract InsertInspectionStage(String) to String {
	var BeforeCasting = "before-casting";
	var AfterCasting = "after-casting";
	var Undecided = "undecided";
}

enum abstract InsertInspectionKind(String) to String {
	var MaterialIdentity = "material-identity";
	var BlankDimensions = "blank-dimensions";
	var HolePosition = "hole-position";
	var ThreadGauge = "thread-gauge";
	var FaceFlatness = "face-flatness";
	var BondSurface = "bond-surface";
}

typedef InsertInspectionRequirement = {
	/** A checklist key scoped to this insert, not a global tooling member ID. */
	var key:String;
	var kind:InsertInspectionKind;
	var sourceIds:Array<String>;
	var stage:InsertInspectionStage;
	var declared:Bool;
	/** Null means the inspection's limit is not declared or is non-numerical.
	 * A declared dimensional inspection always has a strictly positive limit.
	 */
	var toleranceMm:Null<Float>;
}

typedef InsertOperationRecord = {
	var id:String;
	var kind:InsertOperationKind;
	var stage:InsertOperationStage;
	var sourceIds:Array<String>;
}

typedef InsertManufacturingJob = {
	var insertId:String;
	var specification:InsertManufacturing;
	var operations:Array<InsertOperationRecord>;
	var inspectionRequirements:Array<InsertInspectionRequirement>;
	var validation:Array<ToolingValidation>;
}

/** Source choices generate the shop sequence. An undecided hole produces no
 * drilling operation; neither geometry nor a fixture screw decides its route.
 */
class InsertManufacturingPlan {
	public static function build(insert:CastingToolingInsert):InsertManufacturingJob {
		var spec=switch insert.origin {
			case Fabricated(manufacturing):InsertManufacturingCodec.snapshot(manufacturing);
			case Catalogue(_):throw "A catalogue insert is not a fabricated manufacturing job: "+insert.id;
		};
		InsertManufacturingCodec.references(spec,insert.holes,insert.interfaces);
		if (spec.material.grade!=insert.material.grade) throw "Insert manufacturing grade differs from source stock";
		var operations:Array<InsertOperationRecord> = [],missing:Array<String> = [];
		function add(suffix:String,kind:InsertOperationKind,stage:InsertOperationStage,ids:Array<String>):Void
			operations.push({id:materia.tooling.ToolingIds.derive("make-",insert.id,"-"+suffix),kind:kind,stage:stage,sourceIds:ids});
		add("cut",CutStock,InsertOperationStage.BeforeCasting,[insert.id]);
		add("blank",MachineBlank,InsertOperationStage.BeforeCasting,[insert.id]);
		if (spec.processReference==null) missing.push("shop process, including workholding and surface preparation");
		var holes=insert.holes.copy();holes.sort((a,b) -> Reflect.compare(a.id,b.id));
		for (hole in holes) {
			var routes=spec.holeRoutes.filter(choice -> choice.holeId==hole.id);
			if (routes.length==0) {missing.push("hole route "+hole.id);continue;}
			var stage=routes[0].route==InsertHoleRoute.BeforeCasting ? InsertOperationStage.BeforeCasting : InsertOperationStage.AfterCasting;
			var suffix=haxe.crypto.Sha256.encode(hole.id).substr(0,12);
			add("drill-"+suffix,stage==InsertOperationStage.BeforeCasting ? Drill : TransferDrill,stage,[hole.id]);
			if (hole.thread!=null) add("tap-"+suffix,Tap,stage,[hole.id]);
		}
		var requirements=insert.interfaces.copy();requirements.sort((a,b) -> Reflect.compare(a.id,b.id));
		for (requirement in requirements) {
			var kind:Null<InsertOperationKind> = switch requirement.route {
				case InterfaceRoute.Machined:MachineInterface;
				case InterfaceRoute.Ground:GrindInterface;
				case InterfaceRoute.Replicated:ReplicateInterface;
				case InterfaceRoute.AdjustedAtAssembly:AdjustInterface;
				case InterfaceRoute.AsSupplied,InterfaceRoute.AsCast:null;
				default:throw "Unknown source finishing route";
			};
			if (kind!=null) add("finish-"+haxe.crypto.Sha256.encode(requirement.id).substr(0,12),kind,InsertOperationStage.AfterCasting,[requirement.id]);
		}
		add("deburr",Deburr,InsertOperationStage.BeforeCasting,[insert.id]);
		add("clean",Clean,InsertOperationStage.BeforeCasting,[insert.id]);
		add("bond",PrepareBondSurface,InsertOperationStage.BeforeCasting,[insert.id]);
		add("inspect",Inspect,InsertOperationStage.BeforeCasting,[insert.id]);
		var finishing=operations.filter(operation -> operation.kind==MachineInterface || operation.kind==GrindInterface
			|| operation.kind==ReplicateInterface || operation.kind==AdjustInterface);
		operations=operations.filter(operation -> operation.stage==InsertOperationStage.BeforeCasting).concat(finishing).concat(
			operations.filter(operation -> operation.stage==InsertOperationStage.AfterCasting && finishing.indexOf(operation)<0));
		var hasPostCastOperations=operations.filter(operation -> operation.stage==InsertOperationStage.AfterCasting).length>0;
		if (hasPostCastOperations) {
			add("post-deburr",Deburr,InsertOperationStage.AfterCasting,[insert.id]);
			add("post-clean",Clean,InsertOperationStage.AfterCasting,[insert.id]);
			add("post-inspect",Inspect,InsertOperationStage.AfterCasting,[insert.id]);
		}
		var inspections=inspectionRequirements(spec,insert);
		for (inspection in inspections) if (!inspection.declared) missing.push("inspection "+inspection.key);
		var validation:Array<ToolingValidation> = [];
		var fit=materia.casting.InsertBlankFit.assess(insert.body,spec.blank),blankRule="fabrication.insert.blank."+insert.id;
		switch fit {
			case Unselected:validation.push(ToolingReadiness.pending(blankRule,[insert.id],"Select the insert's actual blank stock and its part-to-stock frame."));
			case Fits:validation.push(check(blankRule,insert.id,ToolingCheckStatus.Pass,"Nominal blank contains the raw insert; stock receipt and inspection remain separate."));
			case Invalid(reason):validation.push(check(blankRule,insert.id,ToolingCheckStatus.Fail,reason));
			case Unsupported(reason):validation.push(check(blankRule,insert.id,ToolingCheckStatus.Unsupported,reason));
		}
		var planRule="fabrication.insert.plan."+insert.id;
		validation.push(missing.length==0 ? check(planRule,insert.id,ToolingCheckStatus.Pass,"Manufacturing routes, process reference and inspection limits are declared.")
			: ToolingReadiness.pending(planRule,[insert.id],"Declare "+missing.join("; ")+"."));
		validation.push(ToolingReadiness.pending("fabrication.insert.inspection."+insert.id,[insert.id],
			"Execute and record the pre-cast inspections before the insert enters the mold; a planned checklist is not a result."));
		if (hasPostCastOperations) validation.push(ToolingReadiness.pending("fabrication.insert.final-inspection."+insert.id,[insert.id],
			"Execute and record inspection after post-cast finishing and drilling before service assembly."));
		var unresolved=holes.filter(hole -> spec.holeRoutes.filter(choice -> choice.holeId==hole.id).length==0);
		var geometryRule="fabrication.insert.hole-geometry."+insert.id;
		validation.push(unresolved.length>0 ? ToolingReadiness.pending(geometryRule,[insert.id],
			"Declare the manufacturing stage of source holes before qualifying pre-cast insert geometry.")
			: check(geometryRule,insert.id,ToolingCheckStatus.Pass,
				"Pre-cast geometry uses only the holes declared before casting; service geometry includes the final hole set."));
		validation.push(finishGeometry(insert));
		return {insertId:insert.id,specification:spec,operations:operations,inspectionRequirements:inspections,validation:validation};
	}

	static function finishGeometry(insert:CastingToolingInsert):ToolingValidation {
		var rule="fabrication.insert.finish-geometry."+insert.id;
		if (insert.interfaces.length==0 && insert.finish!=materia.casting.InsertSpec.InsertFinish.AsSupplied
			&& insert.finish!=materia.casting.InsertSpec.InsertFinish.SourceAsCast)
			return ToolingReadiness.pending(rule,[insert.id],"Declare finished interfaces before qualifying service body geometry.");
		return switch materia.casting.InsertFinishing.assess(insert.body,insert.interfaces) {
			case Invalid(issues):check(rule,insert.id,ToolingCheckStatus.Fail,
				[for (issue in issues) issue.interfaceIds.join(", ")+": "+issue.message].join("; "));
			case Unsupported(issues):check(rule,insert.id,ToolingCheckStatus.Unsupported,
				[for (issue in issues) issue.interfaceIds.join(", ")+": "+issue.message].join("; "));
			case Resolved(steel):
				var error:Null<String> = null,box=materia.casting.InsertFinishing.box(steel);
				try {
					if (box!=null && steel.removedSteel.length>0) for (hole in insert.holes) materia.casting.InsertStages.validateServiceHole(hole,box);
					InsertFixturePlanner.holes(insert);
				} catch (failure:Dynamic) error=Std.string(failure);
				if (error!=null) check(rule,insert.id,ToolingCheckStatus.Fail,error);
				else if (steel.replicatedLayers.length>0) check(rule,insert.id,ToolingCheckStatus.Unsupported,
					[for (layer in steel.replicatedLayers) layer.interfaceId].join(", ")
					+": replicated service surface still needs its material-layer realization");
				else check(rule,insert.id,ToolingCheckStatus.Pass,
					"Finished steel and drilling derive from the declared stock and source interfaces; process and inspection remain separate.");
		};
	}

	static function inspectionRequirements(spec:InsertManufacturing,insert:CastingToolingInsert):Array<InsertInspectionRequirement> {
		var result:Array<InsertInspectionRequirement> = [];
		function add(kind:InsertInspectionKind,ids:Array<String>,stage:InsertInspectionStage):Void {
			var key=(kind:String)+(ids.length==0 ? "" : "/"+ids[0]);
			var declarations=spec.inspections.filter(inspection -> InsertManufacturingCodec.inspectionKey(inspection)==key);
			var tolerance:Null<Float> = null;
			if (declarations.length==1) tolerance=switch declarations[0] {
				case BlankDimensions(value),HolePosition(_,value),FaceFlatness(_,value):value;
				case MaterialIdentity,ThreadGauge(_),BondSurface:null;
			};
			result.push({key:key,kind:kind,sourceIds:ids,stage:stage,declared:declarations.length==1,toleranceMm:tolerance});
		}
		for (kind in [InsertInspectionKind.MaterialIdentity,InsertInspectionKind.BlankDimensions,InsertInspectionKind.BondSurface])
			add(kind,[],InsertInspectionStage.BeforeCasting);
		var holes=insert.holes.copy();holes.sort((a,b) -> Reflect.compare(a.id,b.id));
		for (hole in holes) {
			var route=spec.holeRoutes.filter(choice -> choice.holeId==hole.id);
			var stage=route.length==0 ? InsertInspectionStage.Undecided : route[0].route==InsertHoleRoute.BeforeCasting
				? InsertInspectionStage.BeforeCasting : InsertInspectionStage.AfterCasting;
			add(InsertInspectionKind.HolePosition,[hole.id],stage);
			if (hole.thread!=null) add(InsertInspectionKind.ThreadGauge,[hole.id],stage);
		}
		var interfaces=insert.interfaces.copy();interfaces.sort((a,b) -> Reflect.compare(a.id,b.id));
		for (requirement in interfaces) switch requirement.geometry {
			case Plane(_):add(InsertInspectionKind.FaceFlatness,[requirement.id],InsertInspectionStage.BeforeCasting);
			case Line(_):
		}
		return result;
	}

	static function check(rule:String,id:String,status:ToolingCheckStatus,message:String):ToolingValidation
		return {rule:rule,affectedIds:[id],status:status,severity:status==ToolingCheckStatus.Pass ? ToolingSeverity.Info : ToolingSeverity.Error,
			message:message,evidence:[]};
}
