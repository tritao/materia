package machinekit.tooling;

import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.casting.MoldVolumeCodec;
import machinekit.tooling.ToolingSectionDrawing.ToolingSectionSheet;

typedef ToolingSectionRecord = {
	var id:String;
	var geometryFingerprint:String;
	var sections:Array<materia.casting.CastingSection>;
	var validation:ToolingValidation;
}

typedef ToolingSectionReviewResult = {
	var records:Array<ToolingSectionRecord>;
	var sheets:Array<{id:String,sheet:ToolingSectionSheet}>;
	var validation:Array<ToolingValidation>;
}

/** Finished-tooling drawing evidence, separate from machining and physical fit. */
class ToolingSectionReview {
	public static function build(plan:CastingMoldPlan):ToolingSectionReviewResult {
		var items=[for (former in plan.formers) {volume:former.volume,direction:former.withdrawal,ids:[former.volume.id].concat(former.featureIds)}];
		for (fixture in plan.fixtures) items.push({volume:fixture.datum,direction:fixture.withdrawal,ids:[fixture.datum.id,fixture.insertId]});
		items.sort((a,b) -> Reflect.compare(a.volume.id,b.volume.id));
		var result:ToolingSectionReviewResult={records:[],sheets:[],validation:[]};
		for (item in items) {
			var id=item.volume.id,rule="fabrication.tooling-drawing."+id;
			var status:ToolingCheckStatus=Pass,message="Exact nominal sections derive from finished tooling geometry.";
			var sections:Array<materia.casting.CastingSection> =[];
			try {
				var sheet=ToolingSectionDrawing.byDirection(item.volume,item.direction);
				sections=sheet.sections;
				result.sheets.push({id:id,sheet:sheet});
			} catch (error:Dynamic) {
				message=Std.string(error);
				if (message.indexOf("Unsupported section curve")<0
					&& message.indexOf("Ambiguous section junction")<0) throw error;
				status=Unsupported;
			}
			var ids:Array<String> =[];
			for (source in item.ids) if (ids.indexOf(source)<0) ids.push(source);
			ids.sort(Reflect.compare);
			var evidence=status==Pass ? [filename(id)+".svg",filename(id)+".dxf"] : [];
			var check=materia.tooling.process.FillAnalysis.check(rule,ids,status,message,evidence);
			result.validation.push(check);
			result.records.push({id:id,geometryFingerprint:haxe.crypto.Sha256.encode(MoldVolumeCodec.encode(item.volume)),
				sections:sections,validation:check});
		}
		return result;
	}

	public static function record(review:ToolingSectionReviewResult,sourceFingerprint:String):Dynamic
		return {format:"materia.tooling.section-drawings",version:1,sourceFingerprint:sourceFingerprint,
			drawings:review.records,validation:review.validation};

	public static function filename(id:String):String return materia.tooling.ToolingIds.derive("volume-",id);
}
