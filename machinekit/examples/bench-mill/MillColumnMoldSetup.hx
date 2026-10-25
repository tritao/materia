import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingDocument;
import materia.tooling.casting.CastingToolingInput;
import materia.tooling.casting.CastingMoldPlan;
import machinekit.tooling.FoundationToolingAdapter;

/** Source geometry stays in BenchMillFrame; this chooses tooling orientation and recipe. */
class MillColumnMoldSetup {
	public static function input(frame:BenchMillFrame):CastingToolingInput {
		var context=machinekit.tooling.FoundationAssemblyContext.snapshot(frame,"column/casting",
			[for (insert in frame.column.insertLocations()) {id:insert.id,member:"column/"+insert.id}]);
		return FoundationToolingAdapter.snapshot(frame.column,"bench-mill-column",1,context);
	}

	public static function document(frame:BenchMillFrame,thickness:Float=CastingShopRecipe.BOARD_THICKNESS_MM):ToolingDocument {
		var document=MillMoldSetup.document(frame,thickness),source=input(frame);
		var depth=frame.layout.columnDepth;
		document.input=source;
		document.setup={id:"column-front-on-bed",revision:1,sourceFingerprint:ToolingCodec.inputFingerprint(source),
			fillHeightMm:depth,freeboardMm:0,
			sourceToMold:F.fromRotationMatrix(0,0,depth/2,[1,0,0,0,0,-1,0,1,0]),features:[]};
		document.recipe.id="mill-column-panel-eps-review";
		// Front-on-bed column orientation puts the wire through mold Z rather than base mold Y.
		document.recipe.coreWireAxis=materia.tooling.ToolingRecipe.CoreWireAxis.Z;
		// Side-open column service passages withdraw along mold Y.
		for (rule in document.recipe.featureRules)
			if (rule.kind==materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening)
				rule.preferredAxis=materia.tooling.casting.CastingStrategies.WithdrawalAxis.Y;
		return document;
	}

	public static function options(frame:BenchMillFrame):CastingMoldOptions {
		var choices=MillMoldSetup.options(frame);
		choices.precision=[for (insert in input(frame).inserts) {
			insertId:insert.id,jigReference:null,masterReference:null,masterFlatnessMm:null,
			releaseSpec:null,holeProtectionReference:null,evidence:[]
		}];
		return choices;
	}
}
