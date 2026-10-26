import machinekit.structural.MineralFoundation;
import machinekit.tooling.FoundationToolingAdapter;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingToolingInput;

enum abstract HobberMoldCasting(String) {
 var Plinth="plinth";
 var Column="column";
}

/** Hobber geometry and orientation only; workshop choices are shared. */
class HobberMoldSetup {
 public static function document(foundation:MineralFoundation,casting:HobberMoldCasting=HobberMoldCasting.Plinth):ToolingDocument {
  var sourceId="hobber-"+casting;
  var input=FoundationToolingAdapter.snapshot(foundation,sourceId,1);
  var recipe=CastingShopRecipe.tooling(sourceId+"-panel-eps-review");
  // Pour mounting faces toward the bed: the existing source threads are
  // reached through dry datum posts, rather than submerged cross-carriers.
  recipe.coreWireAxis=materia.tooling.ToolingRecipe.CoreWireAxis.Z;
  // The catalogue sockets have a small circular sealing land, qualified in
  // the existing hobber socket fixture coupon rather than a mill plate land.
  recipe.hardware.sealMargin=0.5;
  // The small catalogue sockets use the bed as their shared source-thread
  // carrier. Raised faces still receive the normal separate datum supports.
  recipe.hardware.fixtureReceiver=materia.tooling.ToolingRecipe.FixtureReceiver.PanelWhenCoplanar;
  var document:ToolingDocument={input:input,recipe:recipe,setup:{
   id:"plinth-mounting-faces-down",revision:1,sourceFingerprint:ToolingCodec.inputFingerprint(input),
   fillHeightMm:foundation.body.parameterNumber("height"),freeboardMm:0,
   sourceToMold:F.fromRotationMatrix(0,0,foundation.body.parameterNumber("height"),
    [1,0,0,0,-1,0,0,0,-1])
  }};
  if(casting==HobberMoldCasting.Column){
   var depth=foundation.body.parameterNumber("depth");
   document.setup.id="column-rail-faces-down";
   document.setup.fillHeightMm=depth;
   document.setup.sourceToMold=F.fromRotationMatrix(0,0,depth/2,[1,0,0,0,0,1,0,-1,0]);
   // The rail faces lie on source +Y; the underside core openings run along
   // mold Y in this orientation rather than along the plinth's mold Z.
   for(rule in recipe.featureRules)
    if(rule.kind==materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening)
     rule.preferredAxis=materia.tooling.casting.CastingStrategies.WithdrawalAxis.Y;
   return document;
  }
  return document;
 }

 public static function options(input:CastingToolingInput):CastingMoldOptions return {
  foundationJointProcesses:[],replicationJigs:[],
  precision:[for(insert in input.inserts){
   insertId:insert.id,jigReference:null,masterReference:null,masterFlatnessMm:null,
   releaseSpec:null,holeProtectionReference:null,evidence:[]
  }],
  compaction:CastingShopRecipe.compaction(),serviceOrientation:F.identity(),
  overlapRule:ReservedThenPriority,liftingEvidence:[]
 };
}
