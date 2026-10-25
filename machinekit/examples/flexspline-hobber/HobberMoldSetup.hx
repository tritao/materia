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
 public static function document(foundation:MineralFoundation,casting:HobberMoldCasting=Plinth):ToolingDocument {
  var sourceId="hobber-"+casting;
  var input=FoundationToolingAdapter.snapshot(foundation,sourceId,1);
  var recipe=CastingShopRecipe.tooling(sourceId+"-panel-eps-review");
  // Pour mounting faces toward the bed: the existing source threads are
  // reached through dry datum posts, rather than submerged cross-carriers.
  // Source lower struts become the upper supports in this orientation.
  recipe.coreWireAxis=materia.tooling.ToolingRecipe.CoreWireAxis.Z;
  // The catalogue sockets have a small circular sealing land, qualified in
  // the existing hobber socket fixture coupon rather than a mill plate land.
  recipe.hardware.sealMargin=0.5;
  // Size datum posts for the selected source clearance hole plus the same
  // pin pair and edge ligaments used by DatumFixtures; no insert-specific sizes.
  var h=recipe.hardware,pin=h.fixturePinDiameter+recipe.jointClearance;
  for(insert in input.inserts){
   var face=F.compose(insert.pose,F.translation(0,0,materia.casting.InsertSpec.InsertBodyCodec.height(insert.body)));
   for(hole in materia.tooling.casting.InsertFixturePlanner.availableHoles(insert)){
    var fixing=materia.tooling.casting.InsertFixturePlanner.select(hole,insert.pose,face,h);
    if(fixing==null)continue;
    var spacing=Math.max(h.fixturePinSpacing,fixing.clearanceDiameter+pin+h.secondaryPinTravel+2*recipe.minimumLigament);
    h.datumPostWidth=Math.max(h.datumPostWidth,spacing+pin+h.secondaryPinTravel+2*recipe.minimumLigament);
   }
  }
  var document:ToolingDocument={input:input,recipe:recipe,setup:{
   id:"plinth-mounting-faces-down",revision:1,sourceFingerprint:ToolingCodec.inputFingerprint(input),
   fillHeightMm:foundation.body.parameterNumber("height"),freeboardMm:0,
   sourceToMold:F.fromRotationMatrix(0,0,foundation.body.parameterNumber("height"),
    [1,0,0,0,-1,0,0,0,-1]),features:[]
  }};
  if(casting==Column){
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
  // The removable rear bridge's specific register owns its overlap with the
  // general tray ledge. Keep that casting-specific decision out of the shop.
  var bridge=materia.tooling.casting.CastingStrategies.resolve(document)
   .filter(strategy->strategy.featureId=="tray-rear-bridge-seat");
  if(bridge.length!=1)throw "Hobber plinth is missing its rear bridge register";
  bridge[0].priority=10;
  document.setup.features=[bridge[0]];
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
