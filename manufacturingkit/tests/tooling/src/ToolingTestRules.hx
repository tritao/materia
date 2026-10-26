import materia.tooling.ToolingDocument;
import materia.tooling.casting.CastingSetup.CoreStrategy;
import materia.tooling.casting.CastingStrategies;
import materia.tooling.casting.CastingStrategies.StrategyOpening;
import materia.tooling.casting.CastingStrategies.WithdrawalAxis;
import materia.casting.CastingFeature.CastingFeatureKind;
import materia.assembly.AssemblyFrames as F;

/** Tests author recipe rules and source axes, never obsolete setup overrides. */
class ToolingTestRules {
 public static function set(d:ToolingDocument,id:String,strategy:CoreStrategy,priority:Int=0,
   ?direction:{x:Float,y:Float,z:Float}):Void {
  var feature=d.input.features.filter(feature->feature.id==id)[0];
  if(feature==null || feature.kind==InsertRecess)throw "Invalid test source feature: "+id;
  var opening=feature.openings.filter(o->o.kind==BodyFace).length==0 ? Enclosed : BodyOpening;
  var rule=d.recipe.featureRules.filter(rule->rule.kind==feature.kind && rule.opening==opening)[0];
  rule.strategy=strategy;rule.priority=priority;
  if(direction!=null) {
   var axis=F.transformVector(F.inverse(d.setup.sourceToMold),direction.x,direction.y,direction.z);
   feature.draft.axis=axis;
   rule.preferredAxis=Math.abs(axis.x)>0.9 ? X : Math.abs(axis.y)>0.9 ? Y : Z;
  }
  d.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(d.input);
 }
 public static function first(d:ToolingDocument,strategy:CoreStrategy,?direction:{x:Float,y:Float,z:Float}):Void {
  set(d,d.input.features.filter(feature->feature.kind!=InsertRecess)[0].id,strategy,0,direction);
 }
}
