package machinekit.motion;

import cadkit.Shape;
import cadkit.modeling.*;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Detailed pinned HGH/HGW carriage references with generic named rail/mount interfaces.
 * No network access or fallback envelope during normal geometry generation.
 */
class HgBlock extends LinearRailBlock {
 static var recipe:Null<ComponentType>;
 public final model:String;
 public function new(model:String="HGH15CA") {
  if(HgReferenceData.blocks().indexOf(model)<0)throw "Unknown HG block: "+model;
  super(HgReferenceData.spec(model),true,"-REF-"+HgReferenceData.hash(model).substr(0,12));
  this.model=model;setMaterial("unspecified");lockManufacturingMaterial("unspecified");
 }
 public static function referenceDirectory():String {
  var configured=Sys.getEnv("MATERIA_HG_REFERENCE");if(configured!=null && configured!="")return configured;
  var root=Sys.getEnv("MATERIA_ROOT");if(root==null || root=="")root=Sys.getCwd();
  return root+"/build/suppliers/vallder/hg-reference";
 }
 public static function load(model:String):Shape {
  var path=referenceDirectory()+"/"+model+".step";
  if(!sys.FileSystem.exists(path))throw "Missing HG reference; run python3 tools/suppliers/hg.py --prepare build/suppliers/vallder/hg-reference";
  if(HiwinHgh15ca.fileDigest(path)!=HgReferenceData.hash(model))throw "Unreviewed HG reference: "+model;
  return Shape.importStep(path);
 }
 /** Drawn from its own CAD, whose shape sets its mass. */
 override public function massGeometry():Null<cadkit.modeling.Part> return null;

 override public function geometry(detail:ComponentDetail=Preview):Part {
  var shape=load(model);
  try {
   if(detail!=Envelope)return new Part(shape);
   var lo=HgReferenceData.low(model),hi=HgReferenceData.high(model);
   var result=Part.prism([new Vector(lo[0],lo[1]),new Vector(hi[0],lo[1]),new Vector(hi[0],hi[1]),new Vector(lo[0],hi[1])],lo[2],hi[2]);
   shape.close();return result;
  }catch(e:Dynamic){shape.close();throw e;}
 }
 /** HGH table gives MxL; HGW table gives only M. Unknown thread depth stays unknown. */
 public function mountThreadDepth():Null<Float> return switch model {
  case "HGH15CA":5;case "HGH20CA":6;case "HGH25CA":8;case "HGH30CA":10;case "HGH35CA":12;case "HGH45CA":17;default:null;
 };
 public static function referenceRecipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.motion.hg-block-reference",[P.choice("model",HgReferenceData.blocks(),"HGH15CA")],v -> new HgBlock(v.token("model")),true,false,null,false);
  return recipe;
 }
 override public function componentType():Null<ComponentType> return referenceRecipeType();
 override public function values():ComponentValues return new ComponentValues().setToken("model",model);
}
