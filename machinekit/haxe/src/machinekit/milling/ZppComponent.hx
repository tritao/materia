package machinekit.milling;

import cadkit.Shape;
import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Reviewed solid groups from pinned supplier STEP. Coordinates: X closes jaws, Y across jaws, Z up. */
class ZppComponent extends MachineComponent {
 static var recipe:Null<ComponentType>;
 static final sources:Map<String,Shape> = [];
 public final role:String;
 public static inline var VISE_BOTTOM:Float=23.3172156163821;
 public static inline var BED:Float=VISE_BOTTOM+11.684;
 public static inline var RAW_GAP:Float=59;
 public static function roles():Array<String> return ["body","positive-jaw","negative-jaw","screw","plate","stud","stud-link","ar-body","ar-positive-carrier","ar-negative-carrier","ar-positive-soft-jaw","ar-negative-soft-jaw","ar-positive-hard-jaw","ar-negative-hard-jaw","plate96","stud96","stud-link96"];
 public function new(role:String="body") {
  if(roles().indexOf(role)<0)throw "Unknown ZPP workholding member";
  this.role=role;
  super("ZPP-"+(isLink()?(role=="stud-link96"?"TS96-M10-LINK":"TS52-M8-LINK"):model())+"-"+role+"-"+(isLink()?(role=="stud-link96"?"NOMINAL-M10x20":"NOMINAL-M8x16"):hash(model()).substr(0,12)),
   isLink()?"Nominal threaded pull-stud connection envelope":"ZPP "+model()+" supplier STEP: "+role,(role=="ar-body"||role.indexOf("soft-jaw")>=0)?"aluminium 6061":"steel");
  addConnector("mount",Mount,Solids.axial(0,0,0));
  if(role=="ar-body")for(i in 0...2) {
   var name=i==0?"airA":"airB";
   addConnector(name,Face,F.alongY(77.5,i==0?-11:11,10,1,0,0));
   addPort({name:name,kind:Pneumatic,role:Consumer,iface:Unspecified,required:false,connector:name});
  }

 }
 public static function directory():String {
  var overridePath=Sys.getEnv("MATERIA_ZPP_ROOT");if(overridePath!=null)return overridePath;
  var root=Sys.getEnv("MATERIA_ROOT");return (root==null?Sys.getCwd():root)+"/build/suppliers/zppcnc";
 }
 public static function hash(model:String):String return switch model {
  case "CV15075":"b782addb56e6a6ef7ba42cc0b92b436095a9446adee44c76b5f46b7c3030ce6a";
  case "TS52-108":"7a1b4c016ef48c9a031434050236be9584657259652eddd20f07fd5530876f76";
  case "ps16f":"7fdc724d4de71ff3437a6a49cd03e7bd3c14928151a2579cf8561de955a4a7e9";
  case "AR155_I":"69bf1c9add67720c3ad1b2c744bc6be97f38cefabe2518e4c475645055c96e84";
  case "TS96-155":"2af3087bdd43a449d84976074027fef8f3661ba2bbd59a27003dc5b0028ef1ec";
  case "ps20f":"0220bb52a9811a081a0768ae03d8bb52ae783ea03130ff9f20264c3352b66b75";
  default:throw "Unknown ZPP CAD model";
 };
 function isLink():Bool return role=="stud-link"||role=="stud-link96";
 function model():String return role=="plate96"?"TS96-155":role=="stud96"?"ps20f":StringTools.startsWith(role,"ar-")?"AR155_I":role=="plate"?"TS52-108":role=="stud"?"ps16f":"CV15075";
 static function source(model:String):Shape {
  if(!sources.exists(model)) {
   var path=directory()+"/"+model+".stp";
   if(!sys.FileSystem.exists(path))throw "Missing ZPP CAD; run python3 tools/suppliers/zppcnc.py --prepare";
   if(machinekit.motion.HiwinHgh15ca.fileDigest(path)!=hash(model))throw "Unreviewed ZPP STEP: "+model;
   sources.set(model,Shape.importStep(path));
  }
  return sources.get(model);
 }
 override public function hasGeometry():Bool return true;
 override public function geometry(detail:ComponentDetail=Preview):Part {
  if(isLink())return role=="stud-link96"?Part.cylinderSpan(5,-10,10):Part.cylinderSpan(4,-8,8);
  var raw=source(model()),indices:Array<Int> = switch role {
   case "body":[for(i in 0...81)i].concat([83]);
   case "positive-jaw":[81,85,86,89,91,92,93,94];
   case "negative-jaw":[82,87,88,90,95,96,97,98];
   case "screw":[84];
   case "plate":[0]; // remove shipping dust covers to expose installed mounting screws
   case "stud","plate96","stud96","ar-body":[0];
   case "ar-positive-carrier":[1];case "ar-negative-carrier":[2];
   case "ar-positive-soft-jaw":[3];case "ar-negative-soft-jaw":[4];
   case "ar-positive-hard-jaw":[5];case "ar-negative-hard-jaw":[6];
   default:throw "Unknown ZPP member";
  };
  var expected=StringTools.startsWith(role,"ar-")?7:(role=="plate96"||role=="stud96"||role=="stud")?1:role=="plate"?5:99;
  if(raw.subshapeCount(CadKit.ShapeKind.Solid)!=expected)throw "ZPP solid layout changed";
  var pieces:Array<Shape> = [];
  try {
   for(i in indices)pieces.push(raw.subshape(CadKit.ShapeKind.Solid,i));
   var group=new Part(Shape.compound(pieces));for(p in pieces)p.close();pieces=[];
   var pose=(role=="stud"||role=="stud96")?F.fromRotationMatrix(0,0,role=="stud"?-24.638:-29.083,[0,0,1,0,-1,0,1,0,0]):
    StringTools.startsWith(role,"ar-")?F.fromRotationMatrix(0,0,0,[1,0,0,0,0,-1,0,1,0]):
    F.fromRotationMatrix(0,0,(role=="plate"||role=="plate96")?0:VISE_BOTTOM,[0,0,1,1,0,0,0,1,0]);
   try {var result=machinekit.assembly.PosedParts.moved(group,pose);group.close();return result;}
   catch(e:Dynamic){group.close();throw e;}
  }catch(e:Dynamic){for(p in pieces)p.close();throw e;}
 }
 public static function recipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.milling.zpp-supplier-member",[P.choice("role",roles(),"body")],v->new ZppComponent(v.token("role")),true,false,null,false);
  return recipe;
 }
 override public function componentType():Null<ComponentType> return recipeType();
 override public function values():ComponentValues return new ComponentValues().setToken("role",role);
}
