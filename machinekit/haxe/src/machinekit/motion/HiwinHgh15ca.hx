package machinekit.motion;

import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;

/** Pinned supplier reference in the existing rail frame: X across, Y mounting normal, Z travel.
 * Explicit-path instances own their imported CAD: close after export/checks.
 * Saved recipes load lazily and retain no native CAD owner. No downloads during generation.
 * Intended as an explicit alternative to the generic HGR15 block, not a supplier certification. */
class HiwinHgh15ca extends LinearRailBlock {
 public static inline var SHA256="3bc285a6e79d23be1a5d6408382785361402575449bf40175d2319b27a55c9d8";
 public static inline var SOURCE="https://www.matis.sk/data/CAD_modely/05_Kolejnicova_vedeni/HG/HGH15CA.STP";
 public static inline var MOUNT_THREAD_DEPTH:Float=5;
 static var recipe:Null<ComponentType>;
 final persisted:Bool;
 var source:Null<Shape>;
 var closed=false;
 public function new(?path:String) {
  super(LinearRailSystem.catalog().get("HGR15"),path!=null,"-HIWIN-HGH15CA-STEP-"+SHA256.substr(0,12));
  persisted=path==null;
  if(persisted){setMaterial("unspecified");lockManufacturingMaterial("unspecified");}
  if(path!=null)source=load(path);
 }
 public static function referencePath():String {
  var configured=Sys.getEnv("MATERIA_HGH15CA_STEP");
  if(configured!=null && configured!="")return configured;
  var root=Sys.getEnv("MATERIA_ROOT");if(root==null || root=="")root=Sys.getCwd();
  return root+"/machinekit/examples/linear-rail-reference/build/supplier/HGH15CA.step";
 }
 static function load(path:String):Shape {
  if(!sys.FileSystem.exists(path))throw "Missing HGH15CA reference: run python3 machinekit/examples/linear-rail-reference/prepare.py";
  if(fileDigest(path)!=SHA256)throw "Unreviewed HGH15CA STEP: run linear-rail-reference/prepare.py for the pinned source";
  var imported=Shape.importStep(path),b=imported.bounds();
  if(Math.abs(b.get_min().get_x()+17)>0.02 || Math.abs(b.get_max().get_x()-17)>0.02 ||
     Math.abs(b.get_min().get_y()+10.7)>0.02 || Math.abs(b.get_max().get_y()-13)>0.02 ||
     Math.abs(b.get_min().get_z()+30.7)>0.02 || b.get_max().get_z()>40) {
   imported.close();throw "HGH15CA supplier frame does not match the reviewed interface";
  }
  return imported;
 }
 public static function referenceRecipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.motion.hgh15ca-reference",[],_ -> new HiwinHgh15ca(),true,false,null,false);
  return recipe;
 }
 override public function componentType():Null<ComponentType> return persisted ? referenceRecipeType() : null;
 override public function values():ComponentValues return persisted ? new ComponentValues() : super.values();
 /** Drawn from its own CAD, whose shape sets its mass. */
 override public function massGeometry():Null<cadkit.modeling.Part> return null;

 override public function geometry(detail:ComponentDetail=Preview):Part {
  if(closed)throw "HGH15CA reference is closed";
  // Saved recipes retain no native CAD owner; explicit-path callers keep the original owned API.
  var imported=persisted ? load(referencePath()) : source;
  if(imported==null)throw "Missing HGH15CA geometry";
  try {
   var result:Part;
   if(detail==Envelope) {
    var b=imported.bounds(),lo=b.get_min(),hi=b.get_max();
    result=Part.prism([new Vector(lo.get_x(),lo.get_y()),new Vector(hi.get_x(),lo.get_y()),
      new Vector(hi.get_x(),hi.get_y()),new Vector(lo.get_x(),hi.get_y())],lo.get_z(),hi.get_z());
   } else result=new Part(imported.cloneShape());
   if(persisted)imported.close();return result;
  } catch(e:Dynamic){if(persisted)imported.close();throw e;}
 }
 public static function fileDigest(path:String):String {
  var digest=haxe.crypto.Sha256.make(sys.io.File.getBytes(path)),hex=new StringBuf();
  for(i in 0...digest.length)hex.add(StringTools.hex(digest.get(i),2).toLowerCase());
  return hex.toString();
 }
 public function close():Void {if(!closed){closed=true;if(source!=null)source.close();source=null;}}
}
