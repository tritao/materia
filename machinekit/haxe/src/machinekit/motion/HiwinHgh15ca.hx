package machinekit.motion;

import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;

/** Pinned supplier reference in the existing rail frame: X across, Y mounting normal, Z travel.
 * Owns its imported CAD: close after geometry export/checks. No download occurs during generation.
 * Intended as an explicit alternative to the generic HGR15 block, not a supplier certification. */
class HiwinHgh15ca extends LinearRailBlock {
 public static inline var SHA256="3bc285a6e79d23be1a5d6408382785361402575449bf40175d2319b27a55c9d8";
 public static inline var SOURCE="https://www.matis.sk/data/CAD_modely/05_Kolejnicova_vedeni/HG/HGH15CA.STP";
 final source:Shape;
 var closed=false;
 public function new(path:String) {
  var digest:haxe.io.Bytes=haxe.crypto.Sha256.make(sys.io.File.getBytes(path));
  var hex=new StringBuf();
  for(i in 0...digest.length) hex.add(StringTools.hex(digest.get(i),2).toLowerCase());
  if(hex.toString()!=SHA256)
   throw "Unreviewed HGH15CA STEP: run linear-rail-reference/prepare.py for the pinned source";
  super(LinearRailSystem.catalog().get("HGR15"),true,"-HIWIN-HGH15CA-STEP-"+SHA256.substr(0,12));
  source=Shape.importStep(path);
  // A supplier-file origin change must not silently move every connector on an assembled axis.
  var b=source.bounds();
  if(Math.abs(b.get_min().get_x()+17)>0.02 || Math.abs(b.get_max().get_x()-17)>0.02 ||
     Math.abs(b.get_min().get_y()+10.7)>0.02 || Math.abs(b.get_max().get_y()-13)>0.02 ||
     Math.abs(b.get_min().get_z()+30.7)>0.02 || b.get_max().get_z()>40) {
   source.close();throw "HGH15CA supplier frame does not match the reviewed interface";
  }
 }
 override public function geometry(detail:ComponentDetail=Preview):Part {
  if(closed)throw "HGH15CA reference is closed";
  if(detail==Envelope) {
   var b=source.bounds(),lo=b.get_min(),hi=b.get_max();
   return Part.prism([new Vector(lo.get_x(),lo.get_y()),new Vector(hi.get_x(),lo.get_y()),
    new Vector(hi.get_x(),hi.get_y()),new Vector(lo.get_x(),hi.get_y())],lo.get_z(),hi.get_z());
  }
  return new Part(source.cloneShape());
 }
 public static function fileDigest(path:String):String {
  var digest=haxe.crypto.Sha256.make(sys.io.File.getBytes(path)),hex=new StringBuf();
  for(i in 0...digest.length)hex.add(StringTools.hex(digest.get(i),2).toLowerCase());
  return hex.toString();
 }
 public function close():Void {if(!closed){closed=true;source.close();}}
}
