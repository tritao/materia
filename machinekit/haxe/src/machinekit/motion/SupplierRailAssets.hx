package machinekit.motion;

/** Pinned distributor-hosted HIWIN design references. No network access at runtime. */
class SupplierRailAssets {
 public static function digest(name:String):String return switch name {
  case "HGH15CA": "3bc285a6e79d23be1a5d6408382785361402575449bf40175d2319b27a55c9d8";
  case "HGR15R": "2897086393c72de3ca2a1af658da9e471bfbe38e234605e89dbb8497283e4351";
  case "HGH25CA": "d1928e51da8a04694e98678ece89710cef6fa22dad3a6ddac47e54261fa491c6";
  case "HGR25R": "a5c25503c681841fbbee95b24a1ebcd890afcf2001b074ee4a5b817e1fc07909";
  default: throw "Unsupported supplier rail asset "+name;
 };
 public static function fileDigest(path:String):String {
  var digest:haxe.io.Bytes=haxe.crypto.Sha256.make(sys.io.File.getBytes(path)), hex=new StringBuf();
  for(i in 0...digest.length)hex.add(StringTools.hex(digest.get(i),2).toLowerCase());
  return hex.toString();
 }
 public static function path(name:String,?explicit:String):String {
  var p=explicit;
  if(p==null){var folder=Sys.getEnv("MACHINEKIT_SUPPLIER_CAD");
   if(folder==null){var root=Sys.getCwd();while(!sys.FileSystem.exists(root+"/machinekit/examples/linear-rail-reference")){
    var parent=haxe.io.Path.directory(root);if(parent==root||parent=="")throw "Set MACHINEKIT_SUPPLIER_CAD or run linear-rail-reference/prepare.py in the repository";root=parent;}
    folder=root+"/machinekit/examples/linear-rail-reference/build/supplier";}
   p=folder+"/"+name+".step";
  }
  if(!sys.FileSystem.exists(p))throw "Missing pinned "+name+" CAD; run python3 machinekit/examples/linear-rail-reference/prepare.py";
  if(fileDigest(p)!=digest(name))throw "Unreviewed "+name+" STEP: digest differs from pinned supplier reference";
  return p;
 }
}
