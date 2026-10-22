package machinekit.assembly;


/** Reviewed drawing data, mm/kg. These are interface records, not verified offers. */
typedef HzMotionHole = {var x:Float;var z:Float;var diameter:Float;}
typedef HzMotionSpec = {
 var model:String;var kind:String;var family:String;var page:Int;var source:String;
 var integrated:Bool;var face:Float;var pilot:Float;var pcd:Float;var thread:String;
 var threadDepth:Null<Float>;var pilotDepth:Null<Float>;var length:Float;var shaftHeight:Float;
 var width:Float;var flange:Float;var rearStart:Float;var rearEnd:Float;var rearWidth:Float;var rearHeight:Float;
 var shaftBore:Float;var shoulderZ:Null<Float>;var massKg:Null<Float>;var baseHoles:Array<HzMotionHole>;
 var bearing:Null<String>;var bearingArrangement:Null<String>;var maxAxialLoadKgf:Null<Float>;
 var oppositeSupport:Null<String>;var endProfile:Null<String>;var geometryBasis:String;var drawingRevision:String;
}
typedef HzMotionCoverage = {var family:String;var motorInterface:String;var support:String;var sourcePage:Int;var status:String;}
typedef HzMotionCadAsset = {var model:String;var sha256:String;var translation:Array<Float>;var rotation:Array<Float>;var ?interfaces:{var width:Float;var baseHoles:Array<HzMotionHole>;var ?bearingCentreZ:Float;var ?boltSeatY:Float;};}
class HzMotionCatalog {
 public static function cadModels():Array<String> return [for(a in HzMotionData.cadAssets())a.model];
 public static function cadAsset(model:String):HzMotionCadAsset {
  for(a in HzMotionData.cadAssets())if(a.model==model)return a;
  throw 'HZMotion STEP interfaces not verified for $model';
 }
 public static function models():Array<String> return [for(p in HzMotionData.parts())p.model];
 public static function revisions():Array<String> return [revision(),"hzmotion-pdf-2025-04-09-v1"];
 public static function get(model:String,?revision:String):HzMotionSpec {
  var parts=revision==null||revision==HzMotionCatalog.revision()?HzMotionData.parts():revision=="hzmotion-pdf-2025-04-09-v1"?HzMotionData.legacyParts():[];
  for(p in parts)if(p.model==model)return p;
  throw 'Unknown HZMotion drawing model $model';
 }
 public static function coverage():Array<HzMotionCoverage> return HzMotionData.coverage();
 public static function discrepancies():Array<String> return HzMotionData.discrepancies();
 public static function revision():String return HzMotionData.revision();
 public static function sourceUrl(model:String):String {
  var p=get(model);
  return 'https://www.hzmotion.com/pdf/${p.source=="motor"?"Motor_Bracket":"Ballscrew_support_unit"}.pdf#page=${p.page}';
 }
}
