import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;

/** Each assembly in isolation, with the production machine's installed parts. */
class HobberUnitPreview {
  static function unit(key:String):Bytes {
    var machine=new FlexsplineHobber();
    for(unit in machine.units) if(unit.key==key) return SceneArtifact.encode(AssemblyPreview.scene(unit,"hobber-"+key));
    throw "Unknown hobber unit: "+key;
  }
  public static function foundation():Bytes return unit("foundation");
  public static function tableWork():Bytes return unit("tableWork");
  public static function xAxis():Bytes return unit("xAxis");
  public static function zAxis():Bytes return unit("zAxis");
  public static function aHead():Bytes return unit("aHead");
  public static function yStage():Bytes return unit("yStage");
  public static function bSpindle():Bytes return unit("bSpindle");
  public static function electrical():Bytes return unit("electrical");
  public static function main():Void foundation();
}
