import HobberMoldSetup.HobberMoldCasting;

/** Same native planner, scene and fabrication exporter as the mill kits. */
class HobberMoldPreview {
 static function foundation(casting:HobberMoldCasting):machinekit.structural.MineralFoundation return switch casting {
  case Plinth:new machinekit.hobbing.MineralCastBed();
  case Column:new machinekit.hobbing.MineralCastColumn();
  default:throw "Unknown hobber mold casting: "+casting;
 };

 public static function scene(casting:HobberMoldCasting=Plinth):haxe.io.Bytes {
  var foundation=foundation(casting);
  var document=HobberMoldSetup.document(foundation,casting);
  var plan=machinekit.tooling.CastingMoldCad.plan(document,HobberMoldSetup.options(document.input));
  return machinekit.tooling.CastingMoldExport.artifact(plan);
 }

 public static function main():Void {
  var casting:HobberMoldCasting=Sys.args().indexOf("--column")>=0 ? Column : Plinth;
  var foundation=foundation(casting);
  var document=HobberMoldSetup.document(foundation,casting),cwd=Sys.getCwd();
  var root=sys.FileSystem.exists(cwd+"/machinekit") ? cwd+"/machinekit/examples/flexspline-hobber" : cwd;
  CastingShopExport.write(document,HobberMoldSetup.options(document.input),foundation,
   casting,"hobber-"+casting+"-process-review",root,HobberMoldChecks.run);
 }
}
