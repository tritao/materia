import materia.project.SceneArtifact;

/** Scene configuration and entry points; physical cell and presentation have their own owners. */
class HobberEnclosurePreview {
  public static function machine(?spec:gearkit.GearSpec, gashes:Int=12, detailedHob:Bool=false):FlexsplineHobber
    return new FlexsplineHobber(spec,gashes,detailedHob);

  public static function cell(?source:FlexsplineHobber):HobberCell.HobberInstalledCell
    return HobberCell.build(source==null?machine():source);

  public static function scene():haxe.io.Bytes {
    var t0=Sys.time();
    var source=machine(new gearkit.GearSpec(1,40,4,new gearkit.Involute(20*Math.PI/180)),4,true);
    var t1=Sys.time();
    var installed=cell(source);
    var t2=Sys.time();
    var data=HobberScenePresentation.build(installed.assembly,source,installed.shell.layout.stroke,true);
    var t3=Sys.time();
    if(Sys.getEnv("HOBBER_PHASES")=="1") {
      Sys.println("PHASES build="+Math.round(t1-t0)+"s fit="+Math.round(t2-t1)+"s export="+Math.round(t3-t2)+"s");
      for(line in machinekit.cache.Memos.report())Sys.println("MEMO "+line);
      for(line in cadkit.cache.OpCache.report())Sys.println("OPS "+line);
    }
    return SceneArtifact.encode(data);
  }

  public static function main():Void {
    var bytes=scene();
    var root=Sys.getCwd();
    while(!sys.FileSystem.exists(root+"/machinekit/examples/flexspline-hobber/machine/FlexsplineHobber.hx")) {
      var parent=haxe.io.Path.directory(root);
      if(parent==root||parent.length==0)throw "Cannot locate hobber worktree";
      root=parent;
    }
    var output=root+"/machinekit/examples/flexspline-hobber/build/hobber";
    sys.FileSystem.createDirectory(haxe.io.Path.directory(output));
    sys.FileSystem.createDirectory(output);
    sys.io.File.saveBytes(output+"/hobber.mtrg",bytes);
    Sys.println("Unified hobber scene written");
  }
}
