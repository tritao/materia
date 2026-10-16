import app.ProjectSourceLoader;
import app.Main.ReferenceEditorApp;
import app.editor.ExampleCatalog;
import sys.io.File;

/** Optional prepared-asset integration check, using the actual project loader and simulator. */
class ReferenceArmsApplicationTests {
  static function check(condition:Bool,message:String):Void {if (!condition) throw message;}
  public static function main():Void {
    var root = app.editor.WorkerAssetPath.resolve("machinekit/examples/reference-arms");
    var entry = ExampleCatalog.find("reference-arms");
    check(entry != null,"Reference library must appear in the application catalogue");
    var reports:Array<{model:String,parts:Int,jointMotion:Float,maxTrackingError:Float}> = [];
    for (id in ["ur5e","fairino5_v6"]) {
      var generated = ProjectSourceLoader.load(root+"/materia.project.json",null,null,id);
      check(generated.objects.length == 7 && generated.projectJob == id,"Source model job must load seven visible links");
      var path = root+"/build/"+id+"/"+id+"-cartesian.mtrg";
      var moving = ProjectSourceLoader.load(path);
      check(moving.robotMotions != null && moving.robotMotions.length == 6,"Cartesian artifact must retain six tracks");
      for (track in moving.robotMotions) check(Math.abs(track.keys[0].position)<1e-12,"Playback coordinates must start at the saved pose without a jump");
      var editor = new ReferenceEditorApp(null,root+"/build/test-workspace.json",null,null,null,null,null,null,true);
      try {
        editor.session.openGeneratedProject(moving,path);
        check(editor.simulation.rebuild(editor.sensors,editor.scene,editor.session),"Reference simulation must rebuild: "+editor.simulation.error);
        var robotId = "assembly:"+id;
        var initial = editor.simulation.snapshot().robot(robotId);
        if (initial == null) throw "Imported assembly must be a simulated robot";
        var before = initial.positions.toArray();
        for (_ in 0...200) editor.simulation.step();
        var observed = editor.simulation.snapshot().robot(robotId);
        if (observed == null) throw "Missing simulated reference robot";
        check(observed.faultCode == 0,"Reference playback must remain fault-free");
        var motion = 0.0, error = 0.0;
        var positions = observed.positions.toArray(), setpoints = observed.setpointPositions.toArray();
        for (j in 0...before.length) {
          motion = Math.max(motion,Math.abs(positions[j]-before[j]));
          error = Math.max(error,Math.abs(positions[j]-setpoints[j]));
        }
        check(motion>1e-4,"Checked Cartesian keys must produce observed joint motion");
        reports.push({model:id,parts:moving.objects.length,jointMotion:motion,maxTrackingError:error});
        editor.dispose();
      } catch (error:Dynamic) {editor.dispose(); throw error;}
    }
    var text = haxe.Json.stringify({scope:"source selection and 200 actual simulator steps; tracking errors are observations, not qualification",robots:reports},null,"  ");
    File.saveContent(root+"/build/application-validation.json",text);
    Sys.println(text);
    // The editor's background project workers outlive this headless check.
    Sys.exit(0);
  }
}
