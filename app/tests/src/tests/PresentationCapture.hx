package tests;

/** Capture a named presentation through the real editor without saving its session cursor. */
@:access(app.Main)
@:access(app.Main.ReferenceEditorApp)
@:access(app.EditorPerspectiveViewport)
class PresentationCapture {
  public static function main():Int {
    var args=Sys.args(),id:Null<String> = null;
    var launch:Array<String> = [];
    var explode=0.0,distanceScale=1.0,step=0;
    var procedure:Null<String> = null,editOffset:Null<String> = null;
    for(arg in args) {
      if(StringTools.startsWith(arg,"--view="))id=arg.substr(7);
      else if(StringTools.startsWith(arg,"--explode="))explode=Std.parseFloat(arg.substr(10));
      else if(StringTools.startsWith(arg,"--distance-scale="))distanceScale=Std.parseFloat(arg.substr(17));
      else if(StringTools.startsWith(arg,"--edit-offset="))editOffset=arg.substr(14);
      else if(StringTools.startsWith(arg,"--procedure="))procedure=arg.substr(12);
      else if(StringTools.startsWith(arg,"--step=")) {
        var parsed=Std.parseInt(arg.substr(7));if(parsed==null||parsed<0)throw "Invalid procedure step";step=parsed;
      }
      else launch.push(arg);
    }
    if(id==null)throw "PresentationCapture requires --view=ID and editor launch arguments";
    var live=app.Main.open(launch),selected=false;
    while(live.tick()) {
      var editor=app.Main.liveEditor;
      if(!selected&&editor!=null&&!editor.openingProject()) {
        var viewport=editor.perspectiveViewport;
        if(viewport!=null&&viewport.presentation!=null) {
          viewport.selectPresentationView(id);viewport.setPresentationExplode(explode);
          if(procedure!=null) {
            viewport.selectPresentationProcedure(procedure);
            var controller=viewport.presentation;if(controller!=null)for(_ in 0...step)controller.next(false);
          }
          viewport.presentationCameraDirty=true;viewport.updatePresentationCamera();
          viewport.camera.setTarget(viewport.camera.targetX,viewport.camera.targetY,viewport.camera.targetZ,viewport.camera.distance*distanceScale);
          if(editOffset!=null){editor.scene.select("project:"+editOffset);editor.editPresentationOffset(viewport);}
          editor.invalidateView();selected=true;
        }
      }
    }
    var result=live.close();
    if(!selected)throw "No assembly presentation opened for capture";
    return result;
  }
}
