package tests;

/** Capture a named presentation through the real editor without saving its session cursor. */
@:access(app.Main)
@:access(app.Main.ReferenceEditorApp)
@:access(app.EditorPerspectiveViewport)
class PresentationCapture {
  public static function main():Int {
    var args=Sys.args(),id:Null<String> = null;
    var launch:Array<String> = [];
    var explode=0.0;
    for(arg in args) {
      if(StringTools.startsWith(arg,"--view="))id=arg.substr(7);
      else if(StringTools.startsWith(arg,"--explode="))explode=Std.parseFloat(arg.substr(10));
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
          viewport.presentationCameraDirty=true;editor.invalidateView();selected=true;
        }
      }
    }
    var result=live.close();
    if(!selected)throw "No assembly presentation opened for capture";
    return result;
  }
}
