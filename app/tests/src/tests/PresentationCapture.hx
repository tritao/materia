package tests;

/** Capture a named presentation through the real editor without saving its session cursor. */
@:access(app.Main)
@:access(app.Main.ReferenceEditorApp)
class PresentationCapture {
  public static function main():Int {
    var args=Sys.args(),id:Null<String> = null;
    var launch:Array<String> = [];
    for(arg in args)if(StringTools.startsWith(arg,"--view="))id=arg.substr(7);else launch.push(arg);
    if(id==null)throw "PresentationCapture requires --view=ID and editor launch arguments";
    var live=app.Main.open(launch),selected=false;
    while(live.tick()) {
      var editor=app.Main.liveEditor;
      if(!selected&&editor!=null&&!editor.openingProject()) {
        var viewport=editor.perspectiveViewport;
        if(viewport!=null&&viewport.presentation!=null) {
          viewport.selectPresentationView(id);editor.invalidateView();selected=true;
        }
      }
    }
    var result=live.close();
    if(!selected)throw "No assembly presentation opened for capture";
    return result;
  }
}
