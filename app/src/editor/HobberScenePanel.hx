package app.editor;

import app.Main.ReferenceEditorApp;
import haxeon.ui.core.View;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.text.Text;

/** Process selection and presentation are independent; neither edits the authored assembly. */
@:access(app.Main.ReferenceEditorApp)
class HobberScenePanel {
  public static function build(app:ReferenceEditorApp):Null<View> {
    var motion=app.session.machineMotion,config=motion==null?null:motion.hobbing;
    if(config==null||config.scenario==null)return null;
    var style=new LayoutStyle();style.width=LayoutAxis.grow();style.childGap=6;
    var cells:Array<KeyedView> = [new KeyedView("scenario",new Text("Play scenario:"))];
    for(choice in [{id:"idle",label:"Idle / doors"},{id:"cutting",label:"Cutting"},{id:"loading",label:"Load / cut / unload"}]) {
      var id=choice.id;
      var button=new Button(choice.label,null,()->{app.simulation.selectHobbingScenario(app.session,id);app.invalidateView();},"hobber-scenario-"+id);
      button.selected=app.session.hobbingScenario==id;
      button.enabled=!app.simulation.isRunning()&&!app.documents.blocked();
      cells.push(new KeyedView(id,button));
    }
    var player=app.simulation.hobbingPlayer();
    cells.push(new KeyedView("state",new Text(player==null?"Select, then Play (changes reset the run)":player.scenarioLabel())));
    return new Row("hobber-scene-controls",cells,style);
  }
}
