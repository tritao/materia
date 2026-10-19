package app.editor;

import app.ApplicationSimulation;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.core.View;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.text.Text;

/** Commands are queued to the playback owner; feedback is a coherent published image. */
class EnclosureGuardPanel {
  public static function build(simulation:ApplicationSimulation):Null<View> {
    var guard=simulation.enclosureController();if(guard==null)return null;
    var s=guard.status(),style=new LayoutStyle();style.width=LayoutAxis.grow();style.childGap=6;
    var controls:Array<KeyedView> =[];
    function action(label:String,key:String,enabled:Bool,run:Void->Void):Void {
      var button=new Button(label,null,()->simulation.control(run),key);button.enabled=enabled;controls.push(new KeyedView(key,button));
    }
    var live=simulation.isRunning();
    action("Open door","guard-open",live&&!s.fault&&(s.phase=="Secured"||s.phase=="Idle"),guard.requestOpen);
    action("Close door","guard-close",live&&!s.fault&&s.phase=="Open",guard.requestClose);
    action("Start cut","guard-start",live&&s.canStart,guard.requestCut);
    action("Reset guard","guard-reset",live&&s.fault&&s.closed&&s.locked&&s.axesStandstill&&s.spindleStandstill,guard.resetGuard);
    var rows:Array<KeyedView> = [new KeyedView("heading",new Text("ENCLOSURE GUARD")),new KeyedView("status",new Text(s.message)),
      new KeyedView("door-feedback",new Text('Door ${s.open?"open":s.closed?"closed":"moving"} · ${s.locked?"locked":"unlocked"}')),
      new KeyedView("motion-feedback",new Text('Spindle ${s.spindleStandstill?"stopped":"stop unconfirmed"} · axes ${s.axesStandstill?"stopped":"stop unconfirmed"}')),
      new KeyedView("door-actions",new Row("guard-door-actions",controls.slice(0,2),style)),
      new KeyedView("cut-actions",new Row("guard-cut-actions",controls.slice(2,4),style))];
    if(!live)rows.push(new KeyedView("hint",new Text("Press Play to operate the door and guard controls.")));
    if(s.interrupted||s.fault) {
      var reset=new Button("Reset simulation",null,()->{simulation.reset();},"guard-reset-simulation");
      rows.push(new KeyedView("reset-simulation",reset));
    }
    return new Column("enclosure-guard",rows,style);
  }
}
