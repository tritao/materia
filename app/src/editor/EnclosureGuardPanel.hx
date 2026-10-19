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
  /** Design previews edit the document pose; they never create a simulation or grant cut permission. */
  public static function preview(session:app.ProjectDocumentSession,editable:Bool,set:(String,Float)->Void):Null<View> {
    var motion=session.machineMotion,config=motion==null?null:motion.enclosureGuard;
    if(config==null)return null;
    var axes=app.MachineControlModel.axes(session).filter(a->a.id==config.doorJoint);
    if(axes.length!=1)return null;
    var axis=axes[0],closed=Math.abs(axis.value)<0.5,opened=Math.abs(axis.value-config.stroke*1000)<0.5;
    var style=new LayoutStyle();style.width=LayoutAxis.grow();style.childGap=6;
    var open=new Button("Open door",null,()->set(config.doorJoint,config.stroke*1000),"guard-preview-open");
    var close=new Button("Close door",null,()->set(config.doorJoint,0),"guard-preview-close");
    open.enabled=editable&&!opened;close.enabled=editable&&!closed;
    return new Column("enclosure-preview",[
      new KeyedView("heading",new Text("ENCLOSURE · DESIGN PREVIEW")),
      new KeyedView("status",new Text('Door ${opened?"open":closed?"closed":"partly open"}')),
      new KeyedView("actions",new Row("guard-preview-actions",[new KeyedView("open",open),new KeyedView("close",close)],style)),
      new KeyedView("hint",new Text(editable?"Preview geometry only. Press Play for guarded operation.":"Stop simulation or release project control to preview the doors."))
    ],style);
  }

  public static function build(simulation:ApplicationSimulation):Null<View> {
    var guard=simulation.enclosureController();if(guard==null)return null;
    var s=guard.status(),style=new LayoutStyle();style.width=LayoutAxis.grow();style.childGap=6;
    var controls:Array<KeyedView> =[];
    function action(label:String,key:String,enabled:Bool,run:Void->Void):Void {
      var button=new Button(label,null,()->simulation.control(run),key);button.enabled=enabled;controls.push(new KeyedView(key,button));
    }
    action("Open door","guard-open",s.canOpen,guard.requestOpen);
    action("Close door","guard-close",s.canClose,guard.requestClose);
    action("Start cut","guard-start",s.canStartCut,guard.requestCut);
    action("Reset guard","guard-reset",s.canReset,guard.resetGuard);
    var rows:Array<KeyedView> = [new KeyedView("heading",new Text("ENCLOSURE · SIMULATION")),new KeyedView("status",new Text(s.message)),
      new KeyedView("door-feedback",new Text(s.doorMessage)),
      new KeyedView("motion-feedback",new Text(s.motionMessage)),
      new KeyedView("door-actions",new Row("guard-door-actions",controls.slice(0,2),style)),
      new KeyedView("cut-actions",new Row("guard-cut-actions",controls.slice(2,4),style))];
    if(s.canResume) {
      rows.push(new KeyedView("resume",new Button("Resume simulation",null,simulation.start,"guard-resume")));
      rows.push(new KeyedView("hint",new Text("Resuming samples feedback and enables guarded door operation. Start cut remains a separate action.")));
    }
    if(s.canResetSimulation) {
      var reset=new Button("Reset simulation",null,()->{simulation.reset();},"guard-reset-simulation");
      rows.push(new KeyedView("reset-simulation",reset));
    }
    return new Column("enclosure-guard",rows,style);
  }
}
