package app.editor;

import haxeon.ui.core.BuildContext;
import haxeon.ui.core.View;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.UiEvent;
import haxeon.ui.core.UiEventKind;
import haxeon.ui.widgets.controls.Slider;

/** Adds pointer gesture boundaries to the shared slider without changing keyboard edits. */
class MachineControlSlider implements View {
  final slider:Slider;
  public var enabled(get,set):Bool;
  function get_enabled():Bool return slider.enabled;
  function set_enabled(value:Bool):Bool return slider.enabled=value;
  final begin:UiEvent->Void;
  final finish:Bool->Void;
  public function new(key:String,label:String,value:Float,minimum:Float,maximum:Float,step:Float,
      change:Float->Void,begin:UiEvent->Void,finish:Bool->Void) {
    slider=new Slider(key,label,value,minimum,maximum,step,change);
    this.begin=begin;this.finish=finish;
  }
  public function build(context:BuildContext):RenderNode {
    var node=slider.build(context);
    node.on(UiEventKind.PointerDown,event->{if(enabled && event.button==0)begin(event);},"target");
    node.on(UiEventKind.PointerUp,event->{if(event.button==0)finish(true);});
    node.on(UiEventKind.PointerCancel,_->finish(false));
    return node;
  }
}
