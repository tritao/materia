package app.editor;

import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.LayoutVisualKind;
import haxeon.ui.PathBuilder;
import haxeon.ui.Rect;
import haxeon.ui.core.BuildContext;
import haxeon.ui.core.Key;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.UiEvent;
import haxeon.ui.core.UiEventKind;
import haxeon.ui.core.UiKey;
import haxeon.ui.core.UiModifier;
import haxeon.ui.core.View;
import haxeon.ui.semantics.AccessibilityAction;
import haxeon.ui.semantics.AccessibilityRole;
import haxeon.ui.semantics.AccessibilityState;
import haxeon.ui.semantics.Semantics;

private typedef AngleDrag = {var active:Bool;var x:Float;var angle:Float;var value:Float;}

/** A dial and an endless scrub strip share accumulated degrees and gesture transactions. */
class MachineAngleControl implements View {
  final key:String;
  final label:String;
  final dial:Bool;
  final change:Float->Void;
  final begin:UiEvent->Void;
  final finish:Bool->Void;
  final minimum:Null<Float>;
  final maximum:Null<Float>;
  public var value:Float;
  public var enabled=true;
  public final style:LayoutStyle;

  public function new(key:String,label:String,value:Float,dial:Bool,change:Float->Void,
      begin:UiEvent->Void,finish:Bool->Void,?minimum:Float,?maximum:Float) {
    this.key=key;this.label=label;this.value=value;this.dial=dial;
    this.change=change;this.begin=begin;this.finish=finish;this.minimum=minimum;this.maximum=maximum;
    style=new LayoutStyle();style.width=dial ? LayoutAxis.fixed(72) : LayoutAxis.grow();
    style.height=LayoutAxis.fixed(dial ? 72 : 36);
  }
  public static function deltaAngle(from:Float,to:Float):Float {
    var delta=(to-from)%360;
    return delta>180 ? delta-360 : delta < -180 ? delta+360 : delta;
  }
  public function build(context:BuildContext):RenderNode return context.withScope(new Key(key),()->{
    var node=new RenderNode(context.id("angle"),LayoutVisualKind.Custom,style);
    node.focusable=enabled;node.enabled=enabled;
    var wrapped=minimum==null && maximum==null;
    var displayed=wrapped ? app.MachineControlModel.wrapDegrees(value) : value;
    var semantics=new Semantics(AccessibilityRole.Slider,label,Std.string(displayed)+" degrees");
    semantics.numericValue=displayed;
    semantics.numericMinimum=wrapped ? 0 : minimum==null ? value-360 : minimum;
    semantics.numericMaximum=wrapped ? 360 : maximum==null ? value+360 : maximum;
    semantics.actions=AccessibilityAction.Increment|AccessibilityAction.Decrement|AccessibilityAction.SetValue;
    if(!enabled)semantics.states|=AccessibilityState.Disabled;
    node.semantics=semantics;
    var drag=context.state(context.id("drag"),{active:false,x:0.0,angle:0.0,value:value});
    var refresh=context.state(context.id("value"),value);
    var emit=function(next:Float) {
      if(!enabled || !Math.isFinite(next))return;
      if(minimum!=null)next=Math.max(minimum,next);
      if(maximum!=null)next=Math.min(maximum,next);
      next=Math.fround(next*10000)/10000;
      if(next==value)return;
      value=next;
      var displayed=wrapped ? app.MachineControlModel.wrapDegrees(value) : value;
      semantics.numericValue=displayed;semantics.value=Std.string(displayed)+" degrees";
      if(!wrapped && minimum==null)semantics.numericMinimum=value-360;
      if(!wrapped && maximum==null)semantics.numericMaximum=value+360;
      refresh.update(value);change(value);
    };
    var point=function(event:UiEvent):{x:Float,y:Float} {
      var bounds=node.resolved;
      if(bounds==null)return {x:0.0,y:0.0};
      var p=bounds.viewportToLayout(event.x,event.y);
      return {x:p.x-bounds.x,y:p.y-bounds.y};
    };
    var move=function(event:UiEvent) {
      var state:AngleDrag=drag.value,bounds=node.resolved;
      if(!state.active || bounds==null || !enabled)return;
      var p=point(event),fine=(event.modifiers & UiModifier.Shift)!=0 ? 0.1 : 1.0;
      var angle=Math.atan2(p.x-bounds.width/2,bounds.height/2-p.y)*180/Math.PI;
      var delta=dial ? deltaAngle(state.angle,angle) : p.x-state.x;
      var next=value+delta*fine;
      // A centre crossing has no reliable angular direction.
      if(!dial || Math.pow(p.x-bounds.width/2,2)+Math.pow(p.y-bounds.height/2,2)>16)emit(next);
      drag.update({active:true,x:p.x,angle:angle,value:value});
    };
    node.on(UiEventKind.PointerDown,event->{
      if(!enabled || event.button!=0)return;
      begin(event);if(event.defaultPrevented)return;
      var bounds=node.resolved;if(bounds==null)return;
      var p=point(event),angle=Math.atan2(p.x-bounds.width/2,bounds.height/2-p.y)*180/Math.PI;
      event.capturePointer();
      if(dial && (event.modifiers & UiModifier.Shift)==0 && Math.pow(p.x-bounds.width/2,2)+Math.pow(p.y-bounds.height/2,2)>16)
        emit(value+deltaAngle(value,angle));
      drag.update({active:true,x:p.x,angle:angle,value:value});event.preventDefault();
    });
    node.on(UiEventKind.PointerMove,move);
    node.on(UiEventKind.PointerUp,event->{
      var state:AngleDrag=drag.value;
      if(event.button!=0 || !state.active)return;
      move(event);drag.update({active:false,x:0.0,angle:0.0,value:value});finish(true);
    });
    node.on(UiEventKind.PointerCancel,_-> {
      var state:AngleDrag=drag.value;if(!state.active)return;
      drag.update({active:false,x:0.0,angle:0.0,value:value});finish(false);
    });
    node.on(UiEventKind.KeyDown,event->{
      if(!enabled)return;
      var step=(event.modifiers & UiModifier.Shift)!=0 ? 0.1 : 1.0;
      if(event.key==UiKey.Left || event.key==UiKey.Down)emit(value-step);
      else if(event.key==UiKey.Right || event.key==UiKey.Up)emit(value+step);
      else return;
      event.preventDefault();
    });
    node.on(UiEventKind.AccessibilityIncrement,_->emit(value+1));
    node.on(UiEventKind.AccessibilityDecrement,_->emit(value-1));
    node.on(UiEventKind.AccessibilitySetValue,event->{
      if(event.text!=null) {try {
        var next=app.MachineControlModel.parsePosition(event.text);
        emit(wrapped ? value+deltaAngle(value,app.MachineControlModel.wrapDegrees(next)) : next);
      }catch(_:Dynamic){}}
    });
    var tokens=context.theme.tokens;
    node.onPaint((canvas,bounds)->{
      var ink=enabled ? tokens.accent : tokens.disabledText;
      if(dial) {
        var cx=bounds.width/2,cy=bounds.height/2,r=Math.min(cx,cy)-6;
        var ring=new PathBuilder();
        for(i in 0...49) {var a=i*Math.PI*2/48;var x=cx+r*Math.sin(a),y=cy-r*Math.cos(a);if(i==0)ring.moveTo(x,y);else ring.lineTo(x,y);}
        canvas.strokeTransient(ring.build(),tokens.controlUnselected,2);
        var marks=new PathBuilder();
        for(i in 0...12) {var a=i*Math.PI/6;marks.moveTo(cx+(r-4)*Math.sin(a),cy-(r-4)*Math.cos(a)).lineTo(cx+r*Math.sin(a),cy-r*Math.cos(a));}
        canvas.strokeTransient(marks.build(),tokens.mutedText,1);
        var a=(value%360)*Math.PI/180;
        canvas.strokeTransient(new PathBuilder().moveTo(cx,cy).lineTo(cx+(r-7)*Math.sin(a),cy-(r-7)*Math.cos(a)).build(),ink,3);
        canvas.fillRoundedRect(new Rect(cx-3,cy-3,6,6),3,ink);
      } else {
        canvas.fillRoundedRect(new Rect(0,0,bounds.width,bounds.height),5,tokens.selectionField);
        var ticks=new PathBuilder(),offset=((value%15)+15)%15;
        for(i in -1...Std.int(bounds.width/15)+2) {var x=i*15-offset;if(x>0 && x<bounds.width)ticks.moveTo(x,12).lineTo(x,24);}
        canvas.strokeTransient(ticks.build(),tokens.mutedText,1);
        canvas.fillRect(new Rect(bounds.width/2-1,7,2,bounds.height-14),ink);
      }
    },"angle:"+value+":"+enabled);
    return node;
  });
}
