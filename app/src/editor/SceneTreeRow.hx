package app.editor;

import haxeon.ui.LayoutStyle;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutAlignmentY;
import haxeon.ui.LayoutVisualKind;
import haxeon.ui.PathBuilder;
import haxeon.ui.core.*;
import haxeon.ui.icons.IconName;
import haxeon.ui.semantics.*;
import haxeon.ui.widgets.Icon;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.overlays.Tooltip;
import haxeon.ui.widgets.text.MiddleEllipsisText;
import haxeon.ui.widgets.text.Text;

/** Scene row presentation, including an independent, accessible visibility hit target. */
class SceneTreeRow implements View {
  final key:String;
  final label:String;
  final icon:IconName;
  final detail:Null<String>;
  final visible:Bool;
  final ownVisible:Bool;
  final mixed:Bool;
  final toggle:Null<Void->Void>;
  public function new(key:String, label:String, icon:IconName, detail:Null<String>, visible:Bool,
      ownVisible:Bool, mixed:Bool, toggle:Null<Void->Void>) {
    this.key=key; this.label=label; this.icon=icon; this.detail=detail;
    this.visible=visible; this.ownVisible=ownVisible; this.mixed=mixed; this.toggle=toggle;
  }
  public function build(context:BuildContext):RenderNode {
    var color = visible ? context.theme.tokens.textPrimary : context.theme.tokens.textSecondary;
    var text = new MiddleEllipsisText("label:" + key, label, true, TextStyleOverride.foreground(color));
    var tooltip = new Tooltip("label-tip:" + key, text, new Text(detail == null ? label : detail));
    tooltip.fillAnchor = true;
    tooltip.showWhen = () -> detail != null || text.truncated;
    var style = new LayoutStyle(); style.width=LayoutAxis.grow(); style.childAlignY=LayoutAlignmentY.Center; style.childGap=6;
    var children = [new KeyedView("icon", new Icon("glyph:" + key, icon, 15, color)), new KeyedView("label", tooltip)];
    if (toggle != null) {
      var hint = (ownVisible ? "Hide " : "Show ") + label + " (Space)";
      if (ownVisible && !visible) hint += " · Hidden by object or parent visibility";
      else if (mixed) hint += " · Some descendants hidden";
      children.push(new KeyedView("eye", new Tooltip("eye-tip:" + key,
        new VisibilityEye("eye:" + key, hint, ownVisible, visible, mixed, toggle), new Text(hint))));
    }
    return new Row("item-row:" + key, children, style).build(context);
  }
}

private class VisibilityEye implements View {
  final key:String; final label:String; final open:Bool; final effective:Bool; final mixed:Bool; final toggle:Void->Void;
  public function new(key:String,label:String,open:Bool,effective:Bool,mixed:Bool,toggle:Void->Void) {
    this.key=key;this.label=label;this.open=open;this.effective=effective;this.mixed=mixed;this.toggle=toggle;
  }
  public function build(context:BuildContext):RenderNode return context.withScope(new Key(key), () -> {
    var style=new LayoutStyle();style.width=LayoutAxis.fixed(24);style.height=LayoutAxis.fixed(24);
    var node=new RenderNode(context.id("visibility"),LayoutVisualKind.Custom,style);
    node.focusable=true;
    node.states=context.interactionStates.get(node.id);
    node.semantics=new Semantics(AccessibilityRole.Button,label);
    node.semantics.actions=AccessibilityAction.Activate;
    node.semantics.states|=AccessibilityState.Focusable;
    var activate=function(event:UiEvent) {event.preventDefault();event.stopPropagation();toggle();};
    node.on(UiEventKind.PointerDown, function(event) {if(event.button==0) {context.requestFocus(node.id);event.stopPropagation();}});
    node.on(UiEventKind.Click, function(event) {if(event.button==0)activate(event);});
    node.on(UiEventKind.Activate, function(event) {if(event.modifiers==0)activate(event);else event.stopPropagation();});
    node.on(UiEventKind.KeyDown, function(event) {
      if(event.modifiers==0 && (event.key==UiKey.Space || event.key==UiKey.Enter)) activate(event);
    });
    node.on(UiEventKind.KeyRepeat, function(event) {
      if(event.key==UiKey.Space || event.key==UiKey.Enter){event.preventDefault();event.stopPropagation();}
    });
    var color=effective ? context.theme.tokens.textPrimary : context.theme.tokens.textSecondary;
    var focused=(node.states & haxeon.ui.style.StyleState.Focused)!=0;
    var focusColor=context.theme.tokens.focusRing;
    node.onPaint(function(canvas,_) {
      if(focused)canvas.strokeTransient(new PathBuilder().roundRect(0.75,0.75,22.5,22.5,4).build(),focusColor,1.5);
      var path=new PathBuilder().moveTo(3,12).cubicTo(7,4,17,4,21,12).cubicTo(17,20,7,20,3,12).close();
      if(open && mixed) path.moveTo(9,12).lineTo(15,12);
      else if(open) path.roundRect(9,9,6,6,3);
      else path.moveTo(4,4).lineTo(20,20);
      canvas.strokeTransient(path.build(),color,1.5);
    }, 'eye:$open:$effective:$mixed:$focused:' + color.red + ':' + color.green + ':' + color.blue);
    return node;
  });
}
