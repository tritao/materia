package app.editor;

import haxeon.ui.LayoutStyle;
import haxeon.ui.Insets;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutAlignmentY;
import haxeon.ui.LayoutVisualKind;
import haxeon.ui.PathBuilder;
import haxeon.ui.style.StyleState;
import haxeon.ui.icons.IconData;
import haxeon.ui.widgets.overlays.TooltipPlacement;
import haxeon.ui.core.*;
import haxeon.ui.icons.IconName;
import haxeon.ui.semantics.*;
import haxeon.ui.widgets.Icon;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.style.StyleTarget;
import haxeon.ui.LayoutDirection;
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
  final toggle:Null<Void -> Void>;
  final isolate:Null<Void -> Void>;
  public function new(
    key:String,
    label:String,
    icon:IconName,
    detail:Null<String>,
    visible:Bool,
    ownVisible:Bool,
    mixed:Bool,
    toggle:Null<Void -> Void>,
    ?isolate:Void -> Void
  ) {
    this.key = key;
    this.label = label;
    this.icon = icon;
    this.detail = detail;
    this.visible = visible;
    this.ownVisible = ownVisible;
    this.mixed = mixed;
    this.toggle = toggle;
    this.isolate = isolate;
  }
  public function build(context:BuildContext):RenderNode {
    return context.withScope(new Key("item-row:" + key), () -> {
      var rowId = context.id("row");
      // Hover changes only the isolate glyph paint; keep the row and its label layout retained.
      return context.selfUpdating(rowId, () -> buildRow(context, rowId), false);
    });
  }
  function buildRow(context:BuildContext, rowId:WidgetId):RenderNode {
    var color = visible ? context.theme.tokens.textPrimary : context.theme.tokens.textSecondary;
    var text = new MiddleEllipsisText("label:" + key, label, true, TextStyleOverride.foreground(color));
    var tooltip = new Tooltip("label-tip:" + key, text, new Text(detail == null ? label : detail));
    tooltip.fillAnchor = true;
    tooltip.placement = TooltipPlacement.Right;
    tooltip.showWhen = () -> detail != null || text.truncated;
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.direction = LayoutDirection.LeftToRight;
    style.childAlignY = LayoutAlignmentY.Center;
    style.childGap = 6;
    // Keep the 24px eye target clear of the overlay scrollbar (12px + 2px inset).
    style.padding = new Insets(0, 0, 16, 0);
    var children = [new KeyedView("icon", new Icon("glyph:" + key, icon, 15, color)), new KeyedView("label", tooltip)];
    var action:Null<IsolateAction> = null;
    if (isolate != null) {
      action = new IsolateAction("isolate:" + key, "Isolate " + label, rowId, isolate);
      var hint = new Tooltip("isolate-tip:" + key, action, new Text("Isolate " + label));
      hint.placement = TooltipPlacement.Right;
      children.push(new KeyedView("isolate", hint));
    }
    if (toggle != null) {
      var hint = (ownVisible ? "Hide " : "Show ") + label + " (Space)";
      if (ownVisible && !visible) hint += " · Hidden by object, group or set visibility";
      else if (mixed) hint += " · Some descendants hidden";
      var hintView = new Tooltip("eye-tip:" + key,
        new VisibilityEye("eye:" + key, hint, ownVisible, visible, mixed, toggle), new Text(hint));
      hintView.placement = TooltipPlacement.Right;
      children.push(new KeyedView("eye", hintView));
    }
    var rowKey = "item-row:" + key;
    var flags = context.interactionStates.get(rowId);
    var computed = context.resolveStyle(new StyleTarget("row", rowKey, rowKey, null, ["row"], flags), style);
    var node = new RenderNode(rowId, LayoutVisualKind.Box, computed.toLayoutStyle());
    node.setStyleIdentity("row", rowKey, rowKey, null, ["row"]);
    node.states = flags;
    node.computedStyle = computed;
    for (child in children) node.add(context.withStyleParent(computed, () -> child.build(context)));
    var isolateAction = action;
    if (isolateAction != null) {
      // The glyph reads this row's hover while painting, so invalidate only that small display list.
      var repaint:UiEvent -> Void = event -> {
        if (event.target.equals(rowId)) isolateAction.invalidatePaint();
      };
      node.on(UiEventKind.HoverEnter, repaint);
      node.on(UiEventKind.HoverLeave, repaint);
    }
    return node;
  }
}

/** Quiet row action: its target and keyboard semantics remain available while the glyph is hidden. */
private class IsolateAction implements View {
  final key:String;
  final label:String;
  final rowId:WidgetId;
  var paintNode:Null<RenderNode> = null;
  final activate:Void -> Void;
  public function new(key:String, label:String, rowId:WidgetId, activate:Void -> Void) {
    this.key = key;
    this.label = label;
    this.rowId = rowId;
    this.activate = activate;
  }
  public function invalidatePaint():Void {
    var current = paintNode;
    if (current != null) RenderNode.latest(current).invalidatePaint();
  }
  public function build(context:BuildContext):RenderNode return context.withScope(new Key(key), () -> {
    var id = context.id("action");
    var result = context.selfUpdating(id, () -> {
      var flags = context.interactionStates.get(id);
      var focused = flags.contains(StyleState.Focused);
      var style = new LayoutStyle();
      style.width = LayoutAxis.fixed(24);
      style.height = LayoutAxis.fixed(24);
      var node = new RenderNode(id, LayoutVisualKind.Custom, style);
      node.states = flags;
      node.focusable = true;
      node.semantics = new Semantics(AccessibilityRole.Button, label);
      node.semantics.actions = AccessibilityAction.Activate;
      node.semantics.states |= AccessibilityState.Focusable;
      node.on(UiEventKind.PointerDown, event -> {
        if (event.button == 0) {
          context.requestFocus(id);
          event.stopPropagation();
        }
      });
      node.onActivate(event -> {
        event.preventDefault();
        event.stopPropagation();
        activate();
      });
      var color = context.theme.tokens.textSecondary;
      var focusColor = context.theme.tokens.focusRing;
      node.onPaint((canvas, _) -> {
        var shown = focused || context.interactionStates.get(rowId).contains(StyleState.Hovered);
        if (focused) canvas.strokeTransient(new PathBuilder().roundRect(0.75, 0.75, 22.5, 22.5, 4).build(), focusColor, 1.5);
        if (shown) canvas.withState(target -> {
          target.translate(5, 5);
          target.scale(14 / 24, 14 / 24);
          target.strokeTransient(IconData.build(IconName.Inspect), color, 2.0,
            haxeon.ui.LineCap.Round, haxeon.ui.LineJoin.Round);
        });
      }, 'isolate:$focused:' + color.red + ':' + color.green + ':' + color.blue + ':' + color.alpha
        + ':' + focusColor.red + ':' + focusColor.green + ':' + focusColor.blue + ':' + focusColor.alpha);
      return node;
    }, false);
    paintNode = result;
    return result;
  });
}

private class VisibilityEye implements View {
  final key:String;
  final label:String;
  final open:Bool;
  final effective:Bool;
  final mixed:Bool;
  final toggle:Void -> Void;
  public function new(key:String, label:String, open:Bool, effective:Bool, mixed:Bool, toggle:Void -> Void) {
    this.key = key;
    this.label = label;
    this.open = open;
    this.effective = effective;
    this.mixed = mixed;
    this.toggle = toggle;
  }
  public function build(context:BuildContext):RenderNode return context.withScope(new Key(key),
    () -> {var style = new LayoutStyle();
  style.width = LayoutAxis.fixed(24);
  style.height = LayoutAxis.fixed(24);
  var node = new RenderNode(context.id("visibility"), LayoutVisualKind.Custom, style);
  node.focusable = true;
  node.states = context.interactionStates.get(node.id);
  node.semantics = new Semantics(AccessibilityRole.Button, label);
  node.semantics.actions = AccessibilityAction.Activate;
  node.semantics.states |= AccessibilityState.Focusable;
  var activate = function(event:UiEvent) {
    event.preventDefault();
    event.stopPropagation();
    toggle();
  };
  node.on(UiEventKind.PointerDown, function(event) {
    if (event.button == 0) {
      context.requestFocus(node.id);
      event.stopPropagation();
    }
  }
  );
  node.on(UiEventKind.Click, function(event) {
    if (event.button == 0) activate(event);
  }
  );
  node.on(UiEventKind.Activate, function(event) {
    if (event.modifiers.isEmpty()) activate(event);
    else event.stopPropagation();
  }
  );
  node.on(UiEventKind.KeyDown, function(event) {
    if (event.modifiers.isEmpty() && (event.key == UiKey.Space || event.key == UiKey.Enter)) activate(event);
  }
  );
  node.on(UiEventKind.KeyRepeat, function(event) {
    if (event.key == UiKey.Space || event.key == UiKey.Enter) {
      event.preventDefault();
      event.stopPropagation();
    }
  }
  );
  var color = effective ? context.theme.tokens.textPrimary : context.theme.tokens.textSecondary;
  var focused = node.states.contains(haxeon.ui.style.StyleState.Focused);
  var focusColor = context.theme.tokens.focusRing;
  node.onPaint(function(canvas, _) {
    if (focused) canvas.strokeTransient(new PathBuilder().roundRect(0.75,
      0.75, 22.5, 22.5, 4).build(), focusColor, 1.5);
    var path = new PathBuilder().moveTo(3, 12).cubicTo(7, 4, 17, 4, 21, 12).cubicTo(17, 20, 7, 20, 3, 12).close();
    if (open && mixed) path.moveTo(9, 12).lineTo(15, 12);
    else if (open) path.roundRect(9, 9, 6, 6, 3);
    else path.moveTo(4, 4).lineTo(20, 20);
    canvas.strokeTransient(path.build(), color, 1.5);
  }, 'eye:$open:$effective:$mixed:$focused:' + color.red + ':' + color.green + ':' + color.blue);
  return node;
  });
}
