package app.editor;

import haxeon.ui.Color;
import haxeon.ui.Insets;
import haxeon.ui.LayoutAlignmentX;
import haxeon.ui.LayoutAlignmentY;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutDirection;
import haxeon.ui.LayoutDistribution;
import haxeon.ui.LayoutPositioning;
import haxeon.ui.LayoutStyle;
import haxeon.ui.LayoutVisualKind;
import haxeon.ui.core.BuildContext;
import haxeon.ui.core.Key;
import haxeon.ui.core.PointerClickSequence;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.TextStyleOverride;
import haxeon.ui.core.UiEventKind;
import haxeon.ui.core.View;
import haxeon.ui.icons.IconName;
import haxeon.ui.settings.SettingsJsonValue;
import haxeon.ui.settings.SettingsStore;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.controls.ButtonVariant;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.layout.Spacer;
import haxeon.ui.widgets.overlays.Tooltip;
import haxeon.ui.widgets.text.Text;

/**
 * The viewport's scene controls (presentation, project selection, mission status) as one card that can be
 * collapsed to its header and dragged to a viewport corner. Corner and collapsed state live in the user's
 * settings file under the "sceneCard" state key.
 *
 * The state is a plain object, not Observable: it is written by pointer handlers and by `setPlaying` during
 * a build, and the build only reads it, so no tracked field is ever written inside a tracking scope.
 */
class SceneCardModel {
  public static inline var TOP_LEFT:String = "top-left";
  public static inline var TOP_RIGHT:String = "top-right";
  public static inline var BOTTOM_LEFT:String = "bottom-left";
  public static inline var BOTTOM_RIGHT:String = "bottom-right";
  public static final CORNERS:Array<String> = [TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT];
  /** Gap between the card and the viewport edges, matching the view-angle control's inset. */
  public static inline var MARGIN:Float = 10.0;
  public static inline var STATE_KEY:String = "sceneCard";

  public var corner(default, null):String = BOTTOM_LEFT;
  /** What the user chose; Play overrides it temporarily without changing it. */
  public var userCollapsed(default, null):Bool = false;
  /** Corners that hold other controls; the view-angle button sits in the top-left. */
  public var blocked:Array<String> = [TOP_LEFT];
  public var playing(default, null):Bool = false;
  // While Play is on, null follows the user's choice; otherwise the card's state during Play.
  var playCollapsed:Null<Bool> = null;
  public var onPersist:Null<Void -> Void> = null;
  // Drag state, in viewport coordinates of the card's top-left corner.
  public var dragging(default, null):Bool = false;
  public var dragX(default, null):Float = 0.0;
  public var dragY(default, null):Float = 0.0;
  /** Corner the card will snap to if released now. */
  public var snapCorner(default, null):String = BOTTOM_LEFT;

  public function new() {}

  public function collapsed():Bool return playCollapsed != null ? playCollapsed : userCollapsed;

  /** The chevron: the user's choice while idle, a temporary choice while Play holds the card collapsed. */
  public function toggleCollapsed():Void {
    var next = !collapsed();
    if (playing) playCollapsed = next;
    else {
      userCollapsed = next;
      persist();
    }
  }

  /** Play collapses the card; pause or stop brings back what it was. */
  public function setPlaying(value:Bool):Void {
    if (value == playing) return;
    playing = value;
    playCollapsed = value ? true : null;
  }

  public function setCorner(value:String):Void {
    if (CORNERS.indexOf(value) < 0 || value == corner) return;
    corner = value;
    persist();
  }

  public static function isLeft(corner:String):Bool return corner == TOP_LEFT || corner == BOTTOM_LEFT;
  public static function isTop(corner:String):Bool return corner == TOP_LEFT || corner == TOP_RIGHT;

  /** Top-left position of a card of the given size resting in the corner of a viewport. */
  public static function restingPosition(
    corner:String,
    viewWidth:Float,
    viewHeight:Float,
    cardWidth:Float,
    cardHeight:Float
  ): {x:Float, y:Float} {
    return {
      x: isLeft(corner) ? MARGIN : Math.max(MARGIN, viewWidth - cardWidth - MARGIN),
      y: isTop(corner) ? MARGIN : Math.max(MARGIN, viewHeight - cardHeight - MARGIN)
    };
  }

  /** The unblocked corner nearest to a point; the nearest blocked ones are skipped in order. */
  public static function nearestCorner(
    x:Float,
    y:Float,
    viewWidth:Float,
    viewHeight:Float,
    blocked:Array<String>
  ):String {
    var ranked = CORNERS.copy();
    function distance(corner:String):Float {
      var dx = x - (isLeft(corner) ? 0.0 : viewWidth), dy = y - (isTop(corner) ? 0.0 : viewHeight);
      return dx * dx + dy * dy;
    }
    ranked.sort((a, b) -> {
      var d = distance(a) - distance(b);
      return d < 0 ? -1 : d > 0 ? 1 : 0;
    });
    for (corner in ranked) if (blocked.indexOf(corner) < 0) return corner;
    return ranked[0];
  }

  /** Starts following the pointer; the card keeps the position it was grabbed at. */
  public function beginDrag(x:Float, y:Float, viewWidth:Float, viewHeight:Float, cardWidth:Float, cardHeight:Float):Void {
    dragging = true;
    moveDrag(x, y, viewWidth, viewHeight, cardWidth, cardHeight);
  }

  /** `x` and `y` are the card's top-left corner in the viewport's coordinates. */
  public function moveDrag(x:Float, y:Float, viewWidth:Float, viewHeight:Float, cardWidth:Float, cardHeight:Float):Void {
    dragX = Math.max(0.0, Math.min(Math.max(0.0, viewWidth - cardWidth), x));
    dragY = Math.max(0.0, Math.min(Math.max(0.0, viewHeight - cardHeight), y));
    snapCorner = nearestCorner(dragX + cardWidth * 0.5, dragY + cardHeight * 0.5, viewWidth, viewHeight, blocked);
  }

  /** Releases the card into the nearest free corner. */
  public function endDrag():Void {
    if (!dragging) return;
    dragging = false;
    setCorner(snapCorner);
  }

  public function cancelDrag():Void dragging = false;

  public function toState(): {corner:String, collapsed:Bool} return {corner: corner, collapsed: userCollapsed};

  /** Restores saved state; anything unusable keeps the defaults. */
  public function load(state:Dynamic):Void {
    if (state == null || !Reflect.isObject(state)) return;
    var savedCorner:Dynamic = Reflect.field(state, "corner");
    if (SettingsJsonValue.isString(savedCorner) && CORNERS.indexOf(savedCorner) >= 0 && blocked.indexOf(savedCorner) < 0) corner = savedCorner;
    var savedCollapsed:Dynamic = Reflect.field(state, "collapsed");
    if (Std.isOfType(savedCollapsed, Bool)) userCollapsed = savedCollapsed;
  }

  function persist():Void if (onPersist != null) onPersist();
}

/** Card view plus the glue to the settings store. */
class SceneControlsCard {
  public final model:SceneCardModel = new SceneCardModel();
  final clicks = new PointerClickSequence();
  @:allow(app.editor.SceneCardLayer) var cardWidth:Float = 300.0;
  @:allow(app.editor.SceneCardLayer) var cardHeight:Float = 40.0;
  @:allow(app.editor.SceneCardLayer) var viewWidth:Float = 0.0;
  @:allow(app.editor.SceneCardLayer) var viewHeight:Float = 0.0;
  // Nodes of the latest build, read by pointer handlers for bounds; not tracked state.
  @:allow(app.editor.SceneCardLayer) var layerNode:Null<RenderNode> = null;
  @:allow(app.editor.SceneCardLayer) var cardNode:Null<RenderNode> = null;
  var grabX:Float = 0.0;
  var grabY:Float = 0.0;
  var downX:Float = 0.0;
  var downY:Float = 0.0;
  var pressed:Bool = false;
  var pressPointer:Int = -1;
  var changed:Void -> Void = () -> {};

  public function new() {}

  /** Restores the saved card state and writes later changes back to the same store. */
  public function attach(store:SettingsStore):Void {
    model.load(store.getState(SceneCardModel.STATE_KEY));
    model.onPersist = () -> store.setState(SceneCardModel.STATE_KEY, model.toState());
  }

  /** "Assembled · Machine": the current assembly view and the selected configuration. */
  public static function summary(viewLabel:Null<String>, variantLabel:Null<String>):String {
    var parts = [for (part in [viewLabel, variantLabel]) if (part != null && part != "") part];
    return parts.length == 0 ? "Scene" : parts.join(" · ");
  }

  /** Label of the selected variant, or null; an empty match must not be indexed in this runtime. */
  public static function variantLabel(variants:Array<app.ProjectVariants.ProjectVariant>, selected:Null<String>):Null<String> {
    for (variant in variants) if (variant.id == selected) return variant.label;
    return null;
  }

  /** The viewport-sized layer holding the card; `controls` are the scene-control cards in stacking order. */
  public function layer(
    controls:Array<KeyedView>,
    summaryText:String,
    playing:Bool,
    availableWidth:Float,
    invalidate:Void -> Void,
    tokens:haxeon.ui.theme.ThemeTokens
  ):View {
    model.setPlaying(playing);
    changed = invalidate;
    return new SceneCardLayer(this, controls, summaryText, availableWidth, tokens);
  }

  @:allow(app.editor.SceneCardLayer)
  function press(x:Float, y:Float, pointerId:Int, cardOriginX:Float, cardOriginY:Float):Void {
    pressed = true;
    pressPointer = pointerId;
    downX = x;
    downY = y;
    grabX = x - cardOriginX;
    grabY = y - cardOriginY;
  }

  @:allow(app.editor.SceneCardLayer)
  function move(x:Float, y:Float, pointerId:Int, layerX:Float, layerY:Float, w:Float, h:Float):Bool {
    if (!pressed || pointerId != pressPointer) return false;
    var dx = x - downX, dy = y - downY;
    if (!model.dragging && dx * dx + dy * dy < 36.0) return false;
    viewWidth = w;
    viewHeight = h;
    var args = [x - grabX - layerX, y - grabY - layerY];
    if (!model.dragging) model.beginDrag(args[0], args[1], w, h, cardWidth, cardHeight);
    else model.moveDrag(args[0], args[1], w, h, cardWidth, cardHeight);
    changed();
    return true;
  }

  /** Returns true when the release ended a drag. */
  @:allow(app.editor.SceneCardLayer)
  function release(pointerId:Int, canceled:Bool, event:haxeon.ui.core.UiEvent):Bool {
    if (!pressed || pointerId != pressPointer) return false;
    pressed = false;
    var wasDragging = model.dragging;
    if (canceled) model.cancelDrag();
    else if (wasDragging) model.endDrag();
    else if (clicks.register("header", event) == 2) model.toggleCollapsed();
    changed();
    return wasDragging;
  }

  @:allow(app.editor.SceneCardLayer)
  function toggle():Void {
    model.toggleCollapsed();
    changed();
  }

  @:allow(app.editor.SceneCardLayer)
  function measured(w:Float, h:Float):Void {
    cardWidth = w;
    cardHeight = h;
  }
}

private class SceneCardLayer implements View {
  final owner:SceneControlsCard;
  final controls:Array<KeyedView>;
  final summaryText:String;
  final availableWidth:Float;
  final tokens:haxeon.ui.theme.ThemeTokens;

  public function new(
    owner:SceneControlsCard,
    controls:Array<KeyedView>,
    summaryText:String,
    availableWidth:Float,
    tokens:haxeon.ui.theme.ThemeTokens
  ) {
    this.owner = owner;
    this.controls = controls;
    this.summaryText = summaryText;
    this.availableWidth = availableWidth;
    this.tokens = tokens;
  }

  public function build(context:BuildContext):RenderNode {
    var model = owner.model;
    var collapsed = model.collapsed();
    var corner = model.corner;
    var layerStyle = new LayoutStyle();
    layerStyle.width = LayoutAxis.grow();
    layerStyle.height = LayoutAxis.grow();
    layerStyle.direction = LayoutDirection.TopToBottom;
    var margin = SceneCardModel.MARGIN;
    layerStyle.padding = new Insets(margin, margin, margin, margin);
    layerStyle.childAlignX = SceneCardModel.isLeft(corner) ? LayoutAlignmentX.Start : LayoutAlignmentX.End;
    layerStyle.childDistribution = SceneCardModel.isTop(corner) ? LayoutDistribution.Start : LayoutDistribution.End;
    var layer = new RenderNode(context.id("scene-card-layer"), LayoutVisualKind.Box, layerStyle);
    layer.hitTestSelf = false;

    var cardStyle = new LayoutStyle();
    cardStyle.direction = LayoutDirection.TopToBottom;
    cardStyle.childGap = 10;
    cardStyle.padding = new Insets(8, 6, 8, collapsed ? 6 : 10);
    cardStyle.background = tokens.surfaceRaised;
    var radius = 8.0;
    cardStyle.radiusTopLeft = cardStyle.radiusTopRight = cardStyle.radiusBottomLeft = cardStyle.radiusBottomRight = radius;
    if (!collapsed) {
      cardStyle.width = LayoutAxis.fixed(Math.max(120.0, Math.min(300.0, (availableWidth > 0 ? availableWidth : 400.0) - 20.0)));
    } else {
      cardStyle.width = LayoutAxis.fit(0, Math.max(120.0, Math.min(300.0, (availableWidth > 0 ? availableWidth : 400.0) - 20.0)));
    }
    var rows:Array<KeyedView> = [new KeyedView("header", header(collapsed))];
    if (!collapsed) {
      var body = new LayoutStyle();
      body.width = LayoutAxis.grow();
      body.childGap = 10;
      rows.push(new KeyedView("body", new Column("scene-card-body", controls, body)));
    }
    var cardNode = context.withScope(new Key("card"), function() return new Column("scene-card", rows, cardStyle).build(context));
    if (model.dragging) {
      cardNode.layout.style.positioning = LayoutPositioning.Absolute;
      cardNode.layout.style.positionX = model.dragX;
      cardNode.layout.style.positionY = model.dragY;
      cardNode.layout.style.zIndex = 5;
    }
    layer.add(cardNode);
    if (model.dragging) {
      // Where the card will land if released now.
      var landing = SceneCardModel.restingPosition(
        model.snapCorner,
        owner.viewWidth,
        owner.viewHeight,
        owner.cardWidth,
        owner.cardHeight
      );
      var accent = tokens.accent;
      var hint = new Spacer("scene-card-snap", LayoutAxis.fixed(owner.cardWidth), LayoutAxis.fixed(owner.cardHeight));
      hint.style.background = Color.rgba(accent.red, accent.green, accent.blue, 0.28);
      hint.style.radiusTopLeft = hint.style.radiusTopRight = hint.style.radiusBottomLeft = hint.style.radiusBottomRight = radius;
      var hintNode = context.withScope(new Key("snap"), function() return hint.build(context));
      hintNode.hitTestSelf = false;
      hintNode.layout.style.positioning = LayoutPositioning.Absolute;
      hintNode.layout.style.positionX = landing.x;
      hintNode.layout.style.positionY = landing.y;
      hintNode.layout.style.zIndex = 4;
      layer.add(hintNode);
    }
    owner.layerNode = layer;
    owner.cardNode = cardNode;
    cardNode.onResolved(function(_) {
      var bounds = cardNode.globalBounds();
      owner.measured(bounds.width, bounds.height);
    });
    return layer;
  }

  function header(collapsed:Bool):View {
    var textRow = new LayoutStyle();
    textRow.width = collapsed ? LayoutAxis.fit() : LayoutAxis.grow();
    textRow.childAlignY = LayoutAlignmentY.Center;
    textRow.childGap = 8;
    var grip = gripDots();
    var label = new Text(summaryText, null, tokens.text, TextStyleOverride.text(12.0));
    var region = new DragRegion(owner, new Row("scene-card-drag", [
      new KeyedView("grip", grip),
      new KeyedView("summary", label)
    ], textRow));
    var chevron = new Button("", null, owner.toggle, "scene-card-toggle");
    chevron.variant = ButtonVariant.Navigation;
    chevron.leadingIcon = collapsed ? IconName.ChevronRight : IconName.ChevronDown;
    chevron.iconSize = 14.0;
    var hint = collapsed ? "Expand scene controls" : "Collapse scene controls";
    chevron.accessibilityLabel = hint;
    var tip = new Tooltip("scene-card-toggle-tip", chevron, new Text(hint + " (double-click the header)"), -120.0, 30.0);
    var style = new LayoutStyle();
    style.width = collapsed ? LayoutAxis.fit() : LayoutAxis.grow();
    style.direction = LayoutDirection.LeftToRight;
    style.childAlignY = LayoutAlignmentY.Center;
    style.childGap = 4;
    return new Row("scene-card-header", [new KeyedView("region", region), new KeyedView("toggle", tip)], style);
  }

  function gripDots():View {
    var style = new LayoutStyle();
    style.childGap = 3;
    var rows:Array<KeyedView> = [];
    for (r in 0...3) {
      var rowStyle = new LayoutStyle();
      rowStyle.childGap = 3;
      var dots:Array<KeyedView> = [];
      for (c in 0...2) {
        var dot = new Spacer("dot-" + r + "-" + c, LayoutAxis.fixed(3.0), LayoutAxis.fixed(3.0));
        dot.style.background = tokens.textSecondary;
        dot.style.radiusTopLeft = dot.style.radiusTopRight = dot.style.radiusBottomLeft = dot.style.radiusBottomRight = 1.5;
        dots.push(new KeyedView("d" + c, dot));
      }
      rows.push(new KeyedView("r" + r, new Row("grip-row-" + r, dots, rowStyle)));
    }
    return new Column("scene-card-grip", rows, style);
  }
}

/** The part of the header that drags the card and toggles it on double-click. */
private class DragRegion implements View {
  final owner:SceneControlsCard;
  final content:View;

  public function new(owner:SceneControlsCard, content:View) {
    this.owner = owner;
    this.content = content;
  }

  public function build(context:BuildContext):RenderNode {
    var node = content.build(context);
    // The layer and card are found from the region at event time, never captured as tracked state.
    node.on(UiEventKind.PointerDown, function(event) {
      if (event.button != 0) return;
      var card = owner.cardNode;
      if (card == null || card.resolved == null) return;
      var bounds = card.globalBounds();
      owner.press(event.x, event.y, event.pointerId, bounds.x, bounds.y);
      event.capturePointer(false);
    });
    node.on(UiEventKind.PointerMove, function(event) {
      var layer = owner.layerNode;
      if (layer == null || layer.resolved == null) return;
      var bounds = layer.globalBounds();
      if (owner.move(event.x, event.y, event.pointerId, bounds.x, bounds.y, bounds.width, bounds.height)) event.preventDefault();
    });
    node.on(UiEventKind.PointerUp, function(event) {
      if (owner.release(event.pointerId, false, event)) event.preventDefault();
      event.releasePointer();
    });
    node.on(UiEventKind.PointerCancel, function(event) {
      owner.release(event.pointerId, true, event);
      event.releasePointer();
    });
    return node;
  }
}
