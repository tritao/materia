package nativekit.ui.widgets.overlays;
import nativekit.ui.widgets.KeyedView;
import nativekit.ui.widgets.controls.Button;
import nativekit.ui.widgets.controls.ButtonVariant;
import nativekit.ui.widgets.layout.Column;

import LayoutAxis;
import LayoutStyle;
import Insets;
import nativekit.ui.core.BuildContext;
import nativekit.ui.core.View;
import nativekit.ui.core.RenderNode;
import nativekit.ui.core.UiKey;
import nativekit.ui.core.UiEventKind;
import nativekit.ui.semantics.AccessibilityAction;
import nativekit.ui.semantics.AccessibilityRole;
import nativekit.ui.semantics.AccessibilityState;
import nativekit.ui.semantics.Semantics;

/** Keyboard-focusable popup menu composed from ordinary Haxe buttons. */
class Menu implements View {
	final key:String;
	final items:Array<MenuItem>;
	public final x:Float;
	public final y:Float;
	public var onDismiss:Void->Void;
	public var hasDismissHandler(default, null):Bool;

	public function new(key:String, items:Array<MenuItem>, x:Float = 0.0, y:Float = 0.0,
			?onDismiss:Void->Void) {
		this.key = key;
		this.items = items == null ? [] : items.copy();
		this.x = x;
		this.y = y;
		hasDismissHandler = onDismiss != null;
		this.onDismiss = onDismiss == null ? function() {} : onDismiss;
	}

	public function build(context:BuildContext):nativekit.ui.core.RenderNode {
		var children:Array<KeyedView> = [];
		for (item in items) {
			var button = new Button(item.label, null, function() {
				if (item.hasSelectHandler)
					item.onSelect();
				if (hasDismissHandler)
					onDismiss();
			}, item.key);
			button.classes = ["menu-item"];
			button.variant = ButtonVariant.Navigation;
			button.enabled = item.enabled;
			button.semanticRole = AccessibilityRole.MenuItem;
			button.semanticActions = AccessibilityAction.Select;
			children.push(new KeyedView(item.key, button));
		}
		var menuStyle = new LayoutStyle();
		menuStyle.width = LayoutAxis.fixed(220.0);
		menuStyle.childGap = 2.0;
		var content = new Column("menu-items", children, menuStyle);
		var popupStyle = new LayoutStyle();
		popupStyle.background = context.theme.tokens.navigationBackground;
		popupStyle.padding = new Insets(4.0, 4.0, 4.0, 4.0);
		popupStyle.radiusTopLeft = 6.0;
		popupStyle.radiusTopRight = 6.0;
		popupStyle.radiusBottomLeft = 6.0;
		popupStyle.radiusBottomRight = 6.0;
		var popup = new Popup(key, content, x, y, popupStyle,
			hasDismissHandler ? onDismiss : null);
		popup.label = "Menu";
		popup.modal = true;
		popup.dimBackdrop = false;
		var root:RenderNode = popup.build(context);
		var focusableItems:Array<RenderNode> = [];
		root.walk(function(node) {
			if (node.semantics != null && node.semantics.role == AccessibilityRole.MenuItem && node.enabled)
				focusableItems.push(node);
		});
		root.on(UiEventKind.KeyDown, function(event) {
			if (event.defaultPrevented || (event.key != UiKey.Down && event.key != UiKey.Up))
				return;
			if (focusableItems.length > 0) {
				var current = -1;
				for (index in 0...focusableItems.length)
					if (focusableItems[index].id.equals(event.target)) {
						current = index;
						break;
					}
				var next = event.key == UiKey.Down ? current + 1 : current - 1;
				if (next < 0) next = focusableItems.length - 1;
				if (next >= focusableItems.length) next = 0;
				context.requestFocus(focusableItems[next].id);
			}
			event.preventDefault();
		});
		var semantics = new Semantics(AccessibilityRole.Menu, "Menu");
		semantics.states |= AccessibilityState.Modal;
		if (hasDismissHandler)
			semantics.actions |= AccessibilityAction.Dismiss;
		root.semantics = semantics;
		if (hasDismissHandler)
			root.on(UiEventKind.Activate, function(event) {
				if (event.target.equals(root.id))
					onDismiss();
			});
		return root;
	}
}
