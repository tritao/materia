package nativekit.ui.widgets.docking;
import nativekit.ui.widgets.controls.TabsSelectionMode;
import nativekit.ui.widgets.KeyedView;
import nativekit.ui.widgets.controls.TabItem;
import nativekit.ui.widgets.layout.Column;
import nativekit.ui.widgets.layout.SizedBox;
import nativekit.ui.widgets.layout.SplitOrientation;
import nativekit.ui.widgets.layout.SplitSide;
import nativekit.ui.widgets.layout.SplitView;
import nativekit.ui.widgets.layout.SplitViewOptions;
import nativekit.ui.widgets.text.Text;

import Color;
import LayoutAxis;
import LayoutPositioning;
import LayoutStyle;
import LayoutVisualKind;
import Rect;
import nativekit.ui.core.BuildContext;
import nativekit.ui.core.Key;
import nativekit.ui.core.RenderNode;
import nativekit.ui.core.State;
import nativekit.ui.core.View;
import nativekit.ui.docking.DockDropZone;
import nativekit.ui.docking.DockNode;
import nativekit.ui.docking.DockPanelDescriptor;
import nativekit.ui.docking.DockSplitAxis;
import nativekit.ui.docking.DockWorkspaceModel;
import nativekit.ui.widgets.docking.DockDropTarget;
import nativekit.ui.widgets.docking.DockTabDropTarget;
import nativekit.ui.widgets.docking.DockWorkspaceInteraction;
import nativekit.ui.widgets.controls.TabsOptions;

/** Renders a DockWorkspaceModel using split panes, tab groups, and lazy panels. */
class DockWorkspace implements View {
	public final key:String;
	public final model:DockWorkspaceModel;
	public var interaction(default, null):DockWorkspaceInteraction;
	public final style:LayoutStyle;
	final suppliedInteraction:Bool;
	final panelContents:Map<String, DockPanelContent>;

	public function new(key:String, model:DockWorkspaceModel,
			?panelContents:Array<DockPanelContent>, ?style:LayoutStyle,
			?interaction:DockWorkspaceInteraction) {
		if (key == null || key.length == 0 || model == null)
			throw "Dock workspaces require a stable key and model";
		this.key = key;
		this.model = model;
		suppliedInteraction = interaction != null;
		this.interaction = interaction == null ? new DockWorkspaceInteraction(model) : interaction;
		this.style = style == null ? defaultStyle() : style.copy();
		this.panelContents = new Map();
		if (panelContents != null)
			for (content in panelContents) {
				if (content == null || this.panelContents.exists(content.panelId))
					throw "Dock workspace panel content IDs must be unique";
				this.panelContents.set(content.panelId, content);
			}
	}

	public function build(context:BuildContext):RenderNode {
		var mounted:State<DockWorkspaceMount> = context.resourceState(context.id("workspace-mount:" + key),
			function() { return new DockWorkspaceMount(model, interaction); },
			function(value) { value.dispose(); });
		var mount = mounted.value;
		mount.bind(model, suppliedInteraction ? interaction : null);
		mount.invalidate = function() context.commands.refresh();
		interaction = mount.interaction;
		interaction.beginFrame();
		var content = buildNode(model.root, context, [], "layout",
			context.viewportWidth, context.viewportHeight);
		var layout = new SizedBox("layout", content, LayoutAxis.grow(), LayoutAxis.grow());
		return new Column(key, [new KeyedView("content", layout)], style).build(context);
	}

	function buildNode(node:DockNode, context:BuildContext, path:Array<Int>, nodeKey:String,
		availableWidth:Float, availableHeight:Float):View {
		if (node == null)
			return new Text("No dock layout");
		switch (node) {
			case DockNode.Empty: return new Text("No panels");
			case DockNode.Panel(panelId):
				return targetView(panelId, buildTabs([panelId], panelId, context, nodeKey));
			case DockNode.Tabs(panelIds, activePanelId):
				var targetPanelId = activePanelId == null && panelIds != null && panelIds.length > 0
					? panelIds[0] : activePanelId;
				return targetPanelId == null ? buildTabs(panelIds, activePanelId, context, nodeKey) :
					targetView(targetPanelId, buildTabs(panelIds, activePanelId, context, nodeKey));
			case DockNode.Split(axis, ratio, first, second):
				return buildSplit(axis, ratio, first, second, context, path, nodeKey,
					availableWidth, availableHeight);
		}
	}

	function buildTabs(panelIds:Array<String>, activePanelId:String,
		context:BuildContext, nodeKey:String):View {
		var items:Array<TabItem> = [];
		if (panelIds != null)
			for (panelId in panelIds) {
				var descriptor = model.get(panelId);
				if (descriptor != null)
					items.push(new TabItem(panelId, descriptor.title, panelView(panelId),
						descriptor.enabled, descriptor.icon));
			}
		var tabsStyle = new LayoutStyle();
		tabsStyle.width = LayoutAxis.grow();
		tabsStyle.height = LayoutAxis.grow();
		var options = new TabsOptions();
		options.style = tabsStyle;
		options.selectionMode = TabsSelectionMode.Controlled;
		options.onTabDragStart = function(panelId, event)
			interaction.beginTabDrag(panelId, event.pointerId, event.x, event.y);
		options.onTabDragMove = function(panelId, event)
			interaction.moveTabDrag(panelId, event.pointerId, event.x, event.y);
		options.onTabDragEnd = function(panelId, event)
			interaction.endTabDrag(panelId, event.pointerId, event.x, event.y);
		options.onTabDragCancel = function(panelId, event)
			interaction.cancelTabDrag(panelId, event.pointerId);
		options.onTabHeaderBuilt = function(panelId, node) {
			interaction.registerTabTarget(new DockTabDropTarget(panelId, node));
			var indicatorStyle = new LayoutStyle();
			indicatorStyle.width = LayoutAxis.grow();
			indicatorStyle.height = LayoutAxis.grow();
			indicatorStyle.positioning = LayoutPositioning.Absolute;
			indicatorStyle.zIndex = 2;
			var indicator = new RenderNode(context.id("tab-drop-indicator:" + panelId),
				LayoutVisualKind.Custom, indicatorStyle);
			indicator.hitTestSelf = false;
			indicator.onPaint(function(canvas, geometry) {
				var preview = interaction.preview;
				if (preview == null || preview.targetPanelId != panelId ||
					(preview.zone != DockDropZone.TabBefore && preview.zone != DockDropZone.TabAfter))
					return;
				var indicatorWidth = Math.max(2.0, Math.min(4.0, geometry.width * 0.08));
				var x = preview.zone == DockDropZone.TabBefore ? 0.0 :
					Math.max(0.0, geometry.width - indicatorWidth);
				canvas.fillRectIfPositive(new Rect(x, 0.0, indicatorWidth, geometry.height),
					Color.rgba(0.18, 0.52, 0.95, 0.9));
			});
			node.add(indicator);
		};
		return nativekit.ui.widgets.controls.Tabs.withOptions(nodeKey, items, activePanelId, function(next) {
			model.activate(next);
		}, options);
	}

	function buildSplit(axis:DockSplitAxis, ratio:Float, first:DockNode, second:DockNode,
		context:BuildContext, path:Array<Int>, nodeKey:String,
		availableWidth:Float, availableHeight:Float):View {
		var firstPath = path.copy();
		firstPath.push(0);
		var secondPath = path.copy();
		secondPath.push(1);
		var horizontal = axis == DockSplitAxis.Horizontal;
		var available = horizontal ? availableWidth : availableHeight;
		if (available <= 0.0)
			available = 1000.0;
		var minimum = horizontal ? 190.0 : 120.0;
		var divider = 8.0;
		var maximum = available - minimum - divider;
		if (maximum < minimum)
			maximum = minimum;
		var options = new SplitViewOptions();
		options.orientation = horizontal ? SplitOrientation.Horizontal : SplitOrientation.Vertical;
		options.resizableSide = SplitSide.Leading;
		options.extent = clamp(ratio * available, minimum, maximum);
		options.minimumExtent = minimum;
		options.maximumExtent = maximum;
		options.dividerExtent = divider;
		options.onResize = function(next) {
			model.setSplitRatio(path, next / available);
		};
		var remaining = Math.max(0.0, available - options.extent - divider);
		var firstWidth = horizontal ? options.extent : availableWidth;
		var firstHeight = horizontal ? availableHeight : options.extent;
		var secondWidth = horizontal ? remaining : availableWidth;
		var secondHeight = horizontal ? availableHeight : remaining;
		return new SplitView(nodeKey,
			buildNode(first, context, firstPath, nodeKey + ":first", firstWidth, firstHeight),
			buildNode(second, context, secondPath, nodeKey + ":second", secondWidth, secondHeight),
			options);
	}

	function panelView(panelId:String):View {
		var descriptor = model.get(panelId);
		var content = panelContents.get(panelId);
		return descriptor == null ? new Text("Missing panel: " + panelId) : content == null ?
			new Text("Missing panel content: " + panelId) : new DockPanelView(descriptor, content);
	}

	function targetView(targetPanelId:String, child:View):View
		return new DockDropTargetView(child, interaction, targetPanelId);

	static function defaultStyle():LayoutStyle {
		var result = new LayoutStyle();
		result.width = LayoutAxis.grow();
		result.height = LayoutAxis.grow();
		return result;
	}

	static inline function clamp(value:Float, minimum:Float, maximum:Float):Float
		return value < minimum ? minimum : value > maximum ? maximum : value;
}

/** One subscription pair per mounted workspace, independent of rebuilt View instances. */
private class DockWorkspaceMount {
	public var model(default, null):DockWorkspaceModel;
	public var interaction(default, null):DockWorkspaceInteraction;
	public var invalidate:Null<Void->Void>;
	var stopModel:Null<Void->Void>;
	var stopInteraction:Null<Void->Void>;

	public function new(model:DockWorkspaceModel, interaction:DockWorkspaceInteraction) {
		this.model = model;
		this.interaction = interaction;
		subscribe();
	}

	public function bind(nextModel:DockWorkspaceModel,
			requestedInteraction:Null<DockWorkspaceInteraction>):Void {
		if (model == nextModel &&
			(requestedInteraction == null || interaction == requestedInteraction))
			return;
		unsubscribe();
		model = nextModel;
		interaction = requestedInteraction == null ? new DockWorkspaceInteraction(nextModel) : requestedInteraction;
		subscribe();
	}

	public function dispose():Void {
		unsubscribe();
		invalidate = null;
	}

	function subscribe():Void {
		var notify = function() { if (invalidate != null) invalidate(); };
		stopModel = model.listen(notify);
		stopInteraction = interaction.listen(notify);
	}

	function unsubscribe():Void {
		if (stopModel != null) stopModel();
		if (stopInteraction != null) stopInteraction();
		stopModel = null;
		stopInteraction = null;
	}
}

private class DockPanelView implements View {
	final descriptor:DockPanelDescriptor;
	final content:DockPanelContent;

	public function new(descriptor:DockPanelDescriptor, content:DockPanelContent) {
		this.descriptor = descriptor;
		this.content = content;
	}

	public function build(context:BuildContext):RenderNode {
		var observed = context.buildProbe != null;
		var preparationStarted = observed ? Sys.time() : 0.0;
		var view = content.build(context);
		if (view == null)
			view = new Text("Panel returned no content: " + descriptor.id);
		var buildStarted = observed ? Sys.time() : 0.0;
		var root = context.withScope(new Key("panel:" + descriptor.id), function() {
			return view.build(context);
		});
		if (observed)
			context.reportBuild("panel:" + descriptor.id, buildStarted - preparationStarted,
				Sys.time() - buildStarted, root);
		return root;
	}
}

private class DockDropTargetView implements View {
	final child:View;
	final interaction:DockWorkspaceInteraction;
	final targetPanelId:String;

	public function new(child:View, interaction:DockWorkspaceInteraction,
			targetPanelId:String) {
		if (child == null || interaction == null || targetPanelId == null)
			throw "Dock drop target views require a child, interaction, and target";
		this.child = child;
		this.interaction = interaction;
		this.targetPanelId = targetPanelId;
	}

	public function build(context:BuildContext):RenderNode {
		var node = child.build(context);
		interaction.registerTarget(new DockDropTarget(targetPanelId, node));
		var activePreview = interaction.preview;
		if (activePreview != null && activePreview.targetPanelId == targetPanelId) {
			var previewStyle = new LayoutStyle();
			previewStyle.width = LayoutAxis.grow();
			previewStyle.height = LayoutAxis.grow();
			previewStyle.positioning = LayoutPositioning.Absolute;
			previewStyle.zIndex = 2;
			var previewNode = new RenderNode(context.id("drop-preview:" + targetPanelId),
				LayoutVisualKind.Custom, previewStyle);
			previewNode.setStyleIdentity("dock-drop-preview", targetPanelId);
			previewNode.hitTestSelf = false;
			previewNode.onPaint(function(canvas, geometry) {
				var preview = interaction.preview;
				if (preview == null || preview.targetPanelId != targetPanelId)
					return;
				canvas.fillRectIfPositive(previewRect(preview.zone, geometry.width, geometry.height),
					Color.rgba(0.18, 0.52, 0.95, 0.22));
			});
			node.add(previewNode);
		}
		return node;
	}

	static function previewRect(zone:DockDropZone, width:Float, height:Float):Rect {
		return switch (zone) {
			case DockDropZone.Center: new Rect(width * 0.2, height * 0.2, width * 0.6, height * 0.6);
			case DockDropZone.TabBefore: new Rect(0.0, 0.0, Math.max(3.0, width * 0.08), height);
			case DockDropZone.TabAfter: new Rect(Math.max(0.0, width * 0.92), 0.0,
				Math.max(3.0, width * 0.08), height);
			case DockDropZone.Left: new Rect(0.0, 0.0, width * 0.25, height);
			case DockDropZone.Right: new Rect(width * 0.75, 0.0, width * 0.25, height);
			case DockDropZone.Top: new Rect(0.0, 0.0, width, height * 0.25);
			case DockDropZone.Bottom: new Rect(0.0, height * 0.75, width, height * 0.25);
		};
	}
}
