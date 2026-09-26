package app;

import nativekit.ui.properties.PropertyDescriptor;
import nativekit.ui.widgets.controls.Select;


import bimkit.BimDocument;
import nativekit.ui.editing.EditorDocument;
import cadkit.parametric.Definition;
import cadkit.parametric.Element;
import cadkit.parametric.ElementId;
import cadkit.parametric.InstanceElement;
import LayoutAxis;
import LayoutStyle;
import Insets;
import TextWrap;
import nativekit.ui.core.BuildContext;
import nativekit.ui.core.RenderNode;
import nativekit.ui.core.TextStyleOverride;
import nativekit.ui.core.View;
import nativekit.ui.widgets.controls.Button;
import nativekit.ui.widgets.controls.ButtonVariant;
import nativekit.ui.widgets.controls.IconButton;
import nativekit.ui.icons.IconName;
import nativekit.ui.widgets.layout.Column;
import nativekit.ui.widgets.KeyedView;
import nativekit.ui.widgets.properties.PropertyInspector;
import nativekit.ui.widgets.layout.Row;
import nativekit.ui.widgets.text.Text;
import nativekit.ui.widgets.collections.TreeView;
import nativekit.ui.widgets.scroll.ScrollView;

/** Reusable editor surface combining spatial/type trees with the generic inspector. */
class BimModelEditor implements View {
	public final key:String;
	public final model:BimDocument;
	final projectDocument:Null<EditorDocument>;
	final applyEdit:Null<BimProjectEdit>;
	public final spatialTree:BimSpatialTree;
	public final typeTree:BimTypeTree;
	public var showingTypes(default, null):Bool;
	private var selectedElementId:Null<String>;
	private var selectedDefinitionId:Null<String>;
	private var panelWidth:Float = 0.0;
	private static inline final COMPACT_WIDTH:Float = 620.0;

	public function new(key:String, model:BimDocument, ?projectDocument:EditorDocument, ?applyEdit:BimProjectEdit) {
		if (key == null || key.length == 0 || model == null)
			throw "BIM editor requires a stable key and a document";
		this.key = key;
		this.model = model;
		this.projectDocument = projectDocument;
		this.applyEdit = applyEdit;
		spatialTree = new BimSpatialTree(model);
		typeTree = new BimTypeTree(model);
		showingTypes = false;
		selectedElementId = null;
		selectedDefinitionId = null;
	}

	public function showSpatialTree():Void {
		showingTypes = false;
		selectedDefinitionId = null;
	}

	public function showTypeTree():Void {
		showingTypes = true;
		selectedElementId = null;
	}

	public function build(context:BuildContext):RenderNode {
		var compact = panelWidth < COMPACT_WIDTH;
		var toolbarStyle = new LayoutStyle();
		toolbarStyle.width = LayoutAxis.grow();
		toolbarStyle.childGap = 4.0;
		toolbarStyle.padding = new Insets(6.0, 4.0, 6.0, 4.0);
		var toolbar = new Row(key + ":tree-mode", [
			new KeyedView("spatial", modeButton("Spatial", false, context)),
			new KeyedView("types", modeButton("Types", true, context)),
			new KeyedView("undo", new IconButton(key + ":undo", IconName.Undo, "Undo BIM", function() {
				if (projectDocument == null) model.undo(); else projectDocument.undo();
				context.commands.refresh();
			})),
			new KeyedView("redo", new IconButton(key + ":redo", IconName.Redo, "Redo BIM", function() {
				if (projectDocument == null) model.redo(); else projectDocument.redo();
				context.commands.refresh();
			}))
		], toolbarStyle);
		var treeStyle = new LayoutStyle();
		treeStyle.width = compact ? LayoutAxis.grow() : LayoutAxis.fixed(280.0);
		treeStyle.height = LayoutAxis.grow();
		var tree:TreeView;
		if (showingTypes) {
			var selected = selectedTypeKey();
			tree = new TreeView(key + ":type-tree", typeTree, treeStyle, null, 400, selected, null, function(nodeKey) {
				selectedDefinitionId = typeTree.definitionIdForKey(nodeKey);
				var instanceId = typeTree.instanceIdForKey(nodeKey);
				selectedElementId = instanceId;
				context.commands.refresh();
			});
		} else {
			var selected = selectedSpatialKey();
			tree = new TreeView(key + ":spatial-tree", spatialTree, treeStyle, null, 400, selected, null, function(nodeKey) {
				var selectedId = spatialTree.elementIdForKey(nodeKey);
				selectedElementId = selectedId == null ? null : selectedId.value;
				selectedDefinitionId = null;
				context.commands.refresh();
			});
		}
		var inspector = buildInspector();
		var inspectorStyle = new LayoutStyle();
		inspectorStyle.width = LayoutAxis.grow();
		inspectorStyle.height = compact ? LayoutAxis.fixed(220.0) : LayoutAxis.grow();
		inspectorStyle.padding = new Insets(6.0, 6.0, 6.0, 6.0);
		inspectorStyle.clipHorizontal = true;
		var inspectorScroll = new ScrollView(key + ":inspector-scroll", inspector, inspectorStyle);
		var contentStyle = new LayoutStyle();
		contentStyle.width = LayoutAxis.grow();
		contentStyle.height = LayoutAxis.grow();
		contentStyle.childGap = 8.0;
		var contentItems:Array<KeyedView> = [
			new KeyedView("tree", tree),
			new KeyedView("inspector", inspectorScroll)
		];
		var content:View = compact ? new Column(key + ":content", contentItems, contentStyle)
			: new Row(key + ":content", contentItems, contentStyle);
		var rootStyle = new LayoutStyle();
		rootStyle.width = LayoutAxis.grow();
		rootStyle.height = LayoutAxis.grow();
		rootStyle.clipHorizontal = true;
		var root = new Column(key, [new KeyedView("mode", toolbar), new KeyedView("content", content)], rootStyle).build(context);
		root.onResolved(function(geometry) {
			if ((panelWidth < COMPACT_WIDTH) != (geometry.width < COMPACT_WIDTH)) {
				panelWidth = geometry.width;
				context.commands.refresh();
			} else panelWidth = geometry.width;
		});
		return root;
	}

	private function modeButton(label:String, types:Bool, context:BuildContext):Button {
		var result = new Button(label, null, function() {
			if (types)
				showTypeTree();
			else
				showSpatialTree();
			context.commands.refresh();
		}, key + ":mode:" + label.toLowerCase());
		result.selected = showingTypes == types;
		result.variant = ButtonVariant.Secondary;
		return result;
	}

	private function buildInspector():View {
		var descriptors = inspectorDescriptors();
		if (descriptors.length == 0)
			return new Text("Select a BIM object or type.", null, null,
				TextStyleOverride.paragraph(TextWrap.Word));
		return new PropertyInspector(key + ":inspector", descriptors, null, null, null, null,
			showingTypes ? "BIM Type Properties" : "BIM Object Properties");
	}

	private function inspectorDescriptors():Array<nativekit.ui.properties.PropertyDescriptor> {
		var result:Array<nativekit.ui.properties.PropertyDescriptor> = [];
		if (selectedDefinitionId != null) {
			var definition = findDefinition(selectedDefinitionId);
			if (definition != null)
				result = BimInspectorDescriptors.forDefinition(definition, applyEdit);
			return result;
		}
		if (selectedElementId == null)
			return result;
		var element = model.cad.findElement(new ElementId(selectedElementId));
		if (element == null)
			return result;
		result = BimInspectorDescriptors.forElement(element, applyEdit);
		if (element.kind == "instance") {
			var instance:InstanceElement = cast element;
			result = result.concat(BimInspectorDescriptors.forDefinition(model.cad.definition(instance.definitionId), applyEdit));
		}
		return result;
	}

	private function selectedSpatialKey():Null<String> {
		if (selectedElementId != null) {
			var candidate = BimSpatialTree.ElementPrefix + selectedElementId;
			if (model.cad.findElement(new ElementId(selectedElementId)) != null)
				return candidate;
		}
		return spatialTree.rootCount() == 0 ? null : spatialTree.rootKeyAt(0);
	}

	private function selectedTypeKey():Null<String> {
		if (selectedDefinitionId != null) {
			var candidate = BimTypeTree.DefinitionPrefix + selectedDefinitionId;
			if (findDefinition(selectedDefinitionId) != null)
				return candidate;
		}
		if (selectedElementId != null) {
			var candidate = BimTypeTree.InstancePrefix + selectedElementId;
			if (model.cad.findElement(new ElementId(selectedElementId)) != null)
				return candidate;
		}
		return typeTree.rootCount() == 0 ? null : typeTree.rootKeyAt(0);
	}

	private function findDefinition(id:String):Null<Definition> {
		for (definition in model.cad.allDefinitions())
			if (definition.id.value == id)
				return definition;
		return null;
	}
}
