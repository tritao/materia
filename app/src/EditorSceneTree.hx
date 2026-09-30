package app;

import nativekit.ui.core.View;
import nativekit.ui.widgets.text.Text;
import nativekit.ui.widgets.text.MiddleEllipsisText;
import nativekit.ui.widgets.overlays.Tooltip;
import nativekit.ui.widgets.Icon;
import nativekit.ui.widgets.KeyedView;
import nativekit.ui.widgets.layout.Row;
import LayoutAlignmentY;
import LayoutAxis;
import LayoutStyle;
import nativekit.ui.icons.IconName;
import nativekit.ui.widgets.collections.TreeRootMetadata;
import nativekit.ui.widgets.collections.TreeViewModel;
import materia.assembly.AssemblyBodies;
import materia.assembly.AssemblyDefinition;

/**
 * The scene hierarchy. An assembly appears as its rigid bodies, not as its raw joint tree: parts held
 * together by fixed joints share one body, and only moving joints nest a body inside another. A body
 * is shown as its root part, with its remaining parts in a collapsed "Parts" node, which opens by
 * itself when one of them is selected.
 */
class EditorSceneTree implements TreeViewModel {
  static inline var PARTS:String = "parts:";
  static inline var PROJECT:String = "project:";
  final scene:EditorScene;
  final generatedLabels:Map<String, String>;
  /** Body roots and Parts nodes to what they contain. */
  final children:Map<String, Array<String>> = [];
  /** Part to the Parts node that holds it. */
  final partsOf:Map<String, String> = [];
  final partsTitles:Map<String, String> = [];
  /** Body root to the kind of moving joint that carries it. */
  final jointOf:Map<String, String> = [];
  /** Assembly objects that hang below another row instead of standing at the top. */
  final nested:Map<String, Bool> = [];
  final childCache:Map<String, Array<String>> = [];
  var cachedSceneRevision:Int = -1;
  var cachedFilterRevision:Int = -1;
  var filter:String = "";
  var filterRevision:Int = 0;
  var revealed:String = "";
  var revealSerial:Int = 0;
  public function setFilter(value:String):Void {
    var next = StringTools.trim(value == null ? "" : value).toLowerCase();
    if (next != filter) { filter = next; filterRevision++; }
  }
  /** Parts nodes group rows; they are not scene objects and cannot be selected. */
  public function isGroup(key:String):Bool return StringTools.startsWith(key, PARTS);
  /** What the row for a scene object says: a readable name for generated parts, or the person's rename. */
  function displayLabel(key:String):String {
    var item = scene.object(key);
    if (item == null) return key;
    var generated = generatedLabels.get(key);
    if (generated != null && generated == item.label && StringTools.startsWith(key, PROJECT))
      return AssemblyBodies.displayName(key.substr(PROJECT.length));
    return item.label;
  }
  function matches(key:String):Bool {
    if (filter == "") return true;
    var group = children.get(key);
    if (isGroup(key)) {
      if (group != null) for (child in group) if (matches(child)) return true;
      return false;
    }
    var item = scene.object(key);
    if (item != null) {
      if (item.label.toLowerCase().indexOf(filter) >= 0 || displayLabel(key).toLowerCase().indexOf(filter) >= 0 ||
          item.kind.toLowerCase().indexOf(filter) >= 0) return true;
      if (group != null) for (child in group) if (matches(child)) return true;
      for (index in 0...scene.cadFeatureCount(key))
        if (scene.cadFeatureNameAt(key, index).toLowerCase().indexOf(filter) >= 0) return true;
      return false;
    }
    return key == "scene";
  }
  function visibleChildren(parentKey:String):Array<String> {
    if (cachedSceneRevision != scene.revision || cachedFilterRevision != filterRevision) {
      childCache.clear();
      cachedSceneRevision = scene.revision;
      cachedFilterRevision = filterRevision;
    }
    var cached = childCache.get(parentKey);
    if (cached != null) return cached;
    var result:Array<String> = [];
    if (parentKey == "scene") {
      for (item in scene.items()) if (!nested.exists(item.id) && matches(item.id)) result.push(item.id);
    } else {
      var members = children.get(parentKey);
      if (members != null) for (child in members) if (matches(child)) result.push(child);
      if (!isGroup(parentKey))
        for (index in 0...scene.cadFeatureCount(parentKey)) {
          var key = parentKey + ":feature:" + index;
          if (filter == "" || scene.cadFeatureNameAt(parentKey, index).toLowerCase().indexOf(filter) >= 0)
            result.push(key);
        }
    }
    childCache.set(parentKey, result);
    return result;
  }
  /**
   * `definition` groups the assembly into bodies; `generatedLabels` are the labels the project gave its
   * objects, so a name the person typed is shown as typed.
   */
  public function new(scene:EditorScene, ?definition:AssemblyDefinition, ?generatedLabels:Map<String, String>) {
    this.scene = scene;
    this.generatedLabels = generatedLabels == null ? new Map() : generatedLabels;
    if (definition == null) return;
    var bodies = AssemblyBodies.of(definition);
    for (body in bodies) {
      var rootKey = PROJECT + body.id;
      if (scene.object(rootKey) == null) continue;
      var members:Array<String> = [];
      for (occurrence in body.occurrences.slice(1)) {
        var partKey = PROJECT + occurrence;
        if (scene.object(partKey) != null) members.push(partKey);
      }
      var list:Array<String> = [];
      if (members.length > 0) {
        var partsKey = PARTS + body.id;
        children.set(partsKey, members);
        partsTitles.set(partsKey, partsTitle(members));
        for (member in members) {
          partsOf.set(member, partsKey);
          nested.set(member, true);
        }
        list.push(partsKey);
      }
      children.set(rootKey, list);
      if (body.jointType != null) jointOf.set(rootKey, Std.string(body.jointType));
    }
    // A body hangs below the body that its moving joint is attached to.
    for (body in bodies) {
      var rootKey = PROJECT + body.id;
      if (body.parent == null || !children.exists(rootKey)) continue;
      var carrier = children.get(PROJECT + body.parent);
      if (carrier == null) continue;
      carrier.push(rootKey);
      nested.set(rootKey, true);
    }
  }
  /** "Tool (6)" when every part shares one nested path such as tool/..., otherwise "Parts (n)". */
  static function partsTitle(members:Array<String>):String {
    var shared:Null<String> = null;
    for (member in members) {
      var id = member.substr(PROJECT.length);
      var slash = id.indexOf("/");
      var first = slash < 0 ? "" : id.substr(0, slash);
      if (first == "" || (shared != null && shared != first)) return "Parts (" + members.length + ")";
      shared = first;
    }
    return AssemblyBodies.displayName(shared == null ? "parts" : shared) + " (" + members.length + ")";
  }
  /** The Parts node that holds the selected part, so it can open to show it. */
  function revealedGroup():String {
    var group = partsOf.get(scene.selectedId);
    return group == null ? "" : group;
  }
  public function rootCount():Int return 1;
  public function rootRange(start:Int, count:Int):Array<TreeRootMetadata>
    return start <= 0 && count > 0 ? [new TreeRootMetadata("scene", true)] : [];
  public function rootKeyAt(index:Int):String return "scene";
  public function childCount(parentKey:String):Int {
    return visibleChildren(parentKey).length;
  }
  public function childKeyAt(parentKey:String, index:Int):String {
    var result = visibleChildren(parentKey);
    if (index < 0 || index >= result.length) throw "Scene tree child index is out of range";
    return result[index];
  }
  public function initiallyExpanded(key:String):Bool {
    if (filter != "") return true;
    if (key == "scene") return true;
    if (isGroup(key)) return revealedGroup() == key;
    var item = scene.object(key);
    return item != null && (scene.isCadPart(item.id) || children.exists(item.id));
  }
  public function estimatedExtent():Float return 28.0;
  public function extentIsUniform():Bool return true;
  public function extentAt(key:String):Float return 28.0;
  function labeledItem(key:String, label:String, ?detail:String):View {
    var text = new MiddleEllipsisText("short-label:" + key, label);
    var tooltip = new Tooltip("label-tooltip:" + key, text, new Text(detail == null ? label : detail));
    tooltip.fillAnchor = true;
    tooltip.showWhen = function() return detail != null || text.truncated;
    return tooltip;
  }
  function iconLabeledItem(key:String, label:String, icon:IconName, ?detail:String):View {
    var rowStyle = new LayoutStyle();
    rowStyle.width = LayoutAxis.grow();
    rowStyle.childAlignY = LayoutAlignmentY.Center;
    rowStyle.childGap = 6.0;
    return new Row("item-row:" + key, [
      new KeyedView("icon", new Icon("item-icon:" + key, icon, 15.0)),
      new KeyedView("label", labeledItem(key, label, detail))
    ], rowStyle);
  }
  public function buildItem(key:String):View {
    if (isGroup(key)) {
      var title = partsTitles.get(key);
      return iconLabeledItem(key, title == null ? "Parts" : title, IconName.Grid);
    }
    var item=scene.object(key);
    if(item!=null) {
      var joint = jointOf.get(key);
      var shown = displayLabel(key);
      // The generated designation stays one hover away when the row shows a readable name.
      return iconLabeledItem(key, shown + (joint == null ? "" : " · " + joint) +
        (item.visible ? "" : " (hidden)"), IconName.Cube, shown == item.label ? null : item.label);
    }
    var marker=key.indexOf(":feature:");
    if(marker>=0){
      var id=key.substr(0,marker),index=Std.parseInt(key.substr(marker+9));
      return iconLabeledItem(key, index==null||index<0||index>=scene.cadFeatureCount(id)?"Feature":
        "Feature "+(index+1)+" · "+scene.cadFeatureNameAt(id,index), IconName.Grid);
    }
    return iconLabeledItem(key, "Scene", IconName.Hierarchy);
  }
  public function revision():Int {
    var group = revealedGroup();
    if (group != revealed) {
      revealed = group;
      revealSerial++;
    }
    return scene.revision + filterRevision * 1000000 + revealSerial;
  }
}
