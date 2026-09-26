package app;

import nativekit.ui.core.View;
import nativekit.ui.widgets.text.Text;
import nativekit.ui.widgets.text.MiddleEllipsisText;
import nativekit.ui.widgets.overlays.Tooltip;
import nativekit.ui.widgets.Icon;
import nativekit.ui.widgets.KeyedView;
import nativekit.ui.widgets.layout.Row;
import nativekit.ui.icons.IconName;
import nativekit.ui.widgets.collections.TreeRootMetadata;
import nativekit.ui.widgets.collections.TreeViewModel;
import materia.project.AssemblyRecord;
import materia.project.AssemblyRecord.AssemblyJoint;

class EditorSceneTree implements TreeViewModel {
  final scene:EditorScene;
  final children:Map<String, Array<String>> = [];
  final incoming:Map<String, AssemblyJoint> = [];
  var filter:String = "";
  var filterRevision:Int = 0;
  public function setFilter(value:String):Void {
    var next = StringTools.trim(value == null ? "" : value).toLowerCase();
    if (next != filter) { filter = next; filterRevision++; }
  }
  function matches(key:String):Bool {
    if (filter == "") return true;
    var item = scene.object(key);
    if (item != null) {
      if (item.label.toLowerCase().indexOf(filter) >= 0 || item.kind.toLowerCase().indexOf(filter) >= 0) return true;
      var nested = children.get(key);
      if (nested != null) for (child in nested) if (matches(child)) return true;
      for (index in 0...scene.cadFeatureCount(key))
        if (scene.cadFeatureNameAt(key, index).toLowerCase().indexOf(filter) >= 0) return true;
      return false;
    }
    return key == "scene";
  }
  function visibleChildren(parentKey:String):Array<String> {
    var result:Array<String> = [];
    if (parentKey == "scene") {
      for (item in scene.items()) if (!incoming.exists(item.id) && matches(item.id)) result.push(item.id);
    } else {
      var nested = children.get(parentKey);
      if (nested != null) for (child in nested) if (matches(child)) result.push(child);
      for (index in 0...scene.cadFeatureCount(parentKey)) {
        var key = parentKey + ":feature:" + index;
        if (filter == "" || scene.cadFeatureNameAt(parentKey, index).toLowerCase().indexOf(filter) >= 0)
          result.push(key);
      }
    }
    return result;
  }
  public function new(scene:EditorScene, ?assembly:AssemblyRecord) {
    this.scene = scene;
    if (assembly == null) return;
    for (joint in assembly.joints) {
      var child = "project:" + joint.child, parent = "project:" + joint.parent;
      if (incoming.exists(child) || scene.object(child) == null || scene.object(parent) == null) continue;
      incoming.set(child, joint);
      var siblings = children.get(parent);
      if (siblings == null) {siblings = []; children.set(parent, siblings);}
      siblings.push(child);
    }
  }
  public function rootCount():Int return 1;
  public function rootRange(start:Int, count:Int):Array<TreeRootMetadata>
    return start <= 0 && count > 0 ? [new TreeRootMetadata("scene", true)] : [];
  public function rootKeyAt(index:Int):String return "scene";
  public function childCount(parentKey:String):Int {
    if (filter != "") return visibleChildren(parentKey).length;
    if (parentKey == "scene") {
      var count = 0;
      for (item in scene.items()) if (!incoming.exists(item.id)) count++;
      return count;
    }
    var nested = children.get(parentKey);
    return (nested == null ? 0 : nested.length) + scene.cadFeatureCount(parentKey);
  }
  public function childKeyAt(parentKey:String, index:Int):String {
    if (filter != "") return visibleChildren(parentKey)[index];
    if (parentKey == "scene") {
      for (item in scene.items()) if (!incoming.exists(item.id)) {
        if (index == 0) return item.id;
        index--;
      }
      throw "Scene tree root child index is out of range";
    }
    var nested = children.get(parentKey);
    if (nested != null) {
      if (index < nested.length) return nested[index];
      index -= nested.length;
    }
    return parentKey+":feature:"+index;
  }
  public function initiallyExpanded(key:String):Bool {
    if (filter != "") return true;
    if(key=="scene")return true;
    var item=scene.object(key);
    return item!=null&&(scene.isCadPart(item.id)||children.exists(item.id));
  }
  public function estimatedExtent():Float return 28.0;
  public function extentIsUniform():Bool return true;
  public function extentAt(key:String):Float return 28.0;
  function labeledItem(key:String, label:String):View {
    var text = new MiddleEllipsisText("short-label:" + key, label);
    var tooltip = new Tooltip("label-tooltip:" + key, text, new Text(label));
    tooltip.fillAnchor = true;
    tooltip.showWhen = function() return text.truncated;
    return tooltip;
  }
  public function buildItem(key:String):View {
    var item=scene.object(key);
    if(item!=null) {
      var joint = incoming.get(item.id);
      return new Row("object-row:" + key, [
        new KeyedView("icon", new Icon("object-icon:" + key,
          scene.isCadPart(item.id) ? IconName.Cube : IconName.Hierarchy, 15.0)),
        new KeyedView("label", labeledItem(key, item.label + (joint == null ? "" : " · " + joint.kind) +
          (item.visible ? "" : " (hidden)")))
      ]);
    }
    var marker=key.indexOf(":feature:");
    if(marker>=0){
      var id=key.substr(0,marker),index=Std.parseInt(key.substr(marker+9));
      return labeledItem(key, index==null||index<0||index>=scene.cadFeatureCount(id)?"Feature":
        "Feature "+(index+1)+" · "+scene.cadFeatureNameAt(id,index));
    }
    return labeledItem(key, "Scene");
  }
  public function revision():Int return scene.revision + filterRevision * 1000000;
}
