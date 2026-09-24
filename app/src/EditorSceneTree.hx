package app;

import nativekit.ui.core.View;
import nativekit.ui.widgets.text.Text;
import nativekit.ui.widgets.collections.TreeRootMetadata;
import nativekit.ui.widgets.collections.TreeViewModel;
import materia.project.AssemblyRecord;
import materia.project.AssemblyRecord.AssemblyJoint;

class EditorSceneTree implements TreeViewModel {
  final scene:EditorScene;
  final children:Map<String, Array<String>> = [];
  final incoming:Map<String, AssemblyJoint> = [];
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
    if (parentKey == "scene") {
      var count = 0;
      for (item in scene.items()) if (!incoming.exists(item.id)) count++;
      return count;
    }
    var nested = children.get(parentKey);
    return (nested == null ? 0 : nested.length) + scene.cadFeatureCount(parentKey);
  }
  public function childKeyAt(parentKey:String, index:Int):String {
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
    if(key=="scene")return true;
    var item=scene.object(key);
    return item!=null&&(scene.isCadPart(item.id)||children.exists(item.id));
  }
  public function estimatedExtent():Float return 28.0;
  public function extentIsUniform():Bool return true;
  public function extentAt(key:String):Float return 28.0;
  public function buildItem(key:String):View {
    var item=scene.object(key);
    if(item!=null) {
      var joint = incoming.get(item.id);
      return new Text(item.label + (joint == null ? "" : " · " + joint.kind) +
        (item.visible ? "" : " (hidden)"));
    }
    var marker=key.indexOf(":feature:");
    if(marker>=0){
      var id=key.substr(0,marker),index=Std.parseInt(key.substr(marker+9));
      return new Text(index==null||index<0||index>=scene.cadFeatureCount(id)?"Feature":
        "Feature "+(index+1)+" · "+scene.cadFeatureNameAt(id,index));
    }
    return new Text("Scene");
  }
  public function revision():Int return scene.revision;
}
