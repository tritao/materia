package app;

import nativekit.ui.core.View;
import nativekit.ui.widgets.text.Text;
import nativekit.ui.widgets.collections.TreeRootMetadata;
import nativekit.ui.widgets.collections.TreeViewModel;

class EditorSceneTree implements TreeViewModel {
  final scene:EditorScene;
  public function new(scene:EditorScene) this.scene = scene;
  public function rootCount():Int return 1;
  public function rootRange(start:Int, count:Int):Array<TreeRootMetadata>
    return start <= 0 && count > 0 ? [new TreeRootMetadata("scene", true)] : [];
  public function rootKeyAt(index:Int):String return "scene";
  public function childCount(parentKey:String):Int return parentKey == "scene" ? scene.items().length
    : scene.cadFeatureCount(parentKey);
  public function childKeyAt(parentKey:String, index:Int):String return parentKey=="scene"
    ? scene.items()[index].id : parentKey+":feature:"+index;
  public function initiallyExpanded(key:String):Bool {
    if(key=="scene")return true;
    var item=scene.object(key);
    return item!=null&&scene.isCadPart(item.id);
  }
  public function estimatedExtent():Float return 28.0;
  public function extentIsUniform():Bool return true;
  public function extentAt(key:String):Float return 28.0;
  public function buildItem(key:String):View {
    var item=scene.object(key);
    if(item!=null)return new Text(item.label + (item.visible ? "" : " (hidden)"));
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
