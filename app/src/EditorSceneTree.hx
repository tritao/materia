package app;

import nativekit.ui.core.View;
import nativekit.ui.widgets.Text;
import nativekit.ui.widgets.TreeRootMetadata;
import nativekit.ui.widgets.TreeViewModel;

class EditorSceneTree implements TreeViewModel {
  final scene:EditorScene;
  public function new(scene:EditorScene) this.scene = scene;
  public function rootCount():Int return 1;
  public function rootRange(start:Int, count:Int):Array<TreeRootMetadata>
    return start <= 0 && count > 0 ? [new TreeRootMetadata("scene", true)] : [];
  public function rootKeyAt(index:Int):String return "scene";
  public function childCount(parentKey:String):Int return parentKey == "scene" ? scene.items().length : 0;
  public function childKeyAt(parentKey:String, index:Int):String return scene.items()[index].id;
  public function initiallyExpanded(key:String):Bool return key == "scene";
  public function estimatedExtent():Float return 28.0;
  public function extentIsUniform():Bool return true;
  public function extentAt(key:String):Float return 28.0;
  public function buildItem(key:String):View {
    var item = scene.object(key);
    return new Text(item == null ? "Scene" : item.label + (scene.info(key).visible() ? "" : " (hidden)"));
  }
  public function revision():Int return scene.revision;
}
