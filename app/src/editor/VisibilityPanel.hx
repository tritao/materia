package app.editor;

import app.EditorSceneTree;
import haxeon.ui.*;
import haxeon.ui.core.*;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.collections.*;
import haxeon.ui.widgets.controls.*;
import haxeon.ui.widgets.layout.*;
import haxeon.ui.widgets.text.Text;
import haxeon.ui.widgets.overlays.Popup;

/** A second view of the scene tree, with independent expansion and no selection side effects. */
class VisibilityPanel implements TreeViewModel {
 final tree:EditorSceneTree;
 final enabled:Bool;
 public function new(tree:EditorSceneTree,enabled:Bool=true){this.tree=tree;this.enabled=enabled;}
 public function rootCount():Int return 1;
 public function rootRange(start:Int,count:Int):Array<TreeRootMetadata> return start<=0&&count>0?[new TreeRootMetadata("scene",true)]:[];
 public function rootKeyAt(index:Int):String return "scene";
 public function childCount(key:String):Int return tree.visibilityChildren(key).length;
 public function childKeyAt(key:String,index:Int):String return tree.visibilityChildren(key)[index];
 // Show primary assemblies on first open; deeper ownership remains independently expandable.
 public function initiallyExpanded(key:String):Bool return tree.objectPath(key).length<=3;
 public function estimatedExtent():Float return 32;
 public function extentIsUniform():Bool return true;
 public function extentAt(key:String):Float return 32;
 public function revision():Int return tree.revision();
 public function buildItem(key:String):View {
  var state=tree.visibilityState(key),label=tree.visibilityLabel(key);
  var check=new Checkbox("visibility-check:"+key,label,state.shown,_-> {tree.toggleVisibility(key);});
  check.enabled=enabled;check.indeterminate=state.mixed;
  check.accessibilityLabel=label+(!state.shown&&state.own?" · Hidden by parent":"");
  return check;
 }
 public static function popup(tree:EditorSceneTree,x:Float,y:Float,width:Float,height:Float,enabled:Bool,dismiss:Void->Void):View {
  var body=new LayoutStyle();body.width=LayoutAxis.fixed(Math.min(340,Math.max(200,width-16)));body.childGap=6;body.padding=new Insets(10,10,10,10);
  var show=new Button("Show all",null,()->{tree.showAll();},"visibility-show-all");show.variant=ButtonVariant.Secondary;show.enabled=enabled;
  var isolate=new Button("Isolate selected",null,()->{tree.isolateSelected();},"visibility-isolate");isolate.variant=ButtonVariant.Secondary;isolate.enabled=enabled&&tree.selectionKey()!="scene";
  var listStyle=new LayoutStyle();listStyle.width=LayoutAxis.grow();listStyle.height=LayoutAxis.fixed(Math.min(380,Math.max(100,height-150)));
  var list=new TreeView("visibility-components",new VisibilityPanel(tree,enabled),listStyle,null,380,null,["scene"]);
  var content=new Column("visibility-content",[
   new KeyedView("title",new Text("Component visibility")),
   new KeyedView("actions",new Row("visibility-actions",[new KeyedView("show",show),new KeyedView("isolate",isolate)])),
   new KeyedView("components",list)],body);
  var popup=new Popup("component-visibility",content,x,y,null,dismiss);
  popup.anchorRectProvider=()->new Rect(x,y,0,0);
  popup.label="Component visibility";popup.dimBackdrop=false;popup.menuSurface=true;popup.viewportMargin=8;
  return popup;
 }
}
