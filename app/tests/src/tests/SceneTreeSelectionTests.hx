package tests;

import app.EditorScene;
import app.EditorSceneTree;
import haxeon.ui.core.UiContext;
import haxeon.ui.core.RenderNode;
import haxeon.ui.FontCollection;
import haxeon.ui.LayoutStyle;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutFrame;
import haxeon.ui.widgets.collections.TreeView;
import haxeon.ui.semantics.AccessibilityRole;
import haxeon.ui.semantics.AccessibilityState;
import sys.FileSystem;

/** Repeated builds intentionally recreate TreeView, as the retained Hierarchy dock does. */
@:access(tests.SceneVisibilityTests)
class SceneTreeSelectionTests {
  static var assertions=0;
  static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw label;}
  static function find(root:RenderNode,key:String):Null<RenderNode> {
    if(root.semantics!=null && root.semantics.role==AccessibilityRole.TreeItem && root.semantics.label==key)return root;
    for(child in root.children){var row=find(child,key);if(row!=null)return row;}return null;
  }
  static function selected(root:RenderNode,key:String):Bool {
    var node=find(root,key);
    return node!=null && node.semantics!=null && (node.semantics.states & AccessibilityState.Selected)!=0;
  }
  public static function main():Int {run();return 0;}
  public static function run():Void {
    var fonts=FontCollection.create(), font:Null<String> = null;
    for(path in ["haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf",
      "../../haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf"])if(FileSystem.exists(path))font=path;
    if(font==null)throw "Missing selection test font";
    fonts.add(font);
    var ui=new UiContext(null,fonts), data=SceneVisibilityTests.data();
    for(i in 0...80) {
      var record=SceneVisibilityTests.data()[0];record.id='filler-$i';record.label='Filler $i';data.insert(i,record);
    }
    var scene=new EditorScene(data), model=new EditorSceneTree(scene,SceneVisibilityTests.definition());
    var callbacks=0, frame=new LayoutFrame(400,180);
    function make(key:String="selection-tree"):TreeView {
      var style=new LayoutStyle();style.width=LayoutAxis.fixed(400);style.height=LayoutAxis.fixed(180);
      var tree=new TreeView(key,model,style,model.scrollController,180,model.selectionKey(),["scene"],function(id){callbacks++;model.selectKey(id);});
      tree.onStateReady=model.synchronizeSelection;
      return tree;
    }
    try {
      scene.select("filler-0");
      var tree=make(), root=ui.submit(tree,frame);
      check(selected(root,"filler-0"),"initial controlled selection");
      tree.setExpanded("scene",false);
      ui.submit(tree,frame);
      scene.select("project:tool");
      tree=make();root=ui.submit(tree,frame);
      check(tree.isExpanded("scene") && tree.isExpanded("project:base") && tree.isExpanded("project:slide") && tree.isExpanded("parts:slide"),
        "3D selection opens all collapsed ancestors");
      check(selected(root,"project:tool") && tree.selectedKey=="project:tool","viewport selection overrides retained tree selection");
      check(tree.controller.offsetY>1000,"selected offscreen row is scrolled into the viewport");
      check(callbacks==0,"programmatic synchronization never fires a user selection callback");
      check(ui.focus.focusedId==null,"reveal does not steal keyboard focus from the viewport");
      var offset=tree.controller.offsetY;
      tree=make();ui.submit(tree,frame);
      check(tree.controller.offsetY==offset,"unchanged selection leaves scroll alone");
      tree.setExpanded("parts:slide",false);
      tree=make();ui.submit(tree,frame);
      check(!tree.isExpanded("parts:slide"),"explicit collapse remains closed until a new selection");
      scene.select("project:cover");tree=make();root=ui.submit(tree,frame);
      check(selected(root,"project:cover") && tree.isExpanded("parts:base"),"select another part in a different Parts group");
      scene.select("project:bracket");tree=make();root=ui.submit(tree,frame);
      check(selected(root,"project:bracket"),"selection within one Parts group also refreshes the row");
      model.setFilter("cover");tree=make("selection-filtered");ui.submit(tree,frame);
      scene.select("project:tool");tree=make("selection-filtered");root=ui.submit(tree,frame);
      check(selected(root,"project:tool"),"viewport-picked row remains reachable through the active filter");
      check(model.childCount("parts:base")==1,"selection reveal preserves the search filter for other rows");
      model.setFilter("");tree=make();root=ui.submit(tree,frame);
      check(selected(root,"project:tool"),"clearing search restores the current selection rather than stale retained state");
      scene.select("scene");tree=make();root=ui.submit(tree,frame);
      check(selected(root,"scene"),"clearing 3D selection highlights the Scene row");
      model.selectKey("parts:base");tree=make();root=ui.submit(tree,frame);
      check(selected(root,"parts:base"),"tree-only group selection is retained");
      scene.select("project:cover");tree=make();root=ui.submit(tree,frame);
      check(selected(root,"project:cover"),"viewport selection replaces a selected group");
      check(!scene.document.isDirty,"selection and expansion never dirty the document");
    } catch(error:Dynamic){ui.dispose();scene.dispose();fonts.dispose();throw error;}
    ui.dispose();scene.dispose();fonts.dispose();
    Sys.println('Scene tree selection: $assertions assertions passed');
  }
}
