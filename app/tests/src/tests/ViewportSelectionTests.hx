package tests;

import app.EditorScene;
import app.EditorSceneTree;
import app.EditorPerspectiveViewport;
import haxeon.ui.*;
import haxeon.ui.core.*;
import haxeon.ui.host.UiHostContext;
import haxeon.platform.NativeKitEvents;
import haxeon.ui.widgets.collections.TreeView;

@:access(tests.SceneVisibilityTests)
@:access(app.EditorPerspectiveViewport)
class ViewportSelectionTests {
  static var checks=0;
  static function check(ok:Bool,message:String):Void {checks++;if(!ok)throw message;}
  public static function main():Int {run();SceneTreeSelectionTests.run();return SceneVisibilityTests.main();}
  public static function run():Void {
    var data=SceneVisibilityTests.data();
    for(item in data) {item.x=item.id=="project:tool" ? 0 : 10;}
    var definition=SceneVisibilityTests.definition();
    for(occurrence in definition.occurrences)occurrence.includePath="machine/axes";
    var scene=new EditorScene(data),tree=new EditorSceneTree(scene,definition);
    var host=new UiHostContext(null,new NativeKitEvents(),function(){},function(){});
    var style=new LayoutStyle();style.width=LayoutAxis.grow();style.height=LayoutAxis.grow();
    var fonts=FontCollection.create(),font:Null<String> =null;
    for(candidate in ["haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf","../../haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf"])
      if(sys.FileSystem.exists(candidate))font=candidate;
    if(font==null)throw "Missing selection test font";fonts.add(font);
    var viewport=new EditorPerspectiveViewport("selection-test",scene,host,style),ui=new UiContext(null,fonts);
    viewport.selectionTree=tree;
    var frame=new LayoutFrame(800,600);
    try {
      ui.submit(viewport,frame);viewport.camera.frame(0,0,0,4,3,2,800/600);
      var point=viewport.camera.project(0,0,0,800,600);if(point==null)throw "Missing projected test point";
      var x=point.x,y=point.y,path=tree.objectPath("project:tool");
      check(path.length>=6,"fixture has deep authored and moving-body groups");
      check(viewport.pick(x,y)=="project:tool","ray hits the nested part");
      var original=tree.selectionKey();ui.pointerDown(x,y,0);
      check(tree.selectionKey()==original,"press alone does not advance hierarchy");
      ui.pointerCancel(0,x,y);check(tree.selectionKey()==original,"cancelled click does not select");
      var history=scene.document.history.undoCount;
      for(key in path.slice(1)) {
        ui.pointerDown(x,y,0);ui.pointerUp(x,y,0);
        check(tree.selectionKey()==key,'completed click selects hierarchy level $key');
        check(tree.selectedObjects().indexOf("project:tool")>=0,"group highlight includes clicked descendant");
        var view=scene.configureRenderView(new nativekit.scene.SceneView(),viewport.camera.viewProjection(800/600),null,null,-1,null,false);
        check(view.selectionOverrideCount()==tree.selectedObjects().length,"render view highlights every selected group member");
        check(ui.focus.focusedId!=null,"click leaves the viewport focused for Space");
      }
      ui.pointerDown(x,y,0);ui.pointerUp(x,y,0);
      check(tree.selectionKey()=="project:tool","clicking a leaf stays at that leaf");
      check(scene.document.history.undoCount==history,"drilling never adds undo entries");
      ui.key(UiEventKind.KeyDown,UiKey.Space);
      check(!scene.isVisible("project:tool"),"Space hides viewport selection");
      check(tree.selectionKey()=="project:tool","hidden part stays selected in tree");
      check(scene.document.history.undoCount==history+1,"Space creates exactly one visibility edit");
      ui.key(UiEventKind.KeyRepeat,UiKey.Space);ui.key(UiEventKind.KeyUp,UiKey.Space);
      check(!scene.isVisible("project:tool") && scene.document.history.undoCount==history+1,"holding Space does not flicker visibility");
      ui.key(UiEventKind.KeyDown,UiKey.Space);ui.key(UiEventKind.KeyUp,UiKey.Space);
      check(scene.isVisible("project:tool"),"Space shows hidden selection again");
      scene.document.undo();check(!scene.isVisible("project:tool"),"visibility change supports undo");scene.document.undo();
      tree.canChangeVisibility=()->false;ui.key(UiEventKind.KeyDown,UiKey.Space);ui.key(UiEventKind.KeyUp,UiKey.Space);
      check(scene.isVisible("project:tool"),"document edit guard blocks Space");tree.canChangeVisibility=()->true;
      ui.key(UiEventKind.KeyDown,UiKey.Space,UiModifier.Shift);ui.key(UiEventKind.KeyUp,UiKey.Space,UiModifier.Shift);
      check(scene.isVisible("project:tool"),"modified Space does not toggle visibility");
      // A new camera position, or another tree selection, starts from the outer group.
      viewport.camera.orbit(1,0);point=viewport.camera.project(0,0,0,800,600);if(point==null)throw "Missing reprojection";x=point.x;y=point.y;
      ui.pointerDown(x,y,0);ui.pointerUp(x,y,0);check(tree.selectionKey()==path[1],"camera movement resets drilling");
      ui.pointerDown(2,2,0);ui.pointerUp(2,2,0);
      check(tree.selectionKey()=="scene" && tree.selectedObjects().length==0,"empty-space click clears a tree-only group selection and highlight");
      ui.pointerDown(x,y,0);ui.pointerUp(x,y,0);
      check(tree.selectionKey()==path[1],"click after empty space restarts at outer group");
      ui.key(UiEventKind.KeyDown,UiKey.Space);ui.key(UiEventKind.KeyUp,UiKey.Space);
      check(!scene.isVisible("project:tool") && !scene.isVisible("project:base"),"group Space hides the whole subtree");
      check(tree.selectionKey()==path[1],"hidden group remains selected");
      scene.document.undo();tree.selectKey("project:cover");
      ui.pointerDown(x,y,0);ui.pointerUp(x,y,0);check(tree.selectionKey()==path[1],"external selection resets drilling");
      tree.selectKey("project:cover");
      ui.pointerDown(x,y,0);ui.pointerMove(x+20,y+10);ui.pointerUp(x+20,y+10,0);
      check(tree.selectionKey()=="project:tool","drag selects the physical part without drilling");
      check(viewport.navigationPointer==null,"drag releases pointer capture");
      // Revealing all ancestors also works for the groups introduced by viewport drilling.
      var treeUi=new UiContext(null,fonts),treeStyle=new LayoutStyle();treeStyle.width=LayoutAxis.fixed(300);treeStyle.height=LayoutAxis.fixed(160);
      var widget=new TreeView("selection-reveal",tree,treeStyle,tree.scrollController,160,tree.selectionKey(),["scene"]);
      widget.onStateReady=tree.synchronizeSelection;
      treeUi.submit(widget,new LayoutFrame(300,160));
      for(key in path.slice(0,path.length-1))widget.setExpanded(key,false);
      tree.selectKey("project:cover");tree.selectKey("project:tool");treeUi.submit(widget,new LayoutFrame(300,160));
      for(key in path.slice(0,path.length-1))check(widget.isExpanded(key),'tree reveals ancestor $key');
      check(widget.selectedKey=="project:tool","deep picked row is selected");
      check(tree.scrollController.offsetY>0,"deep row scrolls into view");
      treeUi.dispose();
      viewport.simulationActive=true;
      ui.key(UiEventKind.KeyDown,UiKey.Space);ui.key(UiEventKind.KeyUp,UiKey.Space);
      check(scene.isVisible("project:tool"),"simulation mode blocks visibility edits");viewport.simulationActive=false;
      var field=new haxeon.ui.widgets.text.TextField("selection-notes","",_ -> {});
      var fieldNode=ui.submit(field,frame);ui.focusWidget(fieldNode.id);
      ui.key(UiEventKind.KeyDown,UiKey.Space);ui.key(UiEventKind.KeyUp,UiKey.Space);
      check(scene.isVisible("project:tool"),"Space in a text field never changes scene visibility");
    } catch(error:Dynamic){ui.dispose();viewport.dispose();scene.dispose();fonts.dispose();throw error;}
    ui.dispose();viewport.dispose();scene.dispose();fonts.dispose();
    Sys.println('Viewport selection: $checks checks passed');
  }
}
