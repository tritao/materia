package tests;

import app.EditorScene;
import app.EditorSceneTree;
import app.SceneCodec;
import app.SceneObjectData;
import app.ProjectDocumentSession;
import app.editor.SceneVisibility;
import app.editor.HierarchyTreeView;
import cadkit.modeling.AssemblyModel;
import materia.assembly.AssemblyFrames;
import haxeon.ui.core.UiEvent;
import haxeon.ui.core.UiEventKind;
import haxeon.ui.core.UiKey;
import haxeon.ui.core.WidgetId;
import sys.io.File;
import sys.FileSystem;

class SceneVisibilityTests {
  static var assertions=0;
  static function check(value:Bool,message:String):Void {assertions++;if(!value)throw message;}
  static function item(scene:EditorScene,id:String):app.EditorScene.EditorSceneObject {
    var result=scene.object(id);if(result==null)throw "Missing visibility test object";return result;
  }
  static function data():Array<SceneObjectData> return [for(id in ["base","cover","bracket","slide","tool","other"])
    {id:"project:"+id,label:id,type:"rectangle",x:0.0,y:0.0,z:0.0,width:1.0,height:1.0,depth:1.0,
      collisionEnabled:false,dynamicBody:false,mass:1.0,red:0.5,green:0.5,blue:0.5,visible:true}];
  static function definition():materia.assembly.AssemblyDefinition {
    var model=new AssemblyModel("m");
    for(id in ["base","cover","bracket","slide","tool","other"]) {
      model.add(id);model.connector(id,"mount",AssemblyFrames.identity());
    }
    model.mate("cover-fixed","fixed","base","mount","cover","mount");
    model.mate("bracket-fixed","fixed","base","mount","bracket","mount");
    model.mate("slide-moving","prismatic","base","mount","slide","mount");
    model.mate("tool-fixed","fixed","slide","mount","tool","mount");
    return model.definition("visibility-test");
  }
  public static function main():Int {
    run();
    if(Sys.args().indexOf("--regressions")>=0) {
      if(SceneAtomicityTests.main()!=0)return 1;
      SceneDocumentTests.run();
      if(ProjectDocumentTests.main()!=0)return 1;
      Sys.println("Visibility document and atomicity regressions passed");
    }
    return 0;
  }
  public static function run():Void {
    var scene=new EditorScene(data()), tree=new EditorSceneTree(scene,definition());
    try {
      var physics=scene.environmentRevision;
      check(tree.toggleVisibility("project:cover"),"hide a leaf");
      check(!item(scene,"project:cover").visible && !scene.info("project:cover").visible(),"own flag and renderer hide leaf");
      var history=scene.document.history.undoCount;
      tree.toggleVisibility("project:base");
      check(scene.document.history.undoCount==history+1,"whole group is one undo operation");
      for(id in ["base","cover","bracket","slide","tool"])check(!scene.isVisible("project:"+id),"body hides its full subtree");
      check(item(scene,"project:bracket").visible,"parent does not overwrite child flag");
      check(scene.isVisible("project:other"),"unrelated object remains visible");
      tree.toggleVisibility("project:base");
      check(scene.isVisible("project:bracket") && !scene.isVisible("project:cover"),"showing parent preserves child hide");
      tree.toggleVisibility("parts:slide");
      tree.toggleVisibility("project:base");tree.toggleVisibility("project:base");
      check(!scene.isVisible("project:tool") && scene.isVisible("project:slide"),"nested group mask survives parent toggles");
      tree.setFilter("cover");tree.toggleVisibility("parts:base");
      check(!scene.isVisible("project:bracket"),"filter does not limit group visibility operation");
      check(tree.childCount("parts:base")==1,"hidden matching row stays in filtered tree");
      scene.document.undo();
      check(scene.isVisible("project:bracket"),"undo filtered group hide");
      tree.selectKey("project:base");tree.isolateSelected();
      check(!scene.isVisible("project:other") && scene.isVisible("project:base"),"isolate subtree");
      check(!scene.isVisible("project:cover") && !scene.isVisible("project:tool"),"isolate preserves hidden descendants");
      scene.document.undo();
      check(scene.isVisible("project:other"),"isolate undo restores unrelated objects");
      tree.selectKey("parts:slide");
      check(tree.selectionKey()=="parts:slide" && scene.selectedId=="scene","group selection clears destructive object target");
      scene.select("project:other");
      check(tree.selectionKey()=="project:other","viewport selection replaces group selection");
      tree.toggleVisibility("project:base");
      var encoded=SceneCodec.encode(scene), restored=new EditorScene(SceneCodec.decode(encoded));
      try {
        var restoredTree=new EditorSceneTree(restored,definition());restoredTree.toggleVisibility("project:base");
        check(restored.isVisible("project:bracket") && !restored.isVisible("project:cover") && !restored.isVisible("project:tool"),
          "scene roundtrip retains parent and child visibility separately");
        restoredTree.showAll();
        for(item in restored.items())check(restored.isVisible(item.id),"show all clears own and inherited hiding");
        restored.document.undo();check(!restored.isVisible("project:tool"),"show all is undoable");
      } catch(error:Dynamic) {restored.dispose();throw error;}
      restored.dispose();
      tree.canChangeVisibility=()->false;
      check(!tree.toggleVisibility("project:other") && !tree.showAll() && !tree.isolateSelected(),"blocked document cannot change visibility");
      check(scene.environmentRevision==physics,"visibility leaves physics unchanged");
    } catch(error:Dynamic) {scene.dispose();throw error;}
    scene.dispose();
    var ordinary=new EditorScene();
    try {
      var plainTree=new EditorSceneTree(ordinary);
      plainTree.toggleVisibility("box");
      check(ordinary.pick(-1.5,0)!="box","hidden object is not picked");
      ordinary.document.undo();check(ordinary.pick(-1.5,0)=="box","undo restores picking");
      plainTree.toggleVisibility("scene");
      check(item(ordinary,"box").visible && !ordinary.isVisible("box"),"scene mask leaves own flags intact");
      ordinary.setVisible("box",false);ordinary.setVisible("box",true);
      check(!ordinary.isVisible("box"),"inspector cannot bypass hidden parent");
      ordinary.createRectangle();
      check(!ordinary.isVisible(ordinary.selectedId),"new roots inherit the hidden Scene row");
      ordinary.document.undo();ordinary.document.redo();
      check(!ordinary.isVisible(ordinary.selectedId),"created object retains inherited hiding after redo");
    } catch(error:Dynamic) {ordinary.dispose();throw error;}
    ordinary.dispose();
    var regenerated=data();regenerated[0].hiddenBy=["project:base"];
    var generatedScene=new EditorScene(regenerated);
    var regeneratedTree=new EditorSceneTree(generatedScene,definition());
    check(!generatedScene.isVisible("project:bracket"),"regeneration rebinds hidden parent to newly introduced descendants");
    generatedScene.dispose();
    generatedRoundtrip();
    uiControls();
    var count=0;
    var key=new UiEvent(UiEventKind.KeyDown,new WidgetId(1),0,0,0,0,0,UiKey.Space);
    HierarchyTreeView.handleKey(key,()->count++);
    check(count==1 && key.defaultPrevented && key.propagationStopped,"tree Space toggles and consumes event");
    var modified=new UiEvent(UiEventKind.KeyDown,new WidgetId(1),0,0,0,0,0,UiKey.Space,1);
    HierarchyTreeView.handleKey(modified,()->count++);
    check(count==1 && !modified.defaultPrevented,"modified Space is left alone");
    var repeat=new UiEvent(UiEventKind.KeyRepeat,new WidgetId(1),0,0,0,0,0,UiKey.Space);
    HierarchyTreeView.handleKey(repeat,()->count++);check(count==1,"key repeat cannot repeatedly toggle");
    Sys.println('Scene visibility: $assertions assertions passed');
  }
  static function findNode(root:haxeon.ui.core.RenderNode, predicate:haxeon.ui.core.RenderNode->Bool):Null<haxeon.ui.core.RenderNode> {
    if(predicate(root))return root;
    for(child in root.children){var found=findNode(child,predicate);if(found!=null)return found;}
    return null;
  }
  static function uiControls():Void {
    var fonts=haxeon.ui.FontCollection.create(), font:Null<String> = null;
    for(path in ["haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf",
      "../../haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf"])
      if(FileSystem.exists(path))font=path;
    if(font==null)throw "Missing visibility test font";
    fonts.add(font);
    var ui=new haxeon.ui.core.UiContext(null,fonts), scene=new EditorScene(data());
    var model=new EditorSceneTree(scene,definition());scene.select("project:other");
    var style=new haxeon.ui.LayoutStyle();style.width=haxeon.ui.LayoutAxis.fixed(400);style.height=haxeon.ui.LayoutAxis.fixed(400);
    var treeStyle=new haxeon.ui.LayoutStyle();treeStyle.width=haxeon.ui.LayoutAxis.grow();treeStyle.height=haxeon.ui.LayoutAxis.grow();
    var tree=new haxeon.ui.widgets.collections.TreeView("test-tree",model,treeStyle,null,350,"project:other",
      ["scene","project:base","parts:base","project:slide","parts:slide"],key->model.selectKey(key));
    var view=new haxeon.ui.widgets.layout.Column("visibility-panel",[
      new haxeon.ui.widgets.KeyedView("search",new haxeon.ui.widgets.controls.SearchField("search","",_ -> {})),
      new haxeon.ui.widgets.KeyedView("tree",new HierarchyTreeView(tree,()->{if(tree.selectedKey!=null)model.toggleVisibility(tree.selectedKey);}))],style);
    var frame=new haxeon.ui.LayoutFrame(400,400);
    try {
      var root=ui.submit(view,frame);
      var eye=findNode(root,node->node.semantics!=null && node.semantics.label=="Hide cover (Space)");
      if(eye==null)throw "Missing visibility eye";
      var bounds=eye.globalBounds();
      check(bounds.width>=20 && bounds.height>=20,"eye has a usable independent hit target");
      ui.pointerDown(bounds.x+bounds.width/2,bounds.y+bounds.height/2,0);
      ui.pointerUp(bounds.x+bounds.width/2,bounds.y+bounds.height/2,0);
      check(!scene.isVisible("project:cover") && scene.selectedId=="project:other","eye click toggles without selecting or expanding row");
      root=ui.submit(view,frame);
      ui.key(UiEventKind.KeyDown,UiKey.Space);ui.key(UiEventKind.KeyUp,UiKey.Space);
      check(scene.isVisible("project:cover"),"focused eye Space toggles exactly once");
      ui.key(UiEventKind.KeyDown,UiKey.Space,1);ui.key(UiEventKind.KeyUp,UiKey.Space,1);
      check(scene.isVisible("project:cover") && scene.selectedId=="project:other","modified Space on an eye does not activate it or its row");
      root=ui.submit(view,frame);
      var row=findNode(root,node->node.semantics!=null && node.semantics.role==haxeon.ui.semantics.AccessibilityRole.TreeItem && node.semantics.label=="project:cover");
      if(row==null)throw "Missing cover row";
      bounds=row.globalBounds();
      ui.pointerDown(bounds.x+80,bounds.y+bounds.height/2,0);ui.pointerUp(bounds.x+80,bounds.y+bounds.height/2,0);
      check(scene.selectedId=="project:cover","row click selects the object");
      ui.key(UiEventKind.KeyDown,UiKey.Space);ui.key(UiEventKind.KeyUp,UiKey.Space);
      check(!scene.isVisible("project:cover"),"Space on a tree row changes visibility");
      root=ui.submit(view,frame);
      var field=findNode(root,node->node.semantics!=null && node.semantics.role==haxeon.ui.semantics.AccessibilityRole.TextField);
      if(field==null)throw "Missing hierarchy search field";
      ui.focusWidget(field.id);
      var undoCount=scene.document.history.undoCount;
      ui.key(UiEventKind.KeyDown,UiKey.Space);ui.key(UiEventKind.KeyUp,UiKey.Space);
      check(scene.document.history.undoCount==undoCount,"Space in search never toggles tree visibility");
    } catch(error:Dynamic){ui.dispose();scene.dispose();fonts.dispose();throw error;}
    ui.dispose();scene.dispose();fonts.dispose();
  }
  static function generatedRoundtrip():Void {
    var folder="build/visibility-tests-"+Std.random(1000000);
    FileSystem.createDirectory(folder);
    var artifact=folder+"/source.mtrg", file=folder+"/saved.materia.json";
    var assembly=new machinekit.assembly.MachineAssembly();
    for(id in ["base","cover","bracket","slide","tool","other"])
      assembly.addComponent(id,new machinekit.milling.MillPanel(100,100,100));
    assembly.addMate("cover-fixed","fixed","base","mount","cover","mount");
    assembly.addMate("bracket-fixed","fixed","base","mount","bracket","mount");
    assembly.addMate("slide-moving","prismatic","base","mount","slide","mount");
    assembly.addMate("tool-fixed","fixed","slide","mount","tool","mount");
    File.saveBytes(artifact,materia.project.SceneArtifact.encode(machinekit.assembly.AssemblyPreview.scene(assembly,"visibility-test")));
    var session=new ProjectDocumentSession(), reopened=new ProjectDocumentSession();
    function cleanup():Void {
      session.dispose();reopened.dispose();
      if(FileSystem.exists(file))FileSystem.deleteFile(file);
      if(FileSystem.exists(artifact))FileSystem.deleteFile(artifact);
      FileSystem.deleteDirectory(folder);
    }
    try {
      session.openGeneratedProject(app.ProjectSourceLoader.load(artifact),artifact);
      var tree=new EditorSceneTree(session.scene,session.projectAssemblyDefinition);
      tree.toggleVisibility("project:cover");tree.toggleVisibility("project:base");
      session.save(file);reopened.open(file);
      check(!reopened.scene.isVisible("project:bracket"),"generated project restores inherited mask");
      var reopenedTree=new EditorSceneTree(reopened.scene,reopened.projectAssemblyDefinition);
      reopenedTree.toggleVisibility("project:base");
      check(reopened.scene.isVisible("project:bracket") && !reopened.scene.isVisible("project:cover"),
        "generated save/open preserves independently hidden children");
      var root:Dynamic=haxe.Json.parse(File.getContent(file));
      var project:Dynamic=Reflect.field(root,"project");
      check(project!=null,"generated project stays source owned");
      var found=false, overrides:Array<Dynamic> = cast Reflect.field(project,"overrides");
      for(edit in overrides)if(Reflect.field(edit,"property")=="hiddenBy")found=true;
      check(found,"generated project persists inherited visibility overrides");
      // The regular scene codec also validates every generated mask's type and duplicate ids.
      var malformed:Dynamic=haxe.Json.parse(SceneCodec.encode(session.scene));
      var objects:Array<Dynamic> = cast Reflect.field(malformed,"objects");
      Reflect.setField(objects[0],"hiddenBy",["same","same"]);
      var rejected=false;try SceneCodec.decode(haxe.Json.stringify(malformed)) catch(_:Dynamic)rejected=true;
      check(rejected,"duplicate persisted visibility masks rejected");
    } catch(error:Dynamic) {cleanup();throw error;}
    cleanup();
  }
}
