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

@:access(haxeon.ui.core.UiContext)
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
    var result=model.definition("visibility-test");
    for(occurrence in result.occurrences)occurrence.includePath=switch occurrence.id {
      case "base","cover","bracket": "machine";
      case "slide": "machine/slide";
      case "tool": "machine/slide/tooling";
      default: null;
    };
    return result;
  }
  public static function main():Int {
    run();
    assemblyGrouping();
    ownershipSemantics();
    legacyVisibility();
    if(Sys.args().indexOf("--regressions")>=0) {
      if(SceneAtomicityTests.main()!=0)return 1;
      SceneDocumentTests.run();
      if(ProjectDocumentTests.main()!=0)return 1;
      Sys.println("Visibility document and atomicity regressions passed");
    }
    return 0;
  }
  static function assemblyGrouping():Void {
    var scene=new EditorScene(data()), authored=definition();
    for (occurrence in authored.occurrences) occurrence.includePath=switch occurrence.id {
      case "base", "other": "tables/infeed/upper-frame";
      case "cover", "bracket": "tables/infeed/worktop";
      default: null;
    };
    var tree=new EditorSceneTree(scene,authored);
    var tables=tree.assemblyKey("tables"), infeed=tree.assemblyKey("tables/infeed");
    check(tree.childCount(tables)==1 && tree.childKeyAt(tables,0)==infeed,"authored assembly ownership nests root groups");
    check(tree.visibilityMembers(tables).length==4,"assembly visibility follows ownership independent of joints");
    scene.select("project:bracket");
    check(tree.selectionPath().indexOf(tables)>=0 && tree.selectionPath().indexOf(tree.assemblyKey("tables/infeed/worktop"))>=0,
      "selection reveals assembly and connection ancestors");
    tree.toggleVisibility(tables);
    check(scene.isVisible("project:tool") && !scene.isVisible("project:other"),"assembly hide covers every descendant");
    tree.toggleVisibility(tables);
    check(scene.isVisible("project:tool"),"assembly visibility restores descendants");
    tree.setFilter("worktop");
    check(tree.childCount(infeed)==1,"assembly labels participate in filtering");
    scene.dispose();
  }
  static function ownershipSemantics():Void {
    var authored=definition(),scene=new EditorScene(data()),tree=new EditorSceneTree(scene,authored);
    try {
      var machine=tree.assemblyKey("machine"),path=tree.objectPath("project:tool");
      tree.toggleVisibility("project:base");
      check(!scene.isVisible("project:base") && scene.isVisible("project:cover") && scene.isVisible("project:tool"),
        "physical joint root remains a single part for visibility");
      scene.document.undo();
      tree.selectKey("project:base");
      check(tree.selectedObjects().length==1,"physical joint root selects only its occurrence");
      tree.selectKey(machine);
      check(tree.selectedObjects().length==5 && scene.selectedId=="scene","assembly selection has members without a destructive part target");
      var info=tree.groupInfo(machine);
      check(info!=null && info.count==5 && info.shown==5,"group inspector reports authored membership");
      authored.joints.resize(0);
      var reparented=new EditorSceneTree(scene,authored);
      check(reparented.objectPath("project:tool").join("|")==path.join("|"),"changing motion topology does not reparent authored occurrences");
      var exported=definition();
      for(occurrence in exported.occurrences)occurrence.includePath=null;
      machinekit.assembly.AssemblyPreview.preserveOwnership(exported,authored);
      var restored=new EditorSceneTree(scene,exported);
      check(restored.objectPath("project:tool").join("|")==path.join("|"),"custom scene exports retain authored ownership");
      for(occurrence in exported.occurrences)occurrence.includePath=null;
      var flat=new EditorSceneTree(scene,exported);
      check(flat.objectPath("project:tool").length==3,"older flat exports get a root assembly without guessed motion groups");
    } catch(error:Dynamic) {scene.dispose();throw error;}
    scene.dispose();
  }
  static function legacyVisibility():Void {
    for (saved in [
      {key:"project:base",hidden:["base","cover","bracket","slide","tool"]},
      {key:"parts:base",hidden:["cover","bracket"]},
      {key:"parts:scene:machine",hidden:["base","cover","bracket","slide","tool"]},
      {key:"parts:parts:base:machine",hidden:["cover","bracket"]}
    ]) {
      var records=data();records[0].hiddenBy=[saved.key];
      var scene=new EditorScene(records);
      try {
        var physics=scene.environmentRevision,tree=new EditorSceneTree(scene,definition());
        for (id in ["base","cover","bracket","slide","tool","other"])
          check(scene.isVisible("project:"+id)==(saved.hidden.indexOf(id)<0),"legacy mask resolves old membership: "+saved.key+" / "+id);
        check(!scene.document.isDirty && scene.environmentRevision==physics,"migration leaves history and physics unchanged");
        tree.showAll();
        for (record in scene.items()) check(scene.isVisible(record.id),"show all recovers migrated visibility");
      } catch(error:Dynamic) {scene.dispose();throw error;}
      scene.dispose();
    }
    var records=data();records[0].hiddenBy=["assembly:visibility-test:machine"];
    var scene=new EditorScene(records);
    try {
      var tree=new EditorSceneTree(scene,definition());
      check(!scene.isVisible("project:tool") && scene.isVisible("project:other"),"canonical masks rebind to current ownership");
      tree.toggleVisibility(tree.assemblyKey("machine"));
      check(scene.isVisible("project:tool"),"canonical group remains independently showable after rebind");
    } catch(error:Dynamic) {scene.dispose();throw error;}
    scene.dispose();
  }
  public static function run():Void {
    var scene=new EditorScene(data()), tree=new EditorSceneTree(scene,definition());
    try {
      var physics=scene.environmentRevision;
      check(tree.toggleVisibility("project:cover"),"hide a leaf");
      check(!item(scene,"project:cover").visible && !scene.info("project:cover").visible(),"own flag and renderer hide leaf");
      var history=scene.document.history.undoCount;
      tree.toggleVisibility(tree.assemblyKey("machine"));
      check(scene.document.history.undoCount==history+1,"whole group is one undo operation");
      for(id in ["base","cover","bracket","slide","tool"])check(!scene.isVisible("project:"+id),"body hides its full subtree");
      check(item(scene,"project:bracket").visible,"parent does not overwrite child flag");
      check(scene.isVisible("project:other"),"unrelated object remains visible");
      tree.toggleVisibility(tree.assemblyKey("machine"));
      check(scene.isVisible("project:bracket") && !scene.isVisible("project:cover"),"showing parent preserves child hide");
      tree.toggleVisibility(tree.assemblyKey("machine/slide/tooling"));
      tree.toggleVisibility(tree.assemblyKey("machine"));tree.toggleVisibility(tree.assemblyKey("machine"));
      check(!scene.isVisible("project:tool") && scene.isVisible("project:slide"),"nested group mask survives parent toggles");
      scene.select("project:other");tree.setFilter("cover");tree.toggleVisibility(tree.assemblyKey("machine"));
      check(!scene.isVisible("project:bracket"),"filter does not limit group visibility operation");
      check(tree.childCount(tree.assemblyKey("machine"))==1,"hidden matching row stays in filtered tree");
      scene.document.undo();
      check(scene.isVisible("project:bracket"),"undo filtered group hide");
      tree.selectKey(tree.assemblyKey("machine"));tree.isolateSelected();
      check(!scene.isVisible("project:other") && scene.isVisible("project:base"),"isolate subtree");
      check(!scene.isVisible("project:cover") && !scene.isVisible("project:tool"),"isolate preserves hidden descendants");
      scene.document.undo();
      check(scene.isVisible("project:other"),"isolate undo restores unrelated objects");
      tree.selectKey(tree.assemblyKey("machine/slide/tooling"));
      check(tree.selectionKey()==tree.assemblyKey("machine/slide/tooling") && scene.selectedId=="scene","group selection clears destructive object target");
      scene.select("project:other");
      check(tree.selectionKey()=="project:other","viewport selection replaces group selection");
      tree.toggleVisibility(tree.assemblyKey("machine"));
      var encoded=SceneCodec.encode(scene), restored=new EditorScene(SceneCodec.decode(encoded));
      try {
        var restoredTree=new EditorSceneTree(restored,definition());restoredTree.toggleVisibility(restoredTree.assemblyKey("machine"));
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
    popupControls();
    scrollbarControls();
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
  static function scrollbarControls():Void {
    var fonts=haxeon.ui.FontCollection.create(),font="haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf";
    if(!FileSystem.exists(font))font="../../"+font;fonts.add(font);
    var records:Array<SceneObjectData> = [for(i in 0...40)
      {id:"row-"+i,label:"Object "+i+" with a long scene label",type:"rectangle",x:0.0,y:0.0,z:0.0,
        width:1.0,height:1.0,depth:1.0,collisionEnabled:false,dynamicBody:false,mass:1.0,
        red:0.5,green:0.5,blue:0.5,visible:true}];
    var scene=new EditorScene(records),model=new EditorSceneTree(scene),ui=new haxeon.ui.core.UiContext(null,fonts);
    ui.buildContext.environment.scrollbarVisibility=haxeon.ui.widgets.scroll.ScrollbarVisibility.Always;
    var controller=model.scrollController;
    var style=new haxeon.ui.LayoutStyle();style.width=haxeon.ui.LayoutAxis.grow();style.height=haxeon.ui.LayoutAxis.grow();
    var tree=new haxeon.ui.widgets.collections.TreeView("scrollbar-eyes",model,style,controller,180,null,["scene"],key->model.selectKey(key));
    try {
      for(width in [180,260,400]) {
        var frame=new haxeon.ui.LayoutFrame(width,180);frame.deltaSeconds=0.25;
        var root=ui.submit(tree,frame);
        ui.scroll(width/2,80,0,28);root=ui.submit(tree,frame);
        var thumb=findNode(root,node->node.semantics!=null && node.semantics.label=="Vertical scroll position");
        if(thumb==null || thumb.parent==null)throw "Missing hierarchy scrollbar";
        var track=thumb.parent,trackBounds=track.globalBounds();
        var eye=findNode(root,node->{
          var semantics=node.semantics;if(semantics==null || semantics.label==null || !StringTools.startsWith(semantics.label,"Hide Object "))return false;
          var bounds=node.globalBounds();return bounds.y>5 && bounds.y+bounds.height<175;
        });
        if(eye==null)throw "Missing visible hierarchy eye";
        var bounds=eye.globalBounds();
        check(track.layout.style.zIndex>0,"scrollbar overlay paints above ordinary tree rows");
        var scrollbarPath=haxeon.ui.core.HitTest.path(root,trackBounds.x+trackBounds.width/2,bounds.y+bounds.height/2);
        check(scrollbarPath.indexOf(track)>=0,"scrollbar owns pointer input in its overlay strip");
        check(bounds.x+bounds.width<=trackBounds.x,"visibility eye does not extend beneath the scrollbar");
        var eyePath=haxeon.ui.core.HitTest.path(root,bounds.x+bounds.width/2,bounds.y+bounds.height/2);
        check(eyePath[eyePath.length-1]==eye,"eye centre remains independently clickable beside scrollbar");
        var history=scene.document.history.undoCount;
        ui.pointerDown(bounds.x+bounds.width/2,bounds.y+bounds.height/2,0);
        ui.pointerUp(bounds.x+bounds.width/2,bounds.y+bounds.height/2,0);
        check(scene.document.history.undoCount==history+1,"eye click toggles visibility rather than scrolling");
        scene.document.undo();
        // Dismiss the eye's tooltip before starting a separate scrollbar gesture.
        ui.pointerMove(-20,-20);root=ui.submit(tree,frame);
        thumb=findNode(root,node->node.semantics!=null && node.semantics.label=="Vertical scroll position");
        if(thumb==null)throw "Missing rebuilt hierarchy scrollbar";
        var before=controller.offsetY,thumbBounds=thumb.globalBounds();
        var x=thumbBounds.x+thumbBounds.width/2,y=thumbBounds.y+thumbBounds.height/2;
        history=scene.document.history.undoCount;
        ui.pointerMove(x,y);root=ui.submit(tree,frame);
        var hitPath=haxeon.ui.core.HitTest.path(root,x,y);
        check(hitPath[hitPath.length-1].id.equals(thumb.id),"scrollbar thumb receives its own drag press");
        ui.pointerDown(x,y,0);ui.pointerMove(x,y+25);ui.pointerUp(x,y+25,0);
        check(controller.offsetY>before,"scrollbar drag moves the hierarchy");
        check(scene.document.history.undoCount==history,"scrollbar drag does not toggle visibility");
      }
      ui.dispose();scene.dispose();fonts.dispose();
    }catch(error:Dynamic){ui.dispose();scene.dispose();fonts.dispose();throw error;}
  }
  static function popupControls():Void {
    var fonts=haxeon.ui.FontCollection.create();
    fonts.add("haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf");
    var ui=new haxeon.ui.core.UiContext(null,fonts),scene=new EditorScene(data()),tree=new EditorSceneTree(scene,definition());
    var dismissed=false,physics=scene.environmentRevision;
    scene.select("project:other");tree.setFilter("other");
    var frame=new haxeon.ui.LayoutFrame(480,480);
    function view():haxeon.ui.core.View return app.editor.VisibilityPanel.popup(tree,450,450,480,480,true,()->{dismissed=true;});
    try {
      var root=ui.submit(view(),frame);
      var cover=findNode(root,node->node.semantics!=null&&node.semantics.label=="cover");
      check(cover!=null,"popup has unfiltered hierarchy despite scene tree search");
      var b=cover.globalBounds();
      check(b.x>=0&&b.x+b.width<=480&&b.y>=0&&b.y+b.height<=480,"popup clamps to the window");
      ui.pointerDown(b.x+25,b.y+b.height/2,0);ui.pointerUp(b.x+25,b.y+b.height/2,0);
      check(!scene.isVisible("project:cover")&&!dismissed,"checkbox hides immediately without dismissing popup");
      check(scene.selectedId=="project:other","checkbox does not change viewport selection");
      root=ui.submit(view(),frame);
      var mixed=findNode(root,node->node.semantics!=null&&node.semantics.value=="mixed");
      check(mixed!=null,"partially hidden parent has a mixed checkbox");
      tree.toggleVisibility(tree.assemblyKey("machine"));tree.toggleVisibility(tree.assemblyKey("machine"));
      check(!scene.isVisible("project:cover")&&scene.isVisible("project:bracket"),"tree and popup share inherited child visibility");
      scene.document.undo();scene.document.undo();scene.document.undo();
      check(scene.isVisible("project:cover"),"popup visibility edits undo through the shared document");
      check(scene.environmentRevision==physics,"popup leaves physics unchanged");
      root=ui.submit(view(),frame);
      ui.pointerDown(2,2,0);ui.pointerUp(2,2,0);check(dismissed,"outside click dismisses popup");
      ui.dispose();scene.dispose();fonts.dispose();
    }catch(error:Dynamic){ui.dispose();scene.dispose();fonts.dispose();throw error;}
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
      ["scene",model.assemblyKey(),model.assemblyKey("machine"),model.assemblyKey("machine/slide"),model.assemblyKey("machine/slide/tooling")],key->model.selectKey(key));
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
      tree.toggleVisibility("project:cover");tree.toggleVisibility(tree.assemblyKey());
      session.save(file);reopened.open(file);
      check(!reopened.scene.isVisible("project:bracket"),"generated project restores inherited mask");
      var reopenedTree=new EditorSceneTree(reopened.scene,reopened.projectAssemblyDefinition);
      reopenedTree.toggleVisibility(reopenedTree.assemblyKey());
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
