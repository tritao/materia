package tests;

import app.ProjectVariants;
import app.ProjectSourceLoader;
import app.Main.ReferenceEditorApp;
import app.SceneCodec;
import app.MateriaProjectRunner;
import app.editor.MissionPanel;
import haxe.Json;
import sys.io.File;
import sys.FileSystem;
import haxeon.ui.LayoutFrame;
import haxeon.ui.core.RenderNode;
import haxeon.ui.widgets.text.Text;
import materia.project.SceneArtifact;

@:access(app.Main.ReferenceEditorApp)
class ProjectVariantsTests {
 static function check(ok:Bool,label:String):Void {if(!ok)throw label;}
 static function rejects(action:Void->Void,label:String):Void {
  var rejected=false;try action()catch(_:Dynamic)rejected=true;check(rejected,label);
 }
 static function find(node:RenderNode,label:String):Bool {
  if(node.semantics!=null&&node.semantics.label==label)return true;
  for(child in node.children)if(find(child,label))return true;return false;
 }
 public static function main():Int {
  try {run();Sys.println("Project variants: schema, configuration UI, switching, persistence, artifact roundtrip and legacy migration passed");return 0;}
  catch(error:Dynamic){Sys.println("Project variants failed: "+error);return 1;}
 }
 static function run():Void {
  var mainFile=FileSystem.fullPath(app.editor.WorkerAssetPath.resolve("app/src/Main.hx"));
  Sys.setCwd(haxe.io.Path.directory(haxe.io.Path.directory(haxe.io.Path.directory(mainFile))));
  var source=app.editor.WorkerAssetPath.resolve("app/tests/fixtures/project-variants/materia.project.json");
  var root:Dynamic=Json.parse(File.getContent(source));
  check(ProjectVariants.decode(root).length==4,"variants decode with legacy scene compatibility");
  check(ProjectVariants.decode(root).filter(v->v.id=="demo")[0].kind=="configuration","legacy demo scenes migrate to configurations");
  check(ProjectVariants.entrypoint(root,null)=="small","default variant resolves");
  check(ProjectSourceLoader.executionRequirement(source,"large").entrypoint=="large","loader resolves variant independently of missions");
  rejects(()->ProjectVariants.selected(root,"unknown"),"unknown variant rejected");
  var bad:Dynamic=Json.parse(Json.stringify(root));var emptyJobs:Array<Dynamic> = [];bad.jobs=emptyJobs;
  rejects(()->ProjectVariants.decode(bad),"ambiguous old/new schemas rejected");
  bad=Json.parse(Json.stringify(root));var invalidKind:Array<Dynamic> = bad.variants;invalidKind[0].kind="mission";
  rejects(()->ProjectVariants.decode(bad),"runtime mission is not a scene variant kind");
  bad=Json.parse(Json.stringify(root));var duplicateVariants:Array<Dynamic> = bad.variants;duplicateVariants.push(duplicateVariants[0]);
  rejects(()->ProjectVariants.decode(bad),"duplicate variants rejected");
  bad=Json.parse(Json.stringify(root));bad.defaultVariant="missing";
  rejects(()->ProjectVariants.decode(bad),"missing default rejected");
  bad=Json.parse(Json.stringify(root));var invalidEntry:Array<Dynamic> = bad.variants;invalidEntry[0].entrypoint="missing";
  rejects(()->ProjectVariants.decode(bad),"missing entrypoint rejected");
  check(ProjectVariants.entrypoint({defaultEntrypoint:"old"},null)=="old","projects without variants remain valid");
  var mill:Dynamic=Json.parse(File.getContent(app.editor.WorkerAssetPath.resolve("machinekit/examples/bench-mill/materia.project.json")));
  check(ProjectVariants.decode(mill).filter(v->v.kind=="configuration").length==2,"mill exposes only Small and Compact VMC configurations");
  check(ProjectVariants.decode(mill).filter(v->v.kind=="demo").length==0,"mill has no demo scene category");
  check(ProjectVariants.decode(mill).filter(v->v.id=="enclosed").length==0,"enclosed bench mill is outside the Small/VMC chooser");
  var bench:Dynamic=Json.parse(File.getContent(app.editor.WorkerAssetPath.resolve("machinekit/examples/bench-mill/materia.bench.project.json")));
  var benchVariants=ProjectVariants.decode(bench);
  check(benchVariants.length==2&&benchVariants[0].id=="bench"&&benchVariants[1].id=="enclosed","bench project owns open and enclosed machines");
  check(ProjectVariants.entrypoint(bench,null)=="bench","bench project defaults to the bench mill");
  rejects(()->ProjectVariants.selected(bench,"small"),"bench project cannot select the Small/VMC design");
  check(Reflect.field(mill,"jobs")==null,"mill no longer describes configurations as jobs");

  var fonts=haxeon.ui.FontCollection.create();fonts.add(app.editor.WorkerAssetPath.resolve("haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf"));
  var dir=app.editor.WorkerAssetPath.resolve("build/project-variants-"+Std.random(100000000));FileSystem.createDirectory(dir);
  var editor=new ReferenceEditorApp(fonts,dir+"/workspace.json",null,null,null,null,null,null,true);
  var small=ProjectSourceLoader.load(source,null,null,"small");
  check(small.projectVariant=="small"&&small.projectJob==null&&small.missions==null,"configuration load invents neither jobs nor missions");
  editor.session.openGeneratedProject(small,source);editor.documentChanged();
  var original=editor.session.document,smallGeometry=editor.scene.records()[0].width;
  var frame=new LayoutFrame(800,600);
  var ui=editor.ui.submit(editor.viewportWithControls(new Text("Scene"),800),frame);
  check(find(ui,"Configuration")&&find(ui,"Study")&&!find(ui,"Demo"),"project selector separates configurations and studies without a demo chooser");
  check(!find(ui,"Mission job"),"configuration UI has no mission-job selector");
  var mission=editor.ui.submit(new MissionPanel().build(editor.missionController,editor.appearance.theme.tokens),frame);
  check(!find(mission,"Configuration"),"mission panel does not manage project configurations");
  check(editor.missionController.snapshot().mission==null,"plain machine does not have a synthetic mission");
  editor.projectVariants.select("large");
  check(editor.projectVariants.isLoading()&&editor.session.document==original&&editor.session.projectVariant=="small","preparation preserves installed scene and selection");
  var pending=editor.projectVariants.snapshot();
  check(find(editor.submit(frame),"Opening large"),"variant loading is visible in shared editor chrome without its chooser");
  check(!pending.switchAllowed&&StringTools.startsWith(pending.status,"Opening large · "),"pending selection reports target and locks chooser");
  var loadingUi=editor.ui.submit(editor.viewportWithControls(new Text("Scene"),800),frame);
  check(find(loadingUi,"Loading project selection")&&find(loadingUi,"Cancel"),"chooser shows live progress and cancellation while preparing");
  var deadline=Sys.time()+180;
  while(editor.projectVariants.isLoading()&&Sys.time()<deadline){editor.tick();Sys.sleep(0.01);}
  check(!editor.projectVariants.isLoading()&&editor.projectVariants.snapshot().error==null,"variant installs: "+editor.projectVariants.snapshot().error);
  check(editor.session.projectVariant=="large"&&editor.session.document!=original,"variant installation updates document and selection together");
  check(editor.scene.records()[0].width!=smallGeometry,"switch changes actual geometry");
  var saved=dir+"/selected.materia";editor.session.save(saved);
  var record:Null<app.ProjectSceneRecord> = SceneCodec.decodeProject(File.getContent(saved));
  check(record!=null&&record.version==2&&record.variantId=="large"&&record.jobId==null,"configuration uses variantId, not jobId");
  editor.session.newDocument();editor.session.open(saved);editor.documentChanged();
  check(editor.session.projectVariant=="large"&&editor.session.projectJob==null,"saved configuration regenerates selected geometry");
  var legacy:Dynamic=Json.parse(File.getContent(saved));legacy.project.version=1;legacy.project.jobId="large";Reflect.deleteField(legacy.project,"variantId");
  File.saveContent(saved,Json.stringify(legacy));editor.session.newDocument();editor.session.open(saved);editor.documentChanged();
  check(editor.session.projectVariant=="large","old mill jobId migrates to a real configuration");
  editor.session.save(saved);record=SceneCodec.decodeProject(File.getContent(saved));
  check(record!=null&&record.variantId=="large"&&record.jobId==null,"migrated document saves only the new identifier");
  legacy.project.variantId="large";legacy.project.version=2;
  rejects(()->SceneCodec.decodeProjectRoot(legacy),"conflicting saved selections rejected");
  var artifact=MateriaProjectRunner.exportArtifact(source,"large"),decoded=SceneArtifact.decodeView(artifact);
  var portable:Null<materia.project.SceneArtifact.SceneArtifactProject> = decoded.project;
  check(portable!=null&&portable.variant=="large"&&portable.job==null,"portable artifact retains variant semantics");
  var artifactPath=dir+"/large.mtrg";File.saveBytes(artifactPath,artifact);
  check(ProjectSourceLoader.load(artifactPath,null,null,"large").projectVariant=="large","artifact reload retains variant");
  rejects(()->ProjectSourceLoader.load(artifactPath,null,null,"small"),"prebuilt artifact cannot switch source geometry");
  var installed=editor.session.document;
  editor.projectVariants.select("small");editor.projectVariants.requestCancel();
  var cancelling=editor.projectVariants.loadingStatus();
  check(cancelling!=null&&cancelling.cancelling&&editor.projectVariants.isLoading(),"variant cancel remains visibly pending until completion");
  deadline=Sys.time()+180;
  while(editor.projectVariants.isLoading()&&Sys.time()<deadline){editor.tick();Sys.sleep(0.01);}
  check(!editor.projectVariants.isLoading()&&editor.projectVariants.snapshot().error==null&&
    editor.session.document==installed&&editor.session.projectVariant=="large","cancelled variant preserves the installed scene without an error");
  editor.dispose();fonts.dispose();
 }
}
