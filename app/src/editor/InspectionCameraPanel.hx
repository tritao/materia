package app.editor;

import app.Main.ReferenceEditorApp;
import app.EditorPerspectiveViewport;
import materia.project.SceneArtifact.SceneArtifactInspectionCamera;
import haxeon.ui.core.View;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.layout.SizedBox;
import haxeon.ui.widgets.text.Text;

/** One renderer and camera, hosted either in a dock or a picture-in-picture layer. */
@:access(app.Main.ReferenceEditorApp)
class InspectionCameraPanel {
  public var overlay(default,null)=false;
  var viewport:Null<EditorPerspectiveViewport>;
  var generation=-1;
  var selected=0;
  var zoom=1.0;

  public function new() {}
  public function showDocked():Void overlay=false;
  public function presentationKey():String return overlay+":"+generation+":"+selected+":"+zoom;
  public function dispose():Void {if(viewport!=null)viewport.dispose();viewport=null;}
  public function build(app:ReferenceEditorApp,inOverlay:Bool=false):View {
    if(generation!=app.session.generation) {dispose();generation=app.session.generation;selected=0;zoom=1;}
    var style=new LayoutStyle();style.width=LayoutAxis.grow();style.height=LayoutAxis.grow();style.childGap=4;
    var rows:Array<KeyedView> = [];
    if(overlay&&!inOverlay) {
      var dock=new Button("Dock camera here",null,()->{overlay=false;app.invalidateView();},"inspection-dock-here");
      return new Column("inspection-overlay-notice",[new KeyedView("notice",new Text("Camera is shown in the main viewport.")),new KeyedView("dock",dock)],style);
    }
    var presets=app.session.inspectionCameras;
    if(presets.length==0) {
      var id=app.scene.selectedId;
      if(id==null||app.scene.object(id)==null)return new Text("Select a part to inspect it, or open a scene with camera presets.");
      var item:app.EditorScene.EditorSceneObject=app.scene.object(id);
      presets=[{id:"selection",label:"Selected part",anchor:id,offset:[0,0,0],span:Math.max(0.01,Math.max(item.width,Math.max(item.height,item.depth))*1.5),yaw:-1.1,pitch:0.35,hiddenPrefixes:[]}];
    }
    if(selected>=presets.length)selected=0;
    var actions:Array<KeyedView> = [];
    for(i in 0...presets.length) {
      var at=i,preset=presets[i];
      var button=new Button(preset.label,null,()->{selected=at;zoom=1;app.invalidateView();},"inspection-preset-"+preset.id);
      button.enabled=i!=selected;actions.push(new KeyedView(preset.id,button));
    }
    rows.push(new KeyedView("presets",new Row("inspection-presets",actions)));
    var tools:Array<KeyedView> = [];
    function action(id:String,label:String,fn:Void->Void):Void tools.push(new KeyedView(id,new Button(label,null,fn,"inspection-"+id)));
    action("zoom-in","Zoom +",()->{zoom=Math.max(0.2,zoom/1.4);app.invalidateView();});
    action("zoom-out","Zoom -",()->{zoom=Math.min(8,zoom*1.4);app.invalidateView();});
    action("reset","Reset",()->{zoom=1;app.invalidateView();});
    action("presentation",inOverlay?"Dock":"Picture in picture",()->{overlay=!inOverlay;
      if(overlay){app.workspace.close("inspection-camera");app.workspace.activate("perspective");app.invalidateView();}
      else app.openInspectionCamera();});
    rows.push(new KeyedView("tools",new Row("inspection-tools",tools)));
    var preset=presets[selected],anchor=app.scene.object("project:"+preset.anchor);
    if(anchor==null)anchor=app.scene.object(preset.anchor);
    if(anchor==null) {rows.push(new KeyedView("missing",new Text("Camera target is unavailable in this scene.")));return new Column("inspection-missing",rows,style);}
    if(app.hostContext==null) {rows.push(new KeyedView("desktop",new Text("Inspection rendering requires the desktop GPU host.")));return new Column("inspection-desktop",rows,style);}
    if(viewport==null) {viewport=new EditorPerspectiveViewport("inspection-camera-view",app.scene,app.hostContext);viewport.setPlacementOptions(false,0,false);viewport.setSimulationOverlays(false);viewport.setCosmeticThreads(false);}
    viewport.setSampleCount(app.antialiasing);viewport.setLightingPreset(app.lightingPreset);
    var frame=app.framePresentation;
    viewport.setSimulationState(app.simulation.isActive(),frame==null?[]:frame.environment,frame==null?0:frame.revision);
    viewport.setInspectionCamera(preset,zoom);
    rows.push(new KeyedView("camera",new SizedBox("inspection-image",viewport,LayoutAxis.grow(),LayoutAxis.grow())));
    rows.push(new KeyedView("caption",new Text("Steady view · follows position · visibility applies only to this camera")));
    return new Column("inspection-camera",rows,style);
  }
}
