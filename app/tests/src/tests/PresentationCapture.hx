package tests;

/** Capture a named presentation through the real editor without saving its session cursor. */
@:access(app.Main)
@:access(app.Main.ReferenceEditorApp)
@:access(app.EditorPerspectiveViewport)
class PresentationCapture {
  static function mean(values:Array<Float>):Float {var sum=0.0;for(value in values)sum+=value;return sum/values.length;}
  public static function main():Int {
    var args=Sys.args(),id:Null<String> = null;
    var launch:Array<String> = [];
    var explode=0.0,distanceScale=1.0,step=0,dragFrames=0;
    var dragSamples:Array<Float> = [],dragTick=0,lastTick=0.0,lastFrame=-1;
    var section:Null<materia.presentation.AssemblyPresentation.SectionPlane> = null;
    var originalPoint:Null<materia.assembly.AssemblyDefinition.AssemblyVector> = null;
    var shaded=false;
    var sectionMode="contours",hide:Array<String> = [],yaw=Math.NaN,pitch=Math.NaN;
    var uiSamples:Array<Float> = [],paintSamples:Array<Float> = [],submitSamples:Array<Float> = [],nativeSamples:Array<Float> = [];
    var procedure:Null<String> = null,editOffset:Null<String> = null;
    for(arg in args) {
      if(arg=="--shaded")shaded=true;
      else if(StringTools.startsWith(arg,"--section-mode="))sectionMode=arg.substr(15);
      else if(StringTools.startsWith(arg,"--hide-occurrence="))hide.push(arg.substr(18));
      else if(StringTools.startsWith(arg,"--camera-yaw="))yaw=Std.parseFloat(arg.substr(13));
      else if(StringTools.startsWith(arg,"--camera-pitch="))pitch=Std.parseFloat(arg.substr(15));
      else if(StringTools.startsWith(arg,"--view="))id=arg.substr(7);
      else if(StringTools.startsWith(arg,"--explode="))explode=Std.parseFloat(arg.substr(10));
      else if(StringTools.startsWith(arg,"--distance-scale="))distanceScale=Std.parseFloat(arg.substr(17));
      else if(StringTools.startsWith(arg,"--section-drag=")) {
        var count=Std.parseInt(arg.substr(15));if(count==null||count<1)throw "Invalid section drag frame count";dragFrames=count;
      }
      else if(StringTools.startsWith(arg,"--edit-offset="))editOffset=arg.substr(14);
      else if(StringTools.startsWith(arg,"--procedure="))procedure=arg.substr(12);
      else if(StringTools.startsWith(arg,"--step=")) {
        var parsed=Std.parseInt(arg.substr(7));if(parsed==null||parsed<0)throw "Invalid procedure step";step=parsed;
      }
      else launch.push(arg);
    }
    if(id==null)throw "PresentationCapture requires --view=ID and editor launch arguments";
    if(sectionMode!="none"&&sectionMode!="caps"&&sectionMode!="contours")throw "Invalid section mode";
    Sys.putEnv("NK_SCENE_SECTION_CONTOURS",sectionMode=="caps"?"0":"1");
    var live=app.Main.open(launch),selected=false;
    while(live.tick()) {
      var editor=app.Main.liveEditor;
      if(!selected&&editor!=null&&!editor.openingProject()) {
        var viewport=editor.perspectiveViewport;
        if(viewport!=null&&viewport.presentation!=null) {
          if(shaded)editor.scene.setCadEdgeMode(Shaded);
          var presentation=viewport.presentation;if(presentation==null)throw "Missing presentation";
          var selectedView=presentation.record.views.filter(view->view.id==id)[0];
          Sys.println("capture-visibility "+haxe.Json.stringify(selectedView.visibility));
          for(occurrence in hide)selectedView.visibility.push({select:{occurrence:occurrence,subtree:false},mode:Hidden});
          viewport.selectPresentationView(id);viewport.setPresentationExplode(explode);
          if(procedure!=null) {
            viewport.selectPresentationProcedure(procedure);
            var controller=viewport.presentation;if(controller!=null)for(_ in 0...step)controller.next(false);
          }
          viewport.presentationCameraDirty=true;viewport.updatePresentationCamera();
          viewport.camera.setTarget(viewport.camera.targetX,viewport.camera.targetY,viewport.camera.targetZ,viewport.camera.distance*distanceScale);
          if(Math.isFinite(yaw)||Math.isFinite(pitch))viewport.camera.setAngle(Math.isFinite(yaw)?yaw:viewport.camera.yaw,Math.isFinite(pitch)?pitch:viewport.camera.pitch);
          if(editOffset!=null){editor.scene.select("project:"+editOffset);editor.editPresentationOffset(viewport);}
          if(dragFrames>0) {
            var controller=viewport.presentation;if(controller==null)throw "Missing presentation controller";
            var view=controller.record.views.filter(view->view.id==id)[0];
            if(view.sections.length==0)throw "Section drag requires an active section";
            section=view.sections[0];originalPoint=section.point;
            if(sectionMode=="none")view.sections=[];
          }
          editor.invalidateView();selected=true;lastTick=Sys.time();
          var metrics=editor.ui.frameMetrics;if(metrics!=null)lastFrame=metrics.frameNumber;
        }
      } else if(selected&&dragFrames>0&&section!=null&&editor!=null) {
        var metrics=editor.ui.frameMetrics;
        if(metrics==null||metrics.frameNumber==lastFrame)continue;
        lastFrame=metrics.frameNumber;
        var now=Sys.time(),viewport=editor.perspectiveViewport;
        if(viewport==null)throw "Missing presentation viewport";
        var controller=viewport.presentation;if(controller==null)throw "Missing presentation controller";
        var point=originalPoint;if(point==null)throw "Missing original section point";
        if(dragTick>=8&&dragTick<dragFrames+8) {
          dragSamples.push((now-lastTick)*1000);
          uiSamples.push(metrics.totalSeconds*1000);paintSamples.push(metrics.customPaintSeconds*1000);
          submitSamples.push(metrics.submitSeconds*1000);nativeSamples.push(metrics.nativeRenderSeconds*1000);
          Sys.println("section-drag-frame "+haxe.Json.stringify({tick:dragTick-8,uiFrame:metrics.frameNumber,
            sampledAtSeconds:now,intervalMs:(now-lastTick)*1000,uiMs:metrics.totalSeconds*1000,
            customPaintMs:metrics.customPaintSeconds*1000,submitMs:metrics.submitSeconds*1000,nativeRenderMs:metrics.nativeRenderSeconds*1000}));
        }
        lastTick=now;
        if(dragTick<dragFrames+8) {
          var delta=2.0*Math.sin(dragTick*2*Math.PI/dragFrames),n=section.normal;
          section.point={x:point.x+n.x*delta,y:point.y+n.y*delta,z:point.z+n.z*delta};
          controller.selectView(id);editor.invalidateView();dragTick++;
        } else if(dragTick==dragFrames+8) {
          section.point=point;controller.selectView(id);editor.invalidateView();dragTick++;
          dragSamples.sort((a,b)->a<b?-1:a>b?1:0);
          var sum=0.0;for(ms in dragSamples)sum+=ms;
          Sys.println("section-drag "+haxe.Json.stringify({view:id,mode:sectionMode,frames:dragSamples.length,
            uiMeanMs:mean(uiSamples),customPaintMeanMs:mean(paintSamples),submitMeanMs:mean(submitSamples),nativeRenderMeanMs:mean(nativeSamples),
            meanMs:sum/dragSamples.length,p95Ms:dragSamples[Std.int(dragSamples.length*0.95)],maxMs:dragSamples[dragSamples.length-1]}));
        }
      }
    }
    if(dragFrames>0&&dragSamples.length!=dragFrames)throw "Section drag capture ended before all frames were measured";
    var result=live.close();
    if(!selected)throw "No assembly presentation opened for capture";
    return result;
  }
}
