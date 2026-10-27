package tests;

import haxeon.platform.event.EventPump;

import app.EditorScene;
import app.EditorPerspectiveViewport;
import nativekit.scene.SceneView;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutFrame;
import haxeon.ui.LayoutStyle;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.UiContext;
import haxeon.ui.host.UiHostContext;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.layout.Spacer;

/** Exercise real layout/event coordinates independently of the capped GPU render target. */
@:access(app.EditorPerspectiveViewport)
@:access(tests.SceneEditingTests)
class ViewportCoordinatesTests {
  static var assertions=0;
  static function check(ok:Bool,message:String):Void {assertions++;if(!ok)throw message;}
  static function near(actual:Float,expected:Float,message:String):Void
    check(Math.abs(actual-expected)<0.01,message+': $actual != $expected');
  static function viewportNode(root:RenderNode):Null<RenderNode> {
    if(root.semantics!=null && root.semantics.label=="Scene perspective GPU view")return root;
    for(child in root.children){var found=viewportNode(child);if(found!=null)return found;}
    return null;
  }
  static function fill():LayoutStyle {
    var style=new LayoutStyle();style.width=LayoutAxis.grow();style.height=LayoutAxis.grow();return style;
  }
  static function sectionInspection():Void {
    var scene=new EditorScene([{id:"part",label:"Part",type:"rectangle",x:0.0,y:0.0,z:0.0,
      width:0.2,height:0.2,depth:0.4,collisionEnabled:false,dynamicBody:false,mass:1.0,
      red:0.5,green:0.5,blue:0.5,visible:true}]);
    var host=new UiHostContext(null,new EventPump(),function(){},function(){});
    var viewport=new EditorPerspectiveViewport("section-test",scene,host,fill()),ui=new UiContext();
    var definition:materia.assembly.AssemblyDefinition={schemaVersion:materia.assembly.AssemblyDefinitionCodec.VERSION,id:"section-test",
      definitions:[{id:"part",connectors:[]}],occurrences:[{id:"part",definition:"part",initialPose:materia.assembly.AssemblyFrames.identity()}],joints:[]};
    var record=materia.presentation.Presentations.assembled(definition);
    viewport.setPresentation(record,definition,null,1);
    viewport.onSectionEdit=planes->{
      var next:materia.presentation.AssemblyPresentation=haxeon.wire.JsonWire.decode(haxeon.wire.JsonWire.encode(record));
      next.views[0].sections=planes;record=next;viewport.setPresentation(record,definition,null,1);
    };
    function section():Array<Float> {
      var frame=viewport.presentationFrame();if(frame==null||frame.sections.length==0)throw "Missing section plane";
      var plane=frame.sections[0],n=plane.normal,p=plane.point;
      return [n.x,n.y,n.z,-n.x*p.x-n.y*p.y-n.z*p.z];
    }

    try {
      ui.submit(viewport,new LayoutFrame(800,600));
      var revision=scene.revision,key=viewport.presentationKey();
      viewport.inspectObject("part");
      check(viewport.sectionActive(),"inspection opens a section plane");
      check(viewport.presentationKey()!=key,"inspection invalidates retained viewport");
      var plane=section();
      check(viewport.sectionView(new SceneView()).clipPlaneCount()==1,"section plane reaches rendering and picking views");
      var ray=viewport.camera.screenRay(400,300,800,600);
      var view=viewport.sectionView(scene.configureRenderView(new SceneView(),viewport.camera.viewProjection(800/600)));
      var hit=scene.pickHitRayWithView(view,ray.originX,ray.originY,ray.originZ,ray.directionX,ray.directionY,ray.directionZ);
      check(hit.id=="part","section picking reaches retained geometry");
      check(plane[0]*hit.x+plane[1]*hit.y+plane[2]*hit.z+plane[3]>=-1e-6,"section picking excludes clipped front faces");
      viewport.updateHover(400,300);check(viewport.hoveredId()=="part","section hover follows the clipped view");
      key=viewport.presentationKey();viewport.flipSection();
      var flipped=section();
      for(i in 0...4)near(flipped[i],-plane[i],"flip reverses the section side");
      check(viewport.presentationKey()!=key,"flip invalidates retained viewport");
      viewport.clearSection();check(!viewport.sectionActive(),"close section restores the complete view");
      check(viewport.sectionView(new SceneView()).clipPlaneCount()==0,"closed section leaves no clip planes");
      check(scene.revision==revision,"section inspection does not edit scene geometry or visibility");
      ui.dispose();viewport.dispose();scene.dispose();
    }catch(error:Dynamic){ui.dispose();viewport.dispose();scene.dispose();throw error;}
  }
  public static function main():Int {
    var scene=new EditorScene([{id:"target",label:"Target",type:"rectangle",x:0.3,y:-0.2,z:0.0,
      width:0.2,height:0.2,depth:0.2,collisionEnabled:false,dynamicBody:false,mass:1.0,
      red:0.5,green:0.5,blue:0.5,visible:true}]);
    var host=new UiHostContext(null,new EventPump(),function(){},function(){});
    var viewport=new EditorPerspectiveViewport("coordinate-test",scene,host,fill()), ui=new UiContext();
    var row=new Row("docked-row",[
      new KeyedView("left",new Spacer("hierarchy",LayoutAxis.fixed(300),LayoutAxis.grow())),
      new KeyedView("viewport",viewport),
      new KeyedView("right",new Spacer("inspector",LayoutAxis.fixed(200),LayoutAxis.grow()))],fill());
    var layout=new Column("docked-column",[
      new KeyedView("toolbar",new Spacer("toolbar",LayoutAxis.grow(),LayoutAxis.fixed(80))),
      new KeyedView("body",row)],fill());
    try {
      // Wide, tall, both capped, fractional layout, and resizing back below the cap.
      for(size in [[3840.0,2160.0],[1700.0,3000.0],[3600.0,2800.0],[3100.5,1700.25],[1400.0,900.0]]) {
        var node=viewportNode(ui.submit(layout,new LayoutFrame(size[0],size[1])));
        check(node!=null,"docked viewport resolves");
        var bounds=node.globalBounds(), width=bounds.width,height=bounds.height;
        near(bounds.x,300,"viewport accounts for the hierarchy dock");
        near(bounds.y,80,"viewport accounts for the toolbar");
        near(viewport.viewportWidth,width,"input dimensions update during layout before painting");
        near(viewport.viewportHeight,height,"input height updates during layout before painting");
        // Model the texture retained by the renderer; input must not use these dimensions.
        viewport.renderedWidth=Std.int(Math.min(2048,Math.ceil(width)));
        viewport.renderedHeight=Std.int(Math.min(2048,Math.ceil(height)));
        near(viewport.aspect(),width/height,"projection follows the displayed aspect ratio");
        scene.setPositionXY("target",0.3,-0.2);
        viewport.camera.frame(0,0,0,4,3,2,width/height);
        var point=viewport.camera.project(0.3,-0.2,0,width,height);
        if(point==null)throw "Target is behind the camera";
        check(viewport.pick(point.x,point.y)=="target","pick matches the displayed object after resize");
        if(width>2048 || height>2048)
          check(EditorPerspectiveViewport.pickScene(scene,viewport.camera,viewport.renderedWidth,
            viewport.renderedHeight,point.x,point.y)!="target","fixture reproduces the old texture-coordinate miss");
        var x=bounds.x+point.x,y=bounds.y+point.y;
        ui.events.pointerMove(x,y);
        check(viewport.hoveredId()=="target","hover uses viewport-local layout coordinates");
        ui.events.pointerDown(x,y,0);
        check(viewport.objectDrag==null,"press waits for click release or drag threshold");
        ui.events.pointerMove(x+36,y-18);
        ui.events.pointerUp(x+36,y-18,0);
        var target=scene.object("target");
        if(target==null)throw "Missing target";
        var moved=viewport.camera.project(target.x,target.y,target.z,width,height);
        if(moved==null)throw "Moved target is behind the camera";
        near(moved.x,point.x+36,"drag follows the cursor horizontally");
        near(moved.y,point.y-18,"drag follows the cursor vertically");
        var ray=viewport.camera.screenRay(moved.x,moved.y,width,height);
        var view=scene.configureRenderView(new SceneView(),viewport.camera.viewProjection(width/height));
        var hit=scene.pickHitRayWithView(view,ray.originX,ray.originY,ray.originZ,ray.directionX,ray.directionY,ray.directionZ);
        check(hit.id=="target","zoom has a visible surface anchor");
        var distance=viewport.camera.distance;
        ui.scroll(bounds.x+moved.x,bounds.y+moved.y,0,-120);
        check(viewport.camera.distance<distance,"wheel zoom reaches the docked viewport");
        var anchor=viewport.camera.project(hit.x,hit.y,hit.z,width,height);
        if(anchor==null)throw "Zoom anchor is behind the camera";
        near(anchor.x,moved.x,"zoom preserves the cursor's surface anchor horizontally");
        near(anchor.y,moved.y,"zoom preserves the cursor's surface anchor vertically");
        // Starting orbit chooses the visible surface without moving the camera.
        var eye=viewport.camera.eyePosition(),focus=[viewport.camera.targetX,viewport.camera.targetY,viewport.camera.targetZ];
        distance=viewport.camera.distance;
        ui.pointerDown(bounds.x+anchor.x,bounds.y+anchor.y,2);
        var pivot=viewport.orbitPivot;if(pivot==null)throw "Orbit did not choose a pivot";
        for(i in 0...3) {
          near(pivot[i],[hit.x,hit.y,hit.z][i],"orbit chooses the displayed surface after zoom and resize");
          check(Math.abs(viewport.camera.eyePosition()[i]-eye[i])<1e-8,"orbit press keeps eye unchanged");
        }
        near(viewport.camera.targetX,focus[0],"orbit press does not recenter X");
        near(viewport.camera.targetY,focus[1],"orbit press does not recenter Y");
        near(viewport.camera.targetZ,focus[2],"orbit press does not recenter Z");
        var radius=0.0;for(i in 0...3)radius+=Math.pow(eye[i]-pivot[i],2);
        for(step in 1...4) {
          ui.submit(layout,new LayoutFrame(size[0],size[1]));
          ui.pointerMove(bounds.x+anchor.x+step*25,bounds.y+anchor.y+step*12);
          check(viewport.orbitPivot==pivot,"orbit pivot stays fixed across view rebuilds");
          var projected=viewport.camera.project(pivot[0],pivot[1],pivot[2],width,height);
          if(projected==null)throw "Orbit pivot moved behind camera";
          near(projected.x,anchor.x,"off-centre orbit pivot stays horizontally fixed");
          near(projected.y,anchor.y,"off-centre orbit pivot stays vertically fixed");
        }
        ui.pointerUp(bounds.x+anchor.x+75,bounds.y+anchor.y+36,2);
        var nextEye=viewport.camera.eyePosition(),nextRadius=0.0;
        for(i in 0...3)nextRadius+=Math.pow(nextEye[i]-pivot[i],2);
        near(nextRadius,radius,"orbit preserves distance from the surface pivot");
        near(viewport.camera.distance,distance,"surface orbit preserves zoom");
        check(viewport.pick(2,2)=="scene","empty-space orbit fixture misses geometry");
        ui.pointerDown(bounds.x+2,bounds.y+2,2);
        check(viewport.orbitPivot==pivot,"empty-space drag keeps the previous pivot");
        ui.pointerMove(bounds.x+12,bounds.y+8);ui.pointerUp(bounds.x+12,bounds.y+8,2);
        viewport.resetView();check(viewport.orbitPivot==null,"reset clears the surface pivot");

      }
    } catch(error:Dynamic) {ui.dispose();viewport.dispose();scene.dispose();throw error;}
    ui.dispose();viewport.dispose();scene.dispose();
    sectionInspection();
    SceneEditingTests.perspectiveHoverInput();
    SceneEditingTests.dockedNavigationInput();
    Sys.println('Viewport coordinates: $assertions assertions passed, plus hover and docked navigation regressions');
    return 0;
  }
}
