package app;

import Canvas;
import Color;
import GraphicsSurface;
import LayoutAxis;
import LayoutStyle;
import LayoutVisualKind;
import Rect;
import PathBuilder;
import ResolvedLayoutItem;
import nativekit.scene.SceneRenderer;
import nativekit.scene.SceneView;
import nativekit.scene.Transform;
import nativekit.ui.core.BuildContext;
import nativekit.ui.core.Key;
import nativekit.ui.core.RenderNode;
import nativekit.ui.core.UiEvent;
import nativekit.ui.core.UiEventKind;
import nativekit.ui.core.UiKey;
import nativekit.ui.core.View;
import nativekit.ui.host.DesktopUiHostContext;
import nativekit.ui.semantics.AccessibilityRole;
import nativekit.ui.semantics.Semantics;

/** Fixed-camera SceneKit GPU render composited into the editor UI. */
class EditorPerspectiveViewport implements View {
  public final key:String;
  final scene:EditorScene;
  final host:DesktopUiHostContext;
  var renderer:Null<SceneRenderer> = null;
  final style:LayoutStyle;
  var surface:Null<GraphicsSurface> = null;
  var renderedRevision:Int = -1;
  var renderedWidth:Int = 0;
  var renderedHeight:Int = 0;
  var renderedCameraRevision:Int = -1;
  var renderCount:Int = 0;
  var lastRenderSeconds:Float = 0.0;
  var totalRenderSeconds:Float = 0.0;
  final camera:PerspectiveCamera = new PerspectiveCamera();
  var navigationPointer:Null<Int> = null;
  var navigationMode:Int = 0;
  var pointerX:Float = 0.0;
  var pointerY:Float = 0.0;
  var pointerStartX:Float = 0.0;
  var pointerStartY:Float = 0.0;
  var pointerMoved:Bool = false;
  var objectDrag:Null<PerspectiveSceneDrag> = null;
  var gridSnapEnabled:Bool = false;
  var gridStep:Float = EditorSceneViewport.GRID_STEP;
  var simulationActive:Bool=false;
  var runtimeRevision:Int=0;
  var presentation:Null<EditorScene> = null;
  var presentationSceneRevision:Int=-1;
  var presentationRuntimeRevision:Int=-1;
  var simulationPoses:Array<SimulationPoseVisual> = [];
  var robotVisuals:Array<SimulationRobotVisual> = [];

  public function new(key:String, scene:EditorScene, host:DesktopUiHostContext, ?style:LayoutStyle) {
    this.key = key;
    this.scene = scene;
    this.host = host;
    this.style = style == null ? defaultStyle() : style.copy();
  }

  public function build(context:BuildContext):RenderNode {
    return context.withScope(new Key(key), function() {
      var node = new RenderNode(context.id("perspective"), LayoutVisualKind.Custom, style);
      node.hitTestSelf = true;
      node.focusable = true;
      node.semantics = new Semantics(AccessibilityRole.Image,
        "Scene perspective GPU view");
      node.onPaint(paint, "perspective:" + scene.revision + ":" + runtimeRevision + ":" + camera.revision + ":" +
        renderedWidth + "x" + renderedHeight);
      installNavigation(node);
      return node;
    });
  }

  function paint(canvas:Canvas, geometry:ResolvedLayoutItem):Void {
    ensureRenderer();
    var width = Std.int(Math.max(1.0, Math.min(2048.0, Math.ceil(geometry.width))));
    var height = Std.int(Math.max(1.0, Math.min(2048.0, Math.ceil(geometry.height))));
    var displayed=displayedScene();
    if(displayed.selectedId!=scene.selectedId)displayed.select(scene.selectedId);
    var displayRevision=scene.revision+runtimeRevision*1000003;
    if (renderer != null && (surface == null || renderedRevision != displayRevision ||
        renderedCameraRevision != camera.revision ||
        width != renderedWidth || height != renderedHeight)) {
      var started = Sys.time();
      var view = displayed.configureRenderView(new SceneView(), camera.viewProjection(width / height));
      var rendered = renderer.renderImage(displayed.renderSnapshot(), view, width, height);
      var next = GraphicsSurface.fromImage(rendered);
      rendered.dispose();
      if (surface != null) surface.dispose();
      surface = next;
      renderedRevision = displayRevision;
      renderedCameraRevision = camera.revision;
      renderedWidth = width;
      renderedHeight = height;
      lastRenderSeconds = Sys.time() - started;
      totalRenderSeconds += lastRenderSeconds;
      renderCount++;
    }
    canvas.fillRect(new Rect(0, 0, geometry.width, geometry.height),
      Color.rgba(0.025, 0.035, 0.055, 1.0));
    if (surface != null) canvas.drawSurface(surface, new Rect(0, 0, geometry.width, geometry.height));
    paintSensors(canvas,geometry.width,geometry.height);
  }

  public function diagnosticState():Dynamic return {
    width: renderedWidth,
    height: renderedHeight,
    composition: "gpu-surface",
    renders: renderCount,
    cpuTransferBytes: 0,
    lastRenderMilliseconds: lastRenderSeconds * 1000.0,
    averageRenderMilliseconds: renderCount == 0 ? 0.0 : totalRenderSeconds * 1000.0 / renderCount,
    camera: {targetX: camera.targetX, targetY: camera.targetY, targetZ: camera.targetZ,
      yaw: camera.yaw, pitch: camera.pitch, distance: camera.distance}
  };

  public function frameSelected():Void {
    var displayed=displayedScene(),selected = displayed.object(scene.selectedId);
    if (selected != null) {
      var transform = displayed.info(selected.id).worldTransform();
      var bounds=transformedBounds(transform,selected.width,selected.height,selected.depth);
      camera.frame((bounds[0]+bounds[3])/2,(bounds[1]+bounds[4])/2,(bounds[2]+bounds[5])/2,
        bounds[3]-bounds[0],bounds[4]-bounds[1],bounds[5]-bounds[2],aspect());
      return;
    }
    var items = displayed.items();
    if (items.length == 0) { camera.reset(); return; }
    var minX = 1000000000.0, minY = 1000000000.0, minZ = 1000000000.0;
    var maxX = -1000000000.0, maxY = -1000000000.0, maxZ = -1000000000.0;
    for (item in items) {
      if (!displayed.info(item.id).visible()) continue;
      var transform = displayed.info(item.id).worldTransform();
      var bounds=transformedBounds(transform,item.width,item.height,item.depth);
      minX=Math.min(minX,bounds[0]);minY=Math.min(minY,bounds[1]);minZ=Math.min(minZ,bounds[2]);
      maxX=Math.max(maxX,bounds[3]);maxY=Math.max(maxY,bounds[4]);maxZ=Math.max(maxZ,bounds[5]);
    }
    if (minX == 1000000000.0) { camera.reset(); return; }
    camera.frame((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2,
      maxX - minX, maxY - minY, maxZ - minZ, aspect());
  }

  public function resetView():Void camera.reset();

  public function setPlacementOptions(snap:Bool, step:Float):Void {
    gridSnapEnabled = snap;
    gridStep = step;
  }
  public function setSimulationState(active:Bool,poses:Array<SimulationPoseVisual>,revision:Int,
      ?robots:Array<SimulationRobotVisual>):Void {
    simulationActive=active;simulationPoses=poses==null?[]:poses.copy();runtimeRevision=revision;
    robotVisuals=robots==null?[]:robots.copy();
    if(!active&&presentation!=null){presentation.dispose();presentation=null;}
  }
  public function editingEnabled():Bool return !simulationActive;

  public function dragging():Bool return objectDrag != null;

  public function commitDrag():Null<PerspectivePointer> {
    if (objectDrag == null) return null;
    objectDrag.commit(); objectDrag = null;
    return releaseNavigation();
  }

  public function cancelDrag():Null<PerspectivePointer> {
    if (objectDrag == null) return null;
    objectDrag.cancel(); objectDrag = null;
    return releaseNavigation();
  }

  function releaseNavigation():Null<PerspectivePointer> {
    var pointer = navigationPointer;
    navigationPointer = null; navigationMode = 0;
    return pointer == null ? null : new PerspectivePointer(pointer, pointerX, pointerY);
  }

  public function pick(localX:Float, localY:Float):String {
    return pickScene(displayedScene(), camera, Math.max(1, renderedWidth), Math.max(1, renderedHeight),
      localX, localY);
  }

  public static function pickScene(scene:EditorScene, camera:PerspectiveCamera,
      width:Float, height:Float, localX:Float, localY:Float):String {
    var ray = camera.screenRay(localX, localY, width, height);
    var closest = 1000000000.0;
    var result = "scene";
    for (item in scene.items()) {
      var state = scene.info(item.id);
      if (!state.visible()) continue;
      var transform = state.worldTransform();
      var centerX = transform.element(12), centerY = transform.element(13);
      var centerZ = transform.element(14);
      var distance = rayBoxDistance(ray,transform,item.width,item.height,item.depth);
      if (distance != null && distance < closest) {
        closest = distance;
        result = item.id;
      }
    }
    return result;
  }

  static function rayBoxDistance(ray:PerspectiveRay,transform:Transform,
      width:Float,height:Float,depth:Float):Null<Float> {
    var delta=[ray.originX-transform.element(12),ray.originY-transform.element(13),ray.originZ-transform.element(14)];
    var origins=[for(column in 0...3)delta[0]*transform.element(column*4)+
      delta[1]*transform.element(column*4+1)+delta[2]*transform.element(column*4+2)];
    var directions=[for(column in 0...3)ray.directionX*transform.element(column*4)+
      ray.directionY*transform.element(column*4+1)+ray.directionZ*transform.element(column*4+2)];
    var minimums=[-width/2,-height/2,-depth/2],maximums=[width/2,height/2,depth/2];
    var near = 0.0, far = 1000000000.0;
    for (axis in 0...3) {
      var direction = directions[axis], origin = origins[axis];
      if (Math.abs(direction) < 0.000001) {
        if (origin < minimums[axis] || origin > maximums[axis]) return null;
      } else {
        var first = (minimums[axis] - origin) / direction;
        var second = (maximums[axis] - origin) / direction;
        if (first > second) { var swap = first; first = second; second = swap; }
        near = Math.max(near, first); far = Math.min(far, second);
        if (near > far) return null;
      }
    }
    return far < 0.0 ? null : near;
  }
  static function transformedBounds(transform:Transform,width:Float,height:Float,depth:Float):Array<Float>{
    var result=[1e300,1e300,1e300,-1e300,-1e300,-1e300];
    for(x in [-width/2,width/2])for(y in [-height/2,height/2])for(z in [-depth/2,depth/2]){
      var local=[x,y,z];for(row in 0...3){var value=transform.element(12+row);
        for(column in 0...3)value+=transform.element(column*4+row)*local[column];
        result[row]=Math.min(result[row],value);result[row+3]=Math.max(result[row+3],value);}}
    return result;
  }

  public function dispose():Void {
    if (surface != null) surface.dispose();
    surface = null;
    if (renderer != null) renderer.dispose();
    renderer = null;
    if(presentation!=null)presentation.dispose();presentation=null;
  }

  function displayedScene():EditorScene {
    if(!simulationActive)return scene;
    if(presentation==null||presentationSceneRevision!=scene.revision||
        presentationRuntimeRevision!=runtimeRevision){
      if(presentation!=null)presentation.dispose();presentation=new EditorScene(scene.records());
      for(pose in simulationPoses)if(presentation.object(pose.id)!=null)
        presentation.setPresentationPose(pose.id,pose.position,pose.rotation);
      presentationSceneRevision=scene.revision;presentationRuntimeRevision=runtimeRevision;
    }
    return presentation;
  }
  function paintSensors(canvas:Canvas,width:Float,height:Float):Void {
    if(!simulationActive)return;
    for(robot in robotVisuals){
      for(link in robot.links){var point=camera.project(link.position[0],link.position[1],link.position[2],width,height);
        if(point!=null)canvas.fillRect(new Rect(point.x-4,point.y-4,8,8),Color.rgba(0.2,0.85,0.55,0.9));}
      for(sensor in robot.sensors){
      var linkPosition=robot.position,linkRotation=robot.rotation;
      for(link in robot.links)if(link.id==sensor.linkId){linkPosition=link.position;linkRotation=link.rotation;break;}
      var offset=rotateVector(linkRotation,sensor.mountPosition.toArray());
      var origin=[linkPosition[0]+offset[0],linkPosition[1]+offset[1],linkPosition[2]+offset[2]];
      var mount=camera.project(origin[0],origin[1],origin[2],width,height);if(mount==null)continue;
      canvas.fillRect(new Rect(mount.x-3,mount.y-3,6,6),Color.rgba(1.0,0.75,0.2,0.95));
      if(sensor.kind!="lidar"||sensor.values.length==0)continue;
      var rotation=multiplyQuaternion(linkRotation,sensor.mountRotation.toArray()),path=new PathBuilder();
      var values=sensor.values.toArray();
      for(index in 0...values.length){var angle=index*6.283185307179586/values.length;
        var direction=rotateVector(rotation,[Math.cos(angle),Math.sin(angle),0.0]);
        var hit=camera.project(origin[0]+direction[0]*values[index],origin[1]+direction[1]*values[index],
          origin[2]+direction[2]*values[index],width,height);
        if(hit!=null)path.moveTo(mount.x,mount.y).lineTo(hit.x,hit.y);}
      canvas.strokeTransient(path.build(),Color.rgba(0.25,0.8,1.0,0.55),1.0);
    }
    }
  }
  static function rotateVector(q:Array<Float>,v:Array<Float>):Array<Float>{
    var x=q[0],y=q[1],z=q[2],w=q[3],tx=2*(y*v[2]-z*v[1]),ty=2*(z*v[0]-x*v[2]),tz=2*(x*v[1]-y*v[0]);
    return [v[0]+w*tx+y*tz-z*ty,v[1]+w*ty+z*tx-x*tz,v[2]+w*tz+x*ty-y*tx];
  }
  static function multiplyQuaternion(a:Array<Float>,b:Array<Float>):Array<Float>return[
    a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
    a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]];

  function ensureRenderer():Void {
    if (renderer != null || host.gpuRendererId == 0) return;
    renderer = SceneRenderer.createBorrowedId(host.gpuRendererId);
  }

  function aspect():Float return renderedWidth <= 0 || renderedHeight <= 0
    ? 1.0 : renderedWidth / renderedHeight;

  function installNavigation(node:RenderNode):Void {
    node.on(UiEventKind.PointerDown, function(event:UiEvent) {
      if (event.button != 0 && event.button != 2) return;
      navigationPointer = event.pointerId;
      navigationMode = event.button == 0 ? 1 : 2;
      if (event.button == 0) {
        var hit = pick(event.localX, event.localY);
        if (hit != "scene"&&editingEnabled()) {
          scene.select(hit);
          objectDrag = PerspectiveSceneDrag.begin(scene, camera, hit, event.localX, event.localY,
            Math.max(1, renderedWidth), Math.max(1, renderedHeight), gridSnapEnabled, gridStep);
          if (objectDrag != null) navigationMode = 3;
        }
      }
      pointerX = event.x; pointerY = event.y;
      pointerStartX = event.x; pointerStartY = event.y; pointerMoved = false;
      event.capturePointer(); event.preventDefault(); event.stopPropagation();
    });
    node.on(UiEventKind.PointerMove, function(event:UiEvent) {
      if (navigationPointer == null || event.pointerId != navigationPointer) return;
      var deltaX = event.x - pointerX, deltaY = event.y - pointerY;
      pointerX = event.x; pointerY = event.y;
      if (Math.abs(event.x - pointerStartX) >= 3.0 || Math.abs(event.y - pointerStartY) >= 3.0)
        pointerMoved = true;
      if (navigationMode == 1) camera.orbit(deltaX, deltaY);
      else if (navigationMode == 2) camera.pan(deltaX, deltaY, Math.max(1, renderedHeight));
      else if (objectDrag != null) objectDrag.update(camera, event.localX, event.localY,
        Math.max(1, renderedWidth), Math.max(1, renderedHeight));
      event.preventDefault(); event.stopPropagation();
    });
    var finish = function(event:UiEvent) {
      if (navigationPointer == null || event.pointerId != navigationPointer) return;
      if (navigationMode == 3 && objectDrag != null) {
        if (event.kind == UiEventKind.PointerUp) objectDrag.commit(); else objectDrag.cancel();
        objectDrag = null;
      } else if (event.kind == UiEventKind.PointerUp && navigationMode == 1 && !pointerMoved)
        scene.select(pick(event.localX, event.localY));
      navigationPointer = null; navigationMode = 0;
      event.releasePointer(); event.preventDefault(); event.stopPropagation();
    };
    node.on(UiEventKind.PointerUp, finish);
    node.on(UiEventKind.PointerCancel, finish);
    node.on(UiEventKind.Scroll, function(event:UiEvent) {
      camera.zoom(event.deltaY);
      event.preventDefault(); event.stopPropagation();
    });
    node.on(UiEventKind.KeyDown, function(event:UiEvent) {
      if (event.key != UiKey.Escape || objectDrag == null) return;
      objectDrag.cancel(); objectDrag = null;
      navigationPointer = null; navigationMode = 0;
      event.releasePointer(); event.preventDefault(); event.stopPropagation();
    });
  }

  static function defaultStyle():LayoutStyle {
    var result = new LayoutStyle();
    result.width = LayoutAxis.stretch();
    result.height = LayoutAxis.stretch();
    result.clipToParent = true;
    return result;
  }
}

class PerspectivePointer {
  public final id:Int;
  public final x:Float;
  public final y:Float;
  public function new(id:Int, x:Float, y:Float) {
    this.id = id; this.x = x; this.y = y;
  }
}
