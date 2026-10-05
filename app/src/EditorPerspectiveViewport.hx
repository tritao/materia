package app;

import haxeon.platform.GraphicsImageRef;
import haxeon.ui.Image;

import haxeon.ui.Canvas;
import app.MissionPlayer.MissionOverlay;
import haxeon.ui.Color;
import haxeon.ui.GraphicsSurface;
import haxeon.ui.GradientStop;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.LayoutVisualKind;
import haxeon.ui.Rect;
import haxeon.ui.PathBuilder;
import haxeon.ui.ResolvedLayoutItem;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import nativekit.scene.SceneRenderer;
import nativekit.scene.SceneView;
import nativekit.scene.Transform;
import haxeon.ui.core.BuildContext;
import haxeon.ui.core.Key;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.UiEvent;
import haxeon.ui.core.UiEventKind;
import haxeon.ui.core.UiKey;
import haxeon.ui.core.View;
import haxeon.ui.host.UiHostContext;
import haxeon.ui.semantics.AccessibilityRole;
import haxeon.ui.semantics.Semantics;

/** Fixed-camera SceneKit GPU render composited into the editor UI. */
class EditorPerspectiveViewport implements View {
  public final key:String;
  final scene:EditorScene;
  final host:UiHostContext;
  var renderer:Null<SceneRenderer> = null;
  final style:LayoutStyle;
  var surface:Null<GraphicsSurface> = null;
  var renderedRevision:String = "";
  var renderedWidth:Int = 0;
  var renderedHeight:Int = 0;
  var renderedCameraRevision:Int = -1;
  var renderedLightingRevision:Int = -1;
  var renderedHoverRevision:Int = -1;
  var lightingRevision:Int = 0;
  var lightingPreset:Int = 0;
  var renderCount:Int = 0;
  var lastRenderSeconds:Float = 0.0;
  var totalRenderSeconds:Float = 0.0;
  final camera:PerspectiveCamera = new PerspectiveCamera();
  var navigationPointer:Null<Int> = null;
  var navigationMode:Int = 0;
  var sketchDragRevision:Int = 0;
  var pointerX:Float = 0.0;
  var pointerY:Float = 0.0;
  var pointerStartX:Float = 0.0;
  var pointerStartY:Float = 0.0;
  var pointerMoved:Bool = false;
  var hoveredObjectId:Null<String> = null;
  var hoveredFaceIndex:Int = -1;
  var hoverRevision:Int = 0;
  var objectDrag:Null<PerspectiveSceneDrag> = null;
  /** An IK drag of a joint-driven part: the pressed point follows a screen-parallel plane. */
  var assemblyDrag:Null<SceneAssemblyDrag> = null;
  var assemblyDragPlane:Array<Float> = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
  var assemblyDragRevision:Int = 0;
  /** The face mate being picked (see `MatePickTool`), or null. */
  var matePick:Null<MatePickTool> = null;
  var matePickRevision:Int = 0;
  var sketchRectangleDrag:Null<PerspectiveSketchRectangleDrag> = null;
  /** A sketch draft point is being dragged (navigation mode 6). */
  var sketchPointDragging:Bool = false;
  var gridSnapEnabled:Bool = false;
  var gridStep:Float = app.editor.EditorGrid.STEP;
  var gridVisible:Bool = true;
  var simulationActive:Bool=false;
  /** Samples per pixel the editor asked for, and what the renderer was last given. */
  var sampleCountRequested:Int = 1;
  var sampleCountApplied:Int = 0;
  var sampleCountEffective:Int = 1;
  var renderedSampleCount:Int = 0;
  var runtimeRevision:Int=0;
  var simulationPoses:Array<SimulationPoseVisual> = [];
  var robotVisuals:Array<SimulationRobotVisual> = [];
  final missionOverlay = new MissionOverlayView();
  var overlaysVisible:Bool = true;

  public function new(key:String, scene:EditorScene, host:UiHostContext, ?style:LayoutStyle) {
    this.key = key;
    this.scene = scene;
    this.host = host;
    this.style = style == null ? defaultStyle() : style.copy();
  }

  public function lightingPresetId():Int return lightingPreset;

  /** Asks for this many samples per pixel; the GPU may support fewer, and one turns anti-aliasing off. */
  public function setSampleCount(requested:Int):Void {
    if (requested < 1 || requested == sampleCountRequested) return;
    sampleCountRequested = requested;
  }

  /** Samples per pixel the last render used. */
  public function sampleCount():Int return sampleCountEffective;

  /** Whether this GPU can render the viewport with `samples` samples per pixel. */
  public function supportsSamples(samples:Int):Bool {
    if (samples <= 1) return true;
    ensureRenderer();
    var active = renderer;
    return active != null && active.maxSampleCount() >= samples;
  }

  /** Revision key for state that changes the retained viewport presentation. */
  public function presentationKey():String
    return scene.visualRevision + ":" + camera.revision + ":" + lightingRevision + ":" + sketchDragRevision + ":" +
      assemblyDragRevision + ":" + matePickRevision + ":" +
      hoverRevision + ":" + gridVisible + ":" + gridStep + ":" + sampleCountRequested;

  public function setLightingPreset(preset:Int):Void {
    if (preset < 0 || preset > 2 || preset == lightingPreset) return;
    lightingPreset = preset;
    lightingRevision++;
  }

  public function build(context:BuildContext):RenderNode {
    return context.withScope(new Key(key), function() {
      var node = new RenderNode(context.id("perspective"), LayoutVisualKind.Custom, style);
      node.hitTestSelf = true;
      node.focusable = true;
      node.semantics = new Semantics(AccessibilityRole.Image,
        "Scene perspective GPU view");
      node.onPaint(paint, "perspective:" + scene.visualRevision + ":" + runtimeRevision + ":" +
        sketchDragRevision + ":" + assemblyDragRevision + ":hover:" + hoverRevision + ":" + camera.revision +
        ":light:" + lightingRevision + ":aa:" + sampleCountRequested + ":" +
        renderedWidth + "x" + renderedHeight + ":grid:" + gridVisible + ":" + gridStep);
      installNavigation(node);
      return node;
    });
  }

  function paint(canvas:Canvas, geometry:ResolvedLayoutItem):Void {
    ensureRenderer();
    var width = Std.int(Math.max(1.0, Math.min(2048.0, Math.ceil(geometry.width))));
    var height = Std.int(Math.max(1.0, Math.min(2048.0, Math.ceil(geometry.height))));
    var displayRevision=scene.visualRevision+":"+runtimeRevision;
    if (renderer != null && (surface == null || renderedRevision != displayRevision ||
        renderedCameraRevision != camera.revision ||
        renderedLightingRevision != lightingRevision ||
        renderedSampleCount != sampleCountRequested ||
        renderedHoverRevision != hoverRevision ||
        width != renderedWidth || height != renderedHeight)) {
      var started = Sys.time();
      fitCameraClipRange();
      var view = scene.configureRenderView(new SceneView(), camera.viewProjection(width / height),
        simulationActive ? simulationPoses : null, hoveredObjectId, hoveredFaceIndex);
      view.setCameraViewPose(camera.eyePosition(), camera.viewDirection());
      if (gridVisible && gridStep > 0.0) {
        var scale = Math.tan(PerspectiveCamera.FOV_Y * Math.PI / 360.0);
        view.setWorkplaneGrid(camera.eyePosition(), camera.studioDirection(0.0, 0.0, -1.0),
          camera.studioDirection(scale * width / height, 0.0, 0.0),
          camera.studioDirection(0.0, scale, 0.0), gridStep, camera.distance);
      }
      var directions:Array<Float> = [];
      // Coin directions point from each light into the scene; the shader needs the reverse.
      var intensities = switch (lightingPreset) {
        case 1: [0.65, 0.45, 0.25];
        case 2: [0.95, 0.20, 0.65];
        default: [0.76, 0.34, 0.50];
      };
      var coinDirections = [[0.6841049, -0.12062616, -0.7193398],
        [-0.6403416, 0.7631294, 0.087155744],
        [-0.7544065, -0.63302225, -0.17364818]];
      for (index in 0...3) {
        var light = coinDirections[index];
        var direction = camera.studioDirection(-light[0], -light[1], -light[2]);
        directions.push(direction[0]); directions.push(direction[1]);
        directions.push(direction[2]); directions.push(intensities[index]);
      }
      var sky = switch (lightingPreset) {
        case 1: [0.25, 0.26, 0.27];
        case 2: [0.18, 0.19, 0.20];
        default: [0.22, 0.23, 0.24];
      };
      var ground = switch (lightingPreset) {
        case 1: [0.22, 0.22, 0.23];
        case 2: [0.08, 0.08, 0.09];
        default: [0.16, 0.16, 0.17];
      };
      view.setStudioLighting(directions, sky, ground);
      // Keep the scene image transparent so the UI gradient shows through
      // wherever the renderer has no geometry.
      if (sampleCountApplied != sampleCountRequested) {
        sampleCountEffective = renderer.setSampleCount(sampleCountRequested);
        sampleCountApplied = sampleCountRequested;
      }
      var changes = scene.takeRenderChanges();
      var rendered:GraphicsImageRef;
      try rendered = renderer.renderImage(scene.renderSnapshot(), view, width, height,
        0.0, 0.0, 0.0, 0.0, changes)
      catch (error:Dynamic) {
        if (changes != null) changes.dispose();
        throw error;
      }
      if (changes != null) changes.dispose();
      var next = GraphicsSurface.fromImage(rendered);
      rendered.dispose();
      if (surface != null) surface.dispose();
      surface = next;
      renderedRevision = displayRevision;
      renderedCameraRevision = camera.revision;
      renderedLightingRevision = lightingRevision;
      renderedSampleCount = sampleCountRequested;
      renderedHoverRevision = hoverRevision;
      renderedWidth = width;
      renderedHeight = height;
      lastRenderSeconds = Sys.time() - started;
      totalRenderSeconds += lastRenderSeconds;
      renderCount++;
    }
    canvas.fillLinearGradientRect(new Rect(0, 0, geometry.width, geometry.height),
      0.0, 0.0, 0.0, geometry.height,
      [new GradientStop(0.0, ViewportBackground.top()),
        new GradientStop(1.0, ViewportBackground.bottom())]);
    paintWorkplane(canvas, geometry.width, geometry.height);
    if (surface != null) canvas.drawSurface(surface, new Rect(0, 0, geometry.width, geometry.height));
    paintSensors(canvas,geometry.width,geometry.height);
    if(simulationActive)missionOverlay.paint(canvas,camera,geometry.width,geometry.height);
    paintAssemblyDrag(canvas, geometry.width, geometry.height);
    paintSketchDraft(canvas, geometry.width, geometry.height);
  }

  public function diagnosticState():Dynamic return {
    width: renderedWidth,
    height: renderedHeight,
    composition: "gpu-surface",
    renders: renderCount,
    sampleCount: sampleCountEffective,
    cpuTransferBytes: 0,
    lastRenderMilliseconds: lastRenderSeconds * 1000.0,
    averageRenderMilliseconds: renderCount == 0 ? 0.0 : totalRenderSeconds * 1000.0 / renderCount,
    camera: {targetX: camera.targetX, targetY: camera.targetY, targetZ: camera.targetZ,
      yaw: camera.yaw, pitch: camera.pitch, distance: camera.distance}
  };

  public function frameSelected():Void {
    var selected = scene.object(scene.selectedId);
    if (selected != null) {
      var transform = displayTransform(selected);
      var bounds=transformedBounds(transform,selected.width,selected.height,selected.depth);
      camera.frame((bounds[0]+bounds[3])/2,(bounds[1]+bounds[4])/2,(bounds[2]+bounds[5])/2,
        bounds[3]-bounds[0],bounds[4]-bounds[1],bounds[5]-bounds[2],aspect());
      return;
    }
    var items = scene.items();
    if (items.length == 0) { camera.reset(); return; }
    var minX = 1000000000.0, minY = 1000000000.0, minZ = 1000000000.0;
    var maxX = -1000000000.0, maxY = -1000000000.0, maxZ = -1000000000.0;
    for (item in items) {
      if (!item.visible) continue;
      var transform = displayTransform(item);
      var bounds=transformedBounds(transform,item.width,item.height,item.depth);
      minX=Math.min(minX,bounds[0]);minY=Math.min(minY,bounds[1]);minZ=Math.min(minZ,bounds[2]);
      maxX=Math.max(maxX,bounds[3]);maxY=Math.max(maxY,bounds[4]);maxZ=Math.max(maxZ,bounds[5]);
    }
    if (minX == 1000000000.0) { camera.reset(); return; }
    camera.frame((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2,
      maxX - minX, maxY - minY, maxZ - minZ, aspect());
  }

  public function resetView():Void camera.reset();

  public function setViewAngle(yaw:Float, pitch:Float):Void camera.setAngle(yaw, pitch);
  public function viewAngleLabel():String return camera.angleLabel();

  public function setPlacementOptions(snap:Bool, step:Float, ?visible:Bool = true):Void {
    if (gridStep != step || gridVisible != visible) renderedRevision = "";
    gridSnapEnabled = snap;
    gridStep = step;
    gridVisible = visible;
  }
  public function setSimulationState(active:Bool,poses:Array<SimulationPoseVisual>,revision:Int,
      ?robots:Array<SimulationRobotVisual>):Void {
    simulationActive=active;simulationPoses=poses==null?[]:poses.copy();runtimeRevision=revision;
    robotVisuals=robots==null?[]:robots.copy();
  }
  /** What the running mission shows on the floor: its route, costmap, sensed obstacles and odometry; null shows none. */
  /** Whether sensor rays (and, through `setMissionOverlay`, the mission's overlays) are drawn. */
  public function setSimulationOverlays(visible:Bool):Void overlaysVisible = visible;
  public function setMissionOverlay(overlay:Null<MissionOverlay>):Void missionOverlay.set(overlay);
  public function editingEnabled():Bool return !simulationActive;

  public function dragging():Bool return objectDrag != null || sketchRectangleDrag != null || assemblyDrag != null ||
    matePick != null || sketchPointDragging;

  /** Starts picking two faces for a mate: the next clicks on faces go to `tool` until it finishes or is cancelled. */
  public function beginMatePick(tool:MatePickTool):Void {
    matePick = tool;
    matePickRevision++;
    host.requestFrame();
  }

  /** The face mate being picked, or null. */
  public function activeMatePick():Null<MatePickTool> return matePick;

  /** What the mate tool asks for, or what it did; null when no mate is being picked. */
  public function matePickMessage():Null<String> return matePick == null ? null : matePick.message;

  /** Hands a clicked face to the mate tool; it ends once the mate is added. */
  function pickMateFace(sceneId:Null<String>, faceIndex:Int):Void {
    var tool = matePick;
    if (tool == null) return;
    tool.pick(sceneId, faceIndex);
    if (tool.finished) matePick = null;
    matePickRevision++;
    host.requestFrame();
  }

  /** While a jointed part is dragged, why it is or is not following the cursor; null otherwise. */
  public function assemblyDragMessage():Null<String> return assemblyDrag == null ? null : assemblyDrag.message();

  public function commitDrag():Null<PerspectivePointer> {
    if (sketchPointDragging) {
      endSketchPointDrag(true);
      return releaseNavigation();
    }
    if (objectDrag != null) {
      objectDrag.commit(); objectDrag = null;
    } else if (assemblyDrag != null) {
      assemblyDrag.commit(); assemblyDrag = null; assemblyDragRevision++;
    } else if (sketchRectangleDrag != null) {
      var active = sketchRectangleDrag;
      scene.addSketchDraftRectangleBetween(active.startX, active.startY, active.currentX, active.currentY);
      sketchRectangleDrag = null;
      sketchDragRevision++;
    } else return null;
    return releaseNavigation();
  }

  public function cancelDrag():Null<PerspectivePointer> {
    if (sketchPointDragging) {
      endSketchPointDrag(false);
      return releaseNavigation();
    }
    if (matePick != null && navigationPointer == null) {
      matePick = null;
      matePickRevision++;
      return null;
    }
    if (objectDrag != null) {
      objectDrag.cancel(); objectDrag = null;
    } else if (assemblyDrag != null) {
      assemblyDrag.cancel(); assemblyDrag = null; assemblyDragRevision++;
    } else if (sketchRectangleDrag != null) {
      sketchRectangleDrag = null;
      sketchDragRevision++;
    } else return null;
    return releaseNavigation();
  }

  /** Starts an IK drag when the press lands on a joint-driven part; the drag plane faces the camera. */
  function beginAssemblyDrag(localX:Float, localY:Float):Bool {
    fitCameraClipRange();
    var ray = camera.screenRay(localX, localY, Math.max(1, renderedWidth), Math.max(1, renderedHeight));
    var view = scene.configureRenderView(new SceneView(), camera.viewProjection(aspect()), null);
    var hit = scene.pickHitRayWithView(view, ray.originX, ray.originY, ray.originZ,
      ray.directionX, ray.directionY, ray.directionZ);
    if (hit.id == "scene" || !Math.isFinite(hit.x) || !Math.isFinite(hit.y) || !Math.isFinite(hit.z)) return false;
    var drag = scene.beginAssemblyDrag(hit.id, [hit.x, hit.y, hit.z]);
    if (drag == null) return false;
    var normal = camera.viewDirection();
    assemblyDragPlane = [hit.x, hit.y, hit.z, normal[0], normal[1], normal[2]];
    assemblyDrag = drag;
    assemblyDragRevision++;
    return true;
  }

  function updateAssemblyDrag(localX:Float, localY:Float):Void {
    var drag = assemblyDrag;
    if (drag == null) return;
    var ray = camera.screenRay(localX, localY, Math.max(1, renderedWidth), Math.max(1, renderedHeight));
    var p = assemblyDragPlane;
    var denominator = ray.directionX * p[3] + ray.directionY * p[4] + ray.directionZ * p[5];
    if (Math.abs(denominator) < 1e-9) return;
    var distance = ((p[0] - ray.originX) * p[3] + (p[1] - ray.originY) * p[4] + (p[2] - ray.originZ) * p[5]) / denominator;
    if (distance < 0.0) return;
    drag.update(ray.originX + ray.directionX * distance, ray.originY + ray.directionY * distance,
      ray.originZ + ray.directionZ * distance);
    assemblyDragRevision++;
    host.requestFrame();
  }

  /** The pull line from the grabbed point to the cursor: green while following, amber when it cannot. */
  function paintAssemblyDrag(canvas:Canvas, width:Float, height:Float):Void {
    var drag = assemblyDrag;
    if (drag == null) return;
    var from = drag.grabbedPoint(), to = drag.target();
    var color = drag.following() ? Color.rgba(0.25, 0.85, 0.45, 0.95) : Color.rgba(1.0, 0.62, 0.15, 0.95);
    var target = camera.project(to[0], to[1], to[2], width, height);
    if (target != null) canvas.fillRect(new Rect(target.x - 4, target.y - 4, 8, 8), color);
    var segment = camera.projectSegment(from[0], from[1], from[2], to[0], to[1], to[2], width, height);
    if (segment == null) return;
    var path = new PathBuilder();
    path.moveTo(segment[0].x, segment[0].y).lineTo(segment[1].x, segment[1].y);
    canvas.strokeTransient(path.build(), color, 2.0);
  }

  function releaseNavigation():Null<PerspectivePointer> {
    var pointer = navigationPointer;
    navigationPointer = null; navigationMode = 0;
    return pointer == null ? null : new PerspectivePointer(pointer, pointerX, pointerY);
  }

  public function pick(localX:Float, localY:Float):String {
    fitCameraClipRange();
    var ray=camera.screenRay(localX,localY,Math.max(1,renderedWidth),Math.max(1,renderedHeight));
    var view=scene.configureRenderView(new SceneView(),camera.viewProjection(aspect()),
      simulationActive?simulationPoses:null);
    return scene.pickRayWithView(view,ray.originX,ray.originY,ray.originZ,
      ray.directionX,ray.directionY,ray.directionZ,edgeAngularTolerance(localX,localY));
  }

  public function hoveredId():Null<String> return hoveredObjectId;

  public function hoveredFace():Int return hoveredFaceIndex;

  function updateHover(localX:Float, localY:Float):Void {
    fitCameraClipRange();
    var ray = camera.screenRay(localX, localY, Math.max(1, renderedWidth),
      Math.max(1, renderedHeight));
    // A running simulation poses the scene, so hover must test the posed geometry, as picking does.
    var hit = simulationActive
      ? scene.hoverHitRayWithView(scene.configureRenderView(new SceneView(),
          camera.viewProjection(aspect()), simulationPoses),
          ray.originX, ray.originY, ray.originZ, ray.directionX, ray.directionY, ray.directionZ)
      : scene.hoverHitRay(ray.originX, ray.originY, ray.originZ,
          ray.directionX, ray.directionY, ray.directionZ);
    var next:Null<String> = hit.id;
    var nextFace = hit.faceIndex;
    if (next == "scene") next = null;
    if (next == hoveredObjectId && nextFace == hoveredFaceIndex) return;
    hoveredObjectId = next;
    hoveredFaceIndex = next == null ? -1 : nextFace;
    hoverRevision++;
    host.requestFrame();
  }

  function clearHover():Void {
    if (hoveredObjectId == null && hoveredFaceIndex < 0) return;
    hoveredObjectId = null;
    hoveredFaceIndex = -1;
    hoverRevision++;
    host.requestFrame();
  }

  function edgeAngularTolerance(localX:Float,localY:Float):Float {
    var width=Math.max(1,renderedWidth),height=Math.max(1,renderedHeight);
    var center=camera.screenRay(localX,localY,width,height);
    var offset=camera.screenRay(localX+3.0,localY,width,height);
    var dot=center.directionX*offset.directionX+center.directionY*offset.directionY+
      center.directionZ*offset.directionZ;
    return Math.acos(Math.max(-1.0,Math.min(1.0,dot)));
  }

  function selectAt(localX:Float,localY:Float):String {
    if(!editingEnabled()){var id=pick(localX,localY);scene.select(id);return id;}
    fitCameraClipRange();
    var ray=camera.screenRay(localX,localY,Math.max(1,renderedWidth),Math.max(1,renderedHeight));
    var view=scene.configureRenderView(new SceneView(),camera.viewProjection(aspect()),null);
    return scene.selectRayWithView(view,ray.originX,ray.originY,ray.originZ,
      ray.directionX,ray.directionY,ray.directionZ,edgeAngularTolerance(localX,localY));
  }

  public static function pickScene(scene:EditorScene, camera:PerspectiveCamera,
      width:Float, height:Float, localX:Float, localY:Float):String {
    var ray = camera.screenRay(localX, localY, width, height);
    return scene.pickRay(ray.originX,ray.originY,ray.originZ,
      ray.directionX,ray.directionY,ray.directionZ);
  }

  static function transformedBounds(transform:Transform,width:Float,height:Float,depth:Float):Array<Float>{
    var result=[1e300,1e300,1e300,-1e300,-1e300,-1e300];
    for(x in [-width/2,width/2])for(y in [-height/2,height/2])for(z in [-depth/2,depth/2]){
      var local=[x,y,z];for(row in 0...3){var value=transform.element(12+row);
        for(column in 0...3)value+=transform.element(column*4+row)*local[column];
        result[row]=Math.min(result[row],value);result[row+3]=Math.max(result[row+3],value);}}
    return result;
  }

  function fitCameraClipRange():Void {
    var bounds:Array<Array<Float>> = [];
    for (item in scene.items()) {
      if (!item.visible) continue;
      if (simulationActive) {
        bounds.push(transformedBounds(displayTransform(item), item.width, item.height, item.depth));
      } else {
        var world = scene.info(item.id).bounds();
        if (world.valid)
          bounds.push([world.minX, world.minY, world.minZ,
            world.maxX, world.maxY, world.maxZ]);
      }
    }
    camera.fitClipRange(bounds);
  }

  function displayTransform(item:EditorSceneObject):Transform {
    if (simulationActive) for (pose in simulationPoses) if (pose.id == item.id)
      return poseTransform(pose.position,pose.rotation);
    return Transform.identity().translated(item.x,item.y,item.z);
  }

  static function poseTransform(position:Array<Float>,rotation:Array<Float>):Transform {
    var x=rotation[0],y=rotation[1],z=rotation[2],w=rotation[3];
    return Transform.identity()
      .set(0,1-2*(y*y+z*z)).set(1,2*(x*y+z*w)).set(2,2*(x*z-y*w))
      .set(4,2*(x*y-z*w)).set(5,1-2*(x*x+z*z)).set(6,2*(y*z+x*w))
      .set(8,2*(x*z+y*w)).set(9,2*(y*z-x*w)).set(10,1-2*(x*x+y*y))
      .translated(position[0],position[1],position[2]);
  }

  public function dispose():Void {
    if (surface != null) surface.dispose();
    surface = null;
    if (renderer != null) renderer.dispose();
    renderer = null;
  }

  function paintWorkplane(canvas:Canvas, width:Float, height:Float):Void {
    if (!gridVisible || gridStep <= 0.0) return;
    // The workplane is a UI overlay that extends beyond scene geometry. Its
    // projection must not inherit the scene's tightly fitted depth range.
    var projection = camera.viewProjection(width / Math.max(1.0, height),
      Math.max(0.001, camera.distance / 10000.0),
      Math.max(100.0, camera.distance * 100.0));
    var extent = visibleAxisExtent(width, height);
    var xAxis = new PathBuilder();
    if (appendWorldSegment(xAxis, projection, -extent, 0.0, 0.0, extent, 0.0, 0.0, width, height)) {
      canvas.strokeTransient(xAxis.build(), Color.rgba(0.72, 0.25, 0.22, 0.72), 1.5);
    }
    var yAxis = new PathBuilder();
    if (appendWorldSegment(yAxis, projection, 0.0, -extent, 0.0, 0.0, extent, 0.0, width, height)) {
      canvas.strokeTransient(yAxis.build(), Color.rgba(0.18, 0.52, 0.35, 0.72), 1.5);
    }
    var zAxis = new PathBuilder();
    if (appendWorldSegment(zAxis, projection, 0.0, 0.0, 0.0, 0.0, 0.0,
        Math.min(extent, Math.max(1.0, camera.distance)), width, height)) {
      canvas.strokeTransient(zAxis.build(), Color.rgba(0.22, 0.39, 0.78, 0.82), 1.5);
    }
  }

  function visibleAxisExtent(width:Float, height:Float):Float {
    var eye = camera.eyePosition();
    var extent = Math.max(gridStep * 8.0, camera.distance * 1.5);
    var hasIntersection = false, hasHorizon = false;
    for (screenX in [0.0, width]) for (screenY in [0.0, height]) {
      var ray = camera.screenRay(screenX, screenY, width, height);
      if (Math.abs(ray.directionZ) < 0.000001) { hasHorizon = true; continue; }
      var distance = -eye[2] / ray.directionZ;
      if (distance <= 0.0) { hasHorizon = true; continue; }
      hasIntersection = true;
      var x = eye[0] + ray.directionX * distance;
      var y = eye[1] + ray.directionY * distance;
      extent = Math.max(extent, Math.max(Math.abs(x - camera.targetX),
        Math.abs(y - camera.targetY)) * 1.2);
    }
    if (hasIntersection && hasHorizon)
      extent = Math.max(extent, camera.distance * 80.0);
    return Math.min(5000.0, extent);
  }

  function appendWorldSegment(path:PathBuilder, projection:Transform, x0:Float, y0:Float, z0:Float,
      x1:Float, y1:Float, z1:Float, width:Float, height:Float):Bool {
    var projected = camera.projectSegmentWithMatrix(projection,
      x0, y0, z0, x1, y1, z1, width, height);
    if (projected == null) return false;
    path.moveTo(projected[0].x, projected[0].y).lineTo(projected[1].x, projected[1].y);
    return true;
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
      var values=sensor.values.toArray(),visibleRays=0;
      for(index in 0...values.length){var angle=index*6.283185307179586/values.length;
        var direction=rotateVector(rotation,[Math.cos(angle),Math.sin(angle),0.0]);
        var hit=camera.project(origin[0]+direction[0]*values[index],origin[1]+direction[1]*values[index],
          origin[2]+direction[2]*values[index],width,height);
        if(hit!=null&&(hit.x!=mount.x||hit.y!=mount.y)){path.moveTo(mount.x,mount.y).lineTo(hit.x,hit.y);visibleRays++;}}
      // Every ray can project to nothing (behind the camera, clipped, or a range that is not finite yet) or have
      // no length (a zero range before the first scan); stroking such a path would hand the renderer nothing.
      if(visibleRays>0&&overlaysVisible)canvas.strokeTransient(path.build(),Color.rgba(0.25,0.8,1.0,0.22),1.0);
    }
    }
  }

  function paintSketchDraft(canvas:Canvas, width:Float, height:Float):Void {
    var sketch = scene.sketchDraftSnapshot();
    var plane = scene.sketchDraftPlane();
    if (sketch == null || plane == null)
      return;
    var solution = scene.sketchDraftSolution();
    var pointValues:Map<String, Array<Float>> = new Map();
    for (point in sketch.points()) {
      var value = [point.x, point.y];
      if (solution != null) {
        try value = solution.point(point.id) catch (_:Dynamic) {}
      }
      pointValues.set(point.id, value);
    }
    // Geometry the constraints still leave free is drawn apart (blue) from constrained geometry (amber).
    var fixedPath = new PathBuilder(), freePath = new PathBuilder();
    var hasLine = false, hasFree = false;
    for (entity in sketch.entities()) {
      var free = solution != null && solution.freeEntities.indexOf(entity.id) >= 0;
      var path = free ? freePath : fixedPath;
      var center = pointValues.get(entity.first);
      if (entity.kind == "line") {
        if (entity.second == null || center == null)
          continue;
        var end = pointValues.get(entity.second);
        if (end == null)
          continue;
        var a = projectSketchPoint(plane, center[0], center[1], width, height);
        var b = projectSketchPoint(plane, end[0], end[1], width, height);
        if (a == null || b == null)
          continue;
        path.moveTo(a.x, a.y).lineTo(b.x, b.y);
        if (free) hasFree = true; else hasLine = true;
      } else if ((entity.kind == "circle" || entity.kind == "arc") && center != null) {
        var radius = entity.radius;
        if (solution != null) {
          try radius = solution.radius(entity.id) catch (_:Dynamic) {}
        }
        if (!Math.isFinite(radius) || radius <= 0)
          continue;
        var startAngle = entity.kind == "circle" ? 0.0 : entity.startAngle;
        var sweep = entity.kind == "circle" ? Math.PI * 2 : entity.endAngle - entity.startAngle;
        if (entity.kind == "arc") {
          sweep %= Math.PI * 2;
          if (entity.clockwise) {
            while (sweep >= 0) sweep -= Math.PI * 2;
          } else {
            while (sweep <= 0) sweep += Math.PI * 2;
          }
        }
        var steps = 48;
        var started = false;
        var prior:Null<PerspectiveScreenPoint> = null;
        for (index in 0...(steps + 1)) {
          var angle = startAngle + sweep * index / steps;
          var point = projectSketchPoint(plane, center[0] + Math.cos(angle) * radius,
            center[1] + Math.sin(angle) * radius, width, height);
          if (point == null)
            continue;
          if (!started) {
            path.moveTo(point.x, point.y);
            started = true;
          } else if (prior != null) {
            path.lineTo(point.x, point.y);
          }
          prior = point;
        }
        if (started) {
          if (free) hasFree = true; else hasLine = true;
        }
      }
    }
    var path = fixedPath;
    var active = sketchRectangleDrag;
    if (active != null) {
      var minX = Math.min(active.startX, active.currentX);
      var maxX = Math.max(active.startX, active.currentX);
      var minY = Math.min(active.startY, active.currentY);
      var maxY = Math.max(active.startY, active.currentY);
      var corners = [[minX, minY], [maxX, minY], [maxX, maxY], [minX, maxY]];
      var first:Null<PerspectiveScreenPoint> = null;
      var prior:Null<PerspectiveScreenPoint> = null;
      for (corner in corners) {
        var projected = projectSketchPoint(plane, corner[0], corner[1], width, height);
        if (projected == null)
          continue;
        if (first == null) first = projected;
        if (prior != null) path.moveTo(prior.x, prior.y).lineTo(projected.x, projected.y);
        prior = projected;
      }
      if (first != null && prior != null) path.moveTo(prior.x, prior.y).lineTo(first.x, first.y);
      hasLine = true;
    }
    if (hasLine)
      canvas.strokeTransient(fixedPath.build(), Color.rgba(1.0, 0.82, 0.22, 0.98), 2.0);
    if (hasFree)
      canvas.strokeTransient(freePath.build(), FREE_SKETCH_COLOR, 2.0);
    for (id => point in pointValues) {
      var projected = projectSketchPoint(plane, point[0], point[1], width, height);
      var free = solution != null && solution.freePoints.indexOf(id) >= 0;
      if (projected != null)
        canvas.fillRect(new Rect(projected.x - 3, projected.y - 3, 6, 6),
          free ? FREE_SKETCH_COLOR : Color.rgba(1.0, 0.88, 0.42, 1.0));
    }
  }

  /** How close (screen pixels) a press must land to a sketch point to drag it. */
  static inline var SKETCH_GRAB_PIXELS:Float = 8.0;

  function endSketchPointDrag(keep:Bool):Void {
    sketchPointDragging = false;
    scene.endSketchDraftPointDrag(keep);
    sketchDragRevision++;
  }

  /** Sketch geometry its constraints still leave free to move. */
  static final FREE_SKETCH_COLOR = Color.rgba(0.36, 0.72, 1.0, 0.98);

  function projectSketchPoint(plane:Plane, x:Float, y:Float,
      width:Float, height:Float):Null<PerspectiveScreenPoint> {
    var world = plane.toWorld(new Vector(x, y, 0));
    return camera.project(world.x, world.y, world.z, width, height);
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
      updateHover(event.localX, event.localY);
      if (event.button == 0 && matePick != null) {
        pickMateFace(hoveredObjectId, hoveredFaceIndex);
        event.preventDefault(); event.stopPropagation();
        return;
      }
      if (event.button == 0 && editingEnabled() && scene.hasActiveSketchEdit()) {
        var point = sketchPlanePoint(event.localX, event.localY);
        if (point == null) return;
        // A press on a sketch point drags it (its constraints hold); elsewhere it draws a rectangle.
        var aside = sketchPlanePoint(event.localX + SKETCH_GRAB_PIXELS, event.localY);
        var radius = aside == null ? 0.0 : Math.sqrt((aside.x - point.x) * (aside.x - point.x) + (aside.y - point.y) * (aside.y - point.y));
        var grabbed = scene.sketchDraftPointNear(point.x, point.y, radius);
        if (grabbed != null && scene.beginSketchDraftPointDrag(grabbed)) {
          sketchPointDragging = true;
          sketchDragRevision++;
          navigationPointer = event.pointerId;
          navigationMode = 6;
          pointerX = event.x; pointerY = event.y;
          event.capturePointer(); event.preventDefault(); event.stopPropagation();
          return;
        }
        sketchRectangleDrag = new PerspectiveSketchRectangleDrag(point.x, point.y);
        sketchDragRevision++;
        navigationPointer = event.pointerId;
        navigationMode = 4;
        pointerX = event.x; pointerY = event.y;
        event.capturePointer(); event.preventDefault(); event.stopPropagation();
        return;
      }
      navigationPointer = event.pointerId;
      navigationMode = event.button == 0 ? 1 : 2;
      if (event.button == 0) {
        var hit = pick(event.localX, event.localY);
        if (hit != "scene"&&editingEnabled()) {
          selectAt(event.localX,event.localY);
          objectDrag = PerspectiveSceneDrag.begin(scene, camera, hit, event.localX, event.localY,
            Math.max(1, renderedWidth), Math.max(1, renderedHeight), gridSnapEnabled, gridStep);
          if (objectDrag != null) navigationMode = 3;
          else if (beginAssemblyDrag(event.localX, event.localY)) navigationMode = 5;
        }
      }
      pointerX = event.x; pointerY = event.y;
      pointerStartX = event.x; pointerStartY = event.y; pointerMoved = false;
      event.capturePointer(); event.preventDefault(); event.stopPropagation();
    });
    node.on(UiEventKind.PointerMove, function(event:UiEvent) {
      if (navigationPointer == null) {
        updateHover(event.localX, event.localY);
        return;
      }
      if (event.pointerId != navigationPointer) return;
      var deltaX = event.x - pointerX, deltaY = event.y - pointerY;
      pointerX = event.x; pointerY = event.y;
      if (Math.abs(event.x - pointerStartX) >= 3.0 || Math.abs(event.y - pointerStartY) >= 3.0)
        pointerMoved = true;
      var sketchDrag = sketchRectangleDrag;
      if (navigationMode == 1) camera.orbit(deltaX, deltaY);
      else if (navigationMode == 2) camera.pan(deltaX, deltaY, Math.max(1, renderedHeight));
      else if (objectDrag != null) objectDrag.update(camera, event.localX, event.localY,
        Math.max(1, renderedWidth), Math.max(1, renderedHeight));
      else if (navigationMode == 5 && assemblyDrag != null) updateAssemblyDrag(event.localX, event.localY);
      else if (navigationMode == 6 && sketchPointDragging) {
        var point = sketchPlanePoint(event.localX, event.localY);
        if (point != null && scene.dragSketchDraftPoint(point.x, point.y)) {
          sketchDragRevision++;
          host.requestFrame();
        }
      }
      else if (navigationMode == 4 && sketchDrag != null) {
        var point = sketchPlanePoint(event.localX, event.localY);
        if (point != null) {
          if (sketchDrag.currentX != point.x || sketchDrag.currentY != point.y) {
            sketchDrag.currentX = point.x;
            sketchDrag.currentY = point.y;
            sketchDragRevision++;
          }
        }
      }
      if (navigationMode != 4) updateHover(event.localX, event.localY);
      event.preventDefault(); event.stopPropagation();
    });
    node.on(UiEventKind.HoverEnter, function(event:UiEvent) {
      if (navigationPointer == null) updateHover(event.localX, event.localY);
    });
    node.on(UiEventKind.HoverLeave, function(_) {
      if (navigationPointer == null) clearHover();
    });
    var finish = function(event:UiEvent) {
      if (navigationPointer == null || event.pointerId != navigationPointer) return;
      var sketchDrag = sketchRectangleDrag;
      if (navigationMode == 3 && objectDrag != null) {
        if (event.kind == UiEventKind.PointerUp) objectDrag.commit(); else objectDrag.cancel();
        objectDrag = null;
      } else if (navigationMode == 6 && sketchPointDragging) {
        endSketchPointDrag(event.kind == UiEventKind.PointerUp);
      } else if (navigationMode == 5 && assemblyDrag != null) {
        if (event.kind == UiEventKind.PointerUp) assemblyDrag.commit(); else assemblyDrag.cancel();
        assemblyDrag = null;
        assemblyDragRevision++;
      } else if (navigationMode == 4 && sketchDrag != null) {
        if (event.kind == UiEventKind.PointerUp) {
          var point = sketchPlanePoint(event.localX, event.localY);
          if (point != null) {
            sketchDrag.currentX = point.x;
            sketchDrag.currentY = point.y;
          }
          scene.addSketchDraftRectangleBetween(sketchDrag.startX, sketchDrag.startY,
            sketchDrag.currentX, sketchDrag.currentY);
        }
        sketchRectangleDrag = null;
        sketchDragRevision++;
      } else if (event.kind == UiEventKind.PointerUp && navigationMode == 1 && !pointerMoved)
        selectAt(event.localX,event.localY);
      navigationPointer = null; navigationMode = 0;
      if (event.kind == UiEventKind.PointerUp) updateHover(event.localX, event.localY);
      else clearHover();
      event.releasePointer(); event.preventDefault(); event.stopPropagation();
    };
    node.on(UiEventKind.PointerUp, finish);
    node.on(UiEventKind.PointerCancel, finish);
    node.on(UiEventKind.Scroll, function(event:UiEvent) {
      camera.zoom(event.deltaY);
      event.preventDefault(); event.stopPropagation();
    });
    node.on(UiEventKind.KeyDown, function(event:UiEvent) {
      if (event.key == UiKey.Escape && sketchRectangleDrag != null) {
        sketchRectangleDrag = null;
        sketchDragRevision++;
        navigationPointer = null; navigationMode = 0;
        event.releasePointer(); event.preventDefault(); event.stopPropagation();
        return;
      }
      if (event.key == UiKey.Escape && sketchPointDragging) {
        endSketchPointDrag(false);
        navigationPointer = null; navigationMode = 0;
        event.releasePointer(); event.preventDefault(); event.stopPropagation();
        host.requestFrame();
        return;
      }
      if (event.key == UiKey.Escape && matePick != null && navigationPointer == null) {
        matePick = null;
        matePickRevision++;
        event.preventDefault(); event.stopPropagation();
        host.requestFrame();
        return;
      }
      if (event.key == UiKey.Escape && assemblyDrag != null) {
        assemblyDrag.cancel(); assemblyDrag = null; assemblyDragRevision++;
        navigationPointer = null; navigationMode = 0;
        event.releasePointer(); event.preventDefault(); event.stopPropagation();
        host.requestFrame();
        return;
      }
      if (event.key != UiKey.Escape || objectDrag == null) return;
      objectDrag.cancel(); objectDrag = null;
      navigationPointer = null; navigationMode = 0;
      event.releasePointer(); event.preventDefault(); event.stopPropagation();
    });
  }

  function sketchPlanePoint(localX:Float, localY:Float):Null<Vector> {
    var plane = scene.sketchDraftPlane();
    if (plane == null)
      return null;
    var ray = camera.screenRay(localX, localY, Math.max(1, renderedWidth), Math.max(1, renderedHeight));
    var denominator = plane.normal.x * ray.directionX + plane.normal.y * ray.directionY +
      plane.normal.z * ray.directionZ;
    if (Math.abs(denominator) < 0.000001)
      return null;
    var distance = (plane.origin.x - ray.originX) * plane.normal.x +
      (plane.origin.y - ray.originY) * plane.normal.y + (plane.origin.z - ray.originZ) * plane.normal.z;
    distance /= denominator;
    if (distance < 0)
      return null;
    var world = new Vector(ray.originX + ray.directionX * distance,
      ray.originY + ray.directionY * distance, ray.originZ + ray.directionZ * distance);
    var local = plane.toLocal(world);
    if (gridSnapEnabled)
      local = new Vector(Math.round(local.x / gridStep) * gridStep,
        Math.round(local.y / gridStep) * gridStep, 0);
    return local;
  }

  static function defaultStyle():LayoutStyle {
    var result = new LayoutStyle();
    result.width = LayoutAxis.stretch();
    result.height = LayoutAxis.stretch();
    result.clipToParent = true;
    return result;
  }
}

private class PerspectiveSketchRectangleDrag {
  public final startX:Float;
  public final startY:Float;
  public var currentX:Float;
  public var currentY:Float;
  public function new(x:Float, y:Float) {
    startX = currentX = x;
    startY = currentY = y;
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
