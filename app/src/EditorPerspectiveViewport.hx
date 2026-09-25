package app;

import Canvas;
import Color;
import GraphicsSurface;
import GradientStop;
import LayoutAxis;
import LayoutStyle;
import LayoutVisualKind;
import Rect;
import PathBuilder;
import ResolvedLayoutItem;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
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
  var renderedLightingRevision:Int = -1;
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
  var objectDrag:Null<PerspectiveSceneDrag> = null;
  var sketchRectangleDrag:Null<PerspectiveSketchRectangleDrag> = null;
  var gridSnapEnabled:Bool = false;
  var gridStep:Float = EditorSceneViewport.GRID_STEP;
  var gridVisible:Bool = true;
  var simulationActive:Bool=false;
  var runtimeRevision:Int=0;
  var simulationPoses:Array<SimulationPoseVisual> = [];
  var robotVisuals:Array<SimulationRobotVisual> = [];

  public function new(key:String, scene:EditorScene, host:DesktopUiHostContext, ?style:LayoutStyle) {
    this.key = key;
    this.scene = scene;
    this.host = host;
    this.style = style == null ? defaultStyle() : style.copy();
  }

  public function lightingPresetId():Int return lightingPreset;

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
      node.onPaint(paint, "perspective:" + scene.revision + ":" + runtimeRevision + ":" +
        sketchDragRevision + ":" + camera.revision + ":light:" + lightingRevision + ":" +
        renderedWidth + "x" + renderedHeight + ":grid:" + gridVisible + ":" + gridStep);
      installNavigation(node);
      return node;
    });
  }

  function paint(canvas:Canvas, geometry:ResolvedLayoutItem):Void {
    ensureRenderer();
    var width = Std.int(Math.max(1.0, Math.min(2048.0, Math.ceil(geometry.width))));
    var height = Std.int(Math.max(1.0, Math.min(2048.0, Math.ceil(geometry.height))));
    var displayRevision=scene.revision+runtimeRevision*1000003;
    if (renderer != null && (surface == null || renderedRevision != displayRevision ||
        renderedCameraRevision != camera.revision ||
        renderedLightingRevision != lightingRevision ||
        width != renderedWidth || height != renderedHeight)) {
      var started = Sys.time();
      var view = scene.configureRenderView(new SceneView(), camera.viewProjection(width / height),
        simulationActive ? simulationPoses : null);
      view.setCameraViewPose(camera.eyePosition(), camera.viewDirection());
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
      var rendered = renderer.renderImage(scene.renderSnapshot(), view, width, height,
        0.0, 0.0, 0.0, 0.0);
      var next = GraphicsSurface.fromImage(rendered);
      rendered.dispose();
      if (surface != null) surface.dispose();
      surface = next;
      renderedRevision = displayRevision;
      renderedCameraRevision = camera.revision;
      renderedLightingRevision = lightingRevision;
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
    paintSketchDraft(canvas, geometry.width, geometry.height);
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

  public function setPlacementOptions(snap:Bool, step:Float, ?visible:Bool = true):Void {
    gridSnapEnabled = snap;
    gridStep = step;
    gridVisible = visible;
  }
  public function setSimulationState(active:Bool,poses:Array<SimulationPoseVisual>,revision:Int,
      ?robots:Array<SimulationRobotVisual>):Void {
    simulationActive=active;simulationPoses=poses==null?[]:poses.copy();runtimeRevision=revision;
    robotVisuals=robots==null?[]:robots.copy();
  }
  public function editingEnabled():Bool return !simulationActive;

  public function dragging():Bool return objectDrag != null || sketchRectangleDrag != null;

  public function commitDrag():Null<PerspectivePointer> {
    if (objectDrag != null) {
      objectDrag.commit(); objectDrag = null;
    } else if (sketchRectangleDrag != null) {
      var active = sketchRectangleDrag;
      scene.addSketchDraftRectangleBetween(active.startX, active.startY, active.currentX, active.currentY);
      sketchRectangleDrag = null;
      sketchDragRevision++;
    } else return null;
    return releaseNavigation();
  }

  public function cancelDrag():Null<PerspectivePointer> {
    if (objectDrag != null) {
      objectDrag.cancel(); objectDrag = null;
    } else if (sketchRectangleDrag != null) {
      sketchRectangleDrag = null;
      sketchDragRevision++;
    } else return null;
    return releaseNavigation();
  }

  function releaseNavigation():Null<PerspectivePointer> {
    var pointer = navigationPointer;
    navigationPointer = null; navigationMode = 0;
    return pointer == null ? null : new PerspectivePointer(pointer, pointerX, pointerY);
  }

  public function pick(localX:Float, localY:Float):String {
    var ray=camera.screenRay(localX,localY,Math.max(1,renderedWidth),Math.max(1,renderedHeight));
    var view=scene.configureRenderView(new SceneView(),camera.viewProjection(aspect()),
      simulationActive?simulationPoses:null);
    return scene.pickRayWithView(view,ray.originX,ray.originY,ray.originZ,
      ray.directionX,ray.directionY,ray.directionZ);
  }

  function selectAt(localX:Float,localY:Float):String {
    if(!editingEnabled()){var id=pick(localX,localY);scene.select(id);return id;}
    var ray=camera.screenRay(localX,localY,Math.max(1,renderedWidth),Math.max(1,renderedHeight));
    var view=scene.configureRenderView(new SceneView(),camera.viewProjection(aspect()),null);
    return scene.selectRayWithView(view,ray.originX,ray.originY,ray.originZ,
      ray.directionX,ray.directionY,ray.directionZ);
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
    var projection = camera.viewProjection(width / Math.max(1.0, height));
    var extent = Math.max(gridStep * 8.0, Math.min(500.0, camera.distance * 1.5));
    var spacing = gridStep;
    while (extent / spacing > 48.0) spacing *= 2.0;
    var minX = Math.floor((camera.targetX - extent) / spacing) * spacing;
    var maxX = Math.ceil((camera.targetX + extent) / spacing) * spacing;
    var minY = Math.floor((camera.targetY - extent) / spacing) * spacing;
    var maxY = Math.ceil((camera.targetY + extent) / spacing) * spacing;
    var grid = new PathBuilder();
    var x = minX;
    while (x <= maxX) {
      appendWorldSegment(grid, projection, x, minY, 0.0, x, maxY, 0.0, width, height);
      x += spacing;
    }
    var y = minY;
    while (y <= maxY) {
      appendWorldSegment(grid, projection, minX, y, 0.0, maxX, y, 0.0, width, height);
      y += spacing;
    }
    canvas.strokeTransient(grid.build(), Color.rgba(0.15, 0.22, 0.28, 0.22), 1.0);

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
    var path = new PathBuilder();
    var hasLine = false;
    for (entity in sketch.entities()) {
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
        hasLine = true;
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
        hasLine = hasLine || started;
      }
    }
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
      canvas.strokeTransient(path.build(), Color.rgba(1.0, 0.82, 0.22, 0.98), 2.0);
    for (point in pointValues) {
      var projected = projectSketchPoint(plane, point[0], point[1], width, height);
      if (projected != null)
        canvas.fillRect(new Rect(projected.x - 3, projected.y - 3, 6, 6),
          Color.rgba(1.0, 0.88, 0.42, 1.0));
    }
  }

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
      if (event.button == 0 && editingEnabled() && scene.hasActiveSketchEdit()) {
        var point = sketchPlanePoint(event.localX, event.localY);
        if (point == null) return;
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
      var sketchDrag = sketchRectangleDrag;
      if (navigationMode == 1) camera.orbit(deltaX, deltaY);
      else if (navigationMode == 2) camera.pan(deltaX, deltaY, Math.max(1, renderedHeight));
      else if (objectDrag != null) objectDrag.update(camera, event.localX, event.localY,
        Math.max(1, renderedWidth), Math.max(1, renderedHeight));
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
      event.preventDefault(); event.stopPropagation();
    });
    var finish = function(event:UiEvent) {
      if (navigationPointer == null || event.pointerId != navigationPointer) return;
      var sketchDrag = sketchRectangleDrag;
      if (navigationMode == 3 && objectDrag != null) {
        if (event.kind == UiEventKind.PointerUp) objectDrag.commit(); else objectDrag.cancel();
        objectDrag = null;
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
