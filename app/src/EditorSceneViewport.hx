package app;

import Canvas;
import Color;
import Rect;
import PathBuilder;
import nativekit.ui.core.ViewportContent;
import nativekit.ui.core.ViewportCamera;

/** XY orthographic presentation of the editor's planar SceneKit meshes. */
class EditorSceneViewport implements ViewportContent {
  public static inline var SCALE:Float = 100.0;
  public static inline var ORIGIN_X:Float = 320.0;
  public static inline var ORIGIN_Y:Float = 260.0;
  public static inline var GRID_STEP:Float = 0.2;
  final scene:EditorScene;
  var drag:Null<EditorSceneDrag> = null;
  public var gridStep(default, null):Float = GRID_STEP;
  public var viewportWidth:Float = 640.0;
  public var viewportHeight:Float = 520.0;
  var runtimeRevision:Int=0;
  var simulationActive:Bool=false;
  var simulationPoses:Map<String, SimulationPoseVisual> = new Map();

  public function new(scene:EditorScene) this.scene = scene;
  public function width():Float return 960.0;
  public function height():Float return 640.0;
  public function revision():Int return scene.revision+runtimeRevision*1000003;
  public function setRuntimeRevision(value:Int):Void runtimeRevision=value;
  public function setSimulationState(active:Bool,poses:Array<SimulationPoseVisual>,revision:Int):Void {
    simulationActive=active;runtimeRevision=revision;simulationPoses.clear();
    if(active)for(pose in poses)simulationPoses.set(pose.id,pose);
  }
  public function editingEnabled():Bool return !simulationActive;

  public function pick(camera:ViewportCamera, x:Float, y:Float):String {
    var point = scenePoint(camera, x, y);
    if(!simulationActive)return scene.pick(point.x,point.y);
    var ordered=scene.items();ordered.sort(function(a,b)return maximumZ(a)<maximumZ(b)?-1:1);
    var result="scene";
    for(item in ordered)if(scene.info(item.id).visible()&&pointInPolygon(point.x,point.y,corners(item)))result=item.id;
    return result;
  }

  public function dragging():Bool return drag != null;

  public function setGridStep(value:Float):Bool {
    if (value != value || value - value != 0.0 || value <= 0.0 || value > 1000000.0)
      throw "Grid spacing must be finite and positive";
    if (value == gridStep) return false;
    gridStep = value;
    return true;
  }

  /** Begins a primary-button move while retaining the pointer-to-origin offset. */
  public function beginDrag(camera:ViewportCamera, x:Float, y:Float, snap:Bool):Bool {
    if(simulationActive)return false;
    if (drag != null) cancelDrag();
    var point = scenePoint(camera, x, y);
    var id = scene.pick(point.x, point.y);
    if (scene.object(id) == null) return false;
    scene.select(id);
    var transform = scene.info(id).localTransform();
    drag = new EditorSceneDrag(id, transform.element(12), transform.element(13),
      transform.element(12) - point.x, transform.element(13) - point.y, snap);
    return true;
  }

  public function updateDrag(camera:ViewportCamera, x:Float, y:Float, constrain:Bool = false):Bool {
    var active = drag;
    if (active == null) return false;
    var point = scenePoint(camera, x, y);
    var nextX = point.x + active.offsetX;
    var nextY = point.y + active.offsetY;
    if (constrain) {
      if (active.constraintAxis < 0)
        active.constraintAxis = Math.abs(nextX - active.startX) >= Math.abs(nextY - active.startY) ? 0 : 1;
      if (active.constraintAxis == 0) nextY = active.startY;
      else nextX = active.startX;
    }
    if (active.snap) {
      nextX = Math.round(nextX / gridStep) * gridStep;
      nextY = Math.round(nextY / gridStep) * gridStep;
    }
    nextX = Math.max(-1000000.0, Math.min(1000000.0, nextX));
    nextY = Math.max(-1000000.0, Math.min(1000000.0, nextY));
    if (nextX == active.currentX && nextY == active.currentY) return false;
    scene.setPositionXY(active.id, nextX, nextY);
    active.currentX = nextX;
    active.currentY = nextY;
    return true;
  }

  public function commitDrag():Bool {
    var active = drag;
    if (active == null) return false;
    drag = null;
    return scene.recordMove(active.id, active.startX, active.startY,
      active.currentX, active.currentY);
  }

  public function cancelDrag():Bool {
    var active = drag;
    if (active == null) return false;
    drag = null;
    if (active.currentX != active.startX || active.currentY != active.startY)
      scene.setPositionXY(active.id, active.startX, active.startY);
    return true;
  }

  function scenePoint(camera:ViewportCamera, x:Float, y:Float):Point {
    var point = camera.viewportToWorld(x, y);
    return new Point((point.x - ORIGIN_X) / SCALE, (ORIGIN_Y - point.y) / SCALE);
  }

  public function frameSelected(camera:ViewportCamera):Void {
    var item = scene.object(scene.selectedId);
    if (item == null) return;
    var points=corners(item),minX=points[0][0],maxX=minX,minY=points[0][1],maxY=minY;
    for(point in points){minX=Math.min(minX,point[0]);maxX=Math.max(maxX,point[0]);
      minY=Math.min(minY,point[1]);maxY=Math.max(maxY,point[1]);}
    camera.fit((maxX-minX)*SCALE,(maxY-minY)*SCALE,viewportWidth,viewportHeight,48.0);
    camera.setPan(ORIGIN_X+(minX+maxX)/2*SCALE-viewportWidth/camera.zoom/2,
      ORIGIN_Y-(minY+maxY)/2*SCALE-viewportHeight/camera.zoom/2);
  }

  public function paint(canvas:Canvas, destination:Rect):Void {
    var ordered = scene.items();
    ordered.sort(function(a, b) {
      var first = scene.info(a.id).worldTransform().element(14);
      var second = scene.info(b.id).worldTransform().element(14);
      return first < second ? -1 : first > second ? 1 : 0;
    });
    for (item in ordered) {
      var state = scene.info(item.id);
      if (!state.visible()) continue;
      var points=corners(item),path=new PathBuilder();
      for(index in 0...points.length){var point=displayPoint(points[index][0],points[index][1]);
        if(index==0)path.moveTo(point.x,point.y);else path.lineTo(point.x,point.y);}
      var shape=path.close().build();
      canvas.fillTransient(shape,Color.rgba(item.red,item.green,item.blue,1.0));
      if (scene.selectedId == item.id) {
        canvas.strokeTransient(shape, Color.rgba(1.0, 0.88, 0.35, 1.0), 3.0);
      }
    }
  }

  function displayedPosition(id:String):Array<Float>{
    var pose=simulationActive?simulationPoses.get(id):null;
    if(pose!=null)return pose.position;
    var transform=scene.info(id).worldTransform();return [transform.element(12),transform.element(13),transform.element(14)];
  }
  function displayedRotation(id:String):Array<Float>{
    var pose=simulationActive?simulationPoses.get(id):null;return pose==null?[0.0,0.0,0.0,1.0]:pose.rotation;
  }
  function corners(item:EditorSceneObject):Array<Array<Float>>{
    var position=displayedPosition(item.id),q=displayedRotation(item.id),result:Array<Array<Float>> = [];
    for(x in [-item.width/2,item.width/2])for(y in [-item.height/2,item.height/2])
      for(z in [-item.depth/2,item.depth/2]){var v=rotate(q,[x,y,z]);result.push([position[0]+v[0],position[1]+v[1],position[2]+v[2]]);}
    return convexHull(result);
  }
  function displayPoint(x:Float,y:Float):Point return new Point(ORIGIN_X+x*SCALE,ORIGIN_Y-y*SCALE);
  function maximumZ(item:EditorSceneObject):Float {var p=displayedPosition(item.id),q=displayedRotation(item.id),result=-1e300;
    for(x in [-item.width/2,item.width/2])for(y in [-item.height/2,item.height/2])for(z in [-item.depth/2,item.depth/2])
      result=Math.max(result,p[2]+rotate(q,[x,y,z])[2]);return result;}
  static function rotate(q:Array<Float>,v:Array<Float>):Array<Float>{
    var x=q[0],y=q[1],z=q[2],w=q[3],tx=2*(y*v[2]-z*v[1]),ty=2*(z*v[0]-x*v[2]),tz=2*(x*v[1]-y*v[0]);
    return [v[0]+w*tx+y*tz-z*ty,v[1]+w*ty+z*tx-x*tz,v[2]+w*tz+x*ty-y*tx];
  }
  static function convexHull(points:Array<Array<Float>>):Array<Array<Float>>{
    points.sort(function(a,b)return a[0]<b[0]?-1:a[0]>b[0]?1:a[1]<b[1]?-1:a[1]>b[1]?1:0);
    var hull:Array<Array<Float>> = [];
    for(point in points){while(hull.length>=2&&cross(hull[hull.length-2],hull[hull.length-1],point)<=0)hull.pop();hull.push(point);}
    var lower=hull.length;
    for(index in 1...points.length){var point=points[points.length-1-index];
      while(hull.length>lower&&cross(hull[hull.length-2],hull[hull.length-1],point)<=0)hull.pop();hull.push(point);}
    if(hull.length>1)hull.pop();return hull;
  }
  static function cross(a:Array<Float>,b:Array<Float>,c:Array<Float>):Float
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
  static function pointInPolygon(x:Float,y:Float,points:Array<Array<Float>>):Bool{
    var inside=false,j=points.length-1;
    for(i in 0...points.length){if(((points[i][1]>y)!=(points[j][1]>y))&&
      x<(points[j][0]-points[i][0])*(y-points[i][1])/(points[j][1]-points[i][1])+points[i][0])inside=!inside;j=i;}
    return inside;
  }
}

private class EditorSceneDrag {
  public final id:String;
  public final startX:Float;
  public final startY:Float;
  public final offsetX:Float;
  public final offsetY:Float;
  public final snap:Bool;
  public var currentX:Float;
  public var currentY:Float;
  public var constraintAxis:Int = -1;
  public function new(id:String, startX:Float, startY:Float, offsetX:Float, offsetY:Float, snap:Bool) {
    this.id = id; this.startX = startX; this.startY = startY;
    this.offsetX = offsetX; this.offsetY = offsetY; this.snap = snap;
    currentX = startX; currentY = startY;
  }
}
