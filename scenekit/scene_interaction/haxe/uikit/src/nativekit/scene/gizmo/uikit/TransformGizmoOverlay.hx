package nativekit.scene.gizmo.uikit;
import nativekit.scene.gizmo.*;
import nativekit.scene.gizmo.TranslationHandle;
import haxeon.ui.*;
import haxeon.ui.Paint.SolidPaint;

/** Optional UIKit presentation of the pure gizmos. No documents, World, assets or history dependencies. */
class TransformGizmoOverlay {
  public var move(default,null):Null<TranslationGizmo>;
  public var rotate(default,null):Null<RotationGizmo>;
  public var scale(default,null):Null<ScaleGizmo>;
  public var tool(default,null):GizmoTool=Move;
  public var hover:Null<String>;
  var frameKey:Null<String>;
  var canvasWidth:Float=0;
  var canvasHeight:Float=0;
  final fonts:FontCollection;
  final labels:Map<String,TextLayout> = new Map();
  var labelPaint:Null<SolidPaint>;
  public function new(fonts:FontCollection) {if(fonts==null) throw "Gizmo overlay requires font services";this.fonts=fonts;}
  public function valid(key:String):Bool return frameKey!=null && frameKey==key;
  public function hide():Void {move=null;rotate=null;scale=null;frameKey=null;hover=null;}
  public function invalidateSize(width:Float,height:Float):Void {
    if(Math.abs(width-canvasWidth)>0.5 || Math.abs(height-canvasHeight)>0.5) hide();
  }
  public function layout(key:String,tool:GizmoTool,camera:GizmoCamera,pivot:GizmoVec3,basis:GizmoBasis,localBasis:GizmoBasis):Void {
    if(key==null || key.length==0 || tool==null || camera==null || pivot==null || basis==null || localBasis==null) throw "Invalid gizmo overlay layout";
    var nextMove:Null<TranslationGizmo> = null,nextRotate:Null<RotationGizmo> = null,nextScale:Null<ScaleGizmo> = null;
    switch tool {case Move:nextMove=new TranslationGizmo(camera,pivot,90,8,basis);case Rotate:nextRotate=new RotationGizmo(camera,pivot,75,basis);case Scale:nextScale=new ScaleGizmo(camera,pivot,localBasis);}
    var previousHover=frameKey==key ? hover : null;
    hide();move=nextMove;rotate=nextRotate;scale=nextScale;hover=previousHover;frameKey=key;this.tool=tool;canvasWidth=camera.width;canvasHeight=camera.height;
  }
  public function pick(x:Float,y:Float):Null<String> return switch tool {
    case Move:var layout=move,handle=layout==null ? null : layout.pick(x,y);handle==null ? null : TranslationGizmo.label(handle);
    case Rotate:var layout=rotate,handle=layout==null ? null : layout.pick(x,y);handle==null ? null : TranslationGizmo.label(handle);
    case Scale:var layout=scale,handle=layout==null ? null : layout.pick(x,y);handle==null ? null : ScaleGizmo.label(handle);
  };
  public function shapes():Array<OverlayShape> {
    var result:Array<OverlayShape> = [];
    switch tool {
      case Move:var layout=move;if(layout!=null) for(shape in layout.shapes()) result.push({handle:shape.label,points:shape.points()});
      case Rotate:var layout=rotate;if(layout!=null) for(shape in layout.shapes()) result.push({handle:shape.label,points:shape.points()});
      case Scale:var layout=scale;if(layout!=null) for(shape in layout.shapes()) result.push({handle:shape.label,points:shape.points()});
    }
    return result;
  }
  public function paint(canvas:Canvas,following:Bool,displacement:GizmoVec3):Void {
    if(tool==Scale){paintScale(canvas);return;}if(tool==Rotate){paintRotation(canvas);return;}
    var layout = move; if (layout == null) return;
    var shapes = following ? layout.translatedShapes(displacement) : layout.shapes();
    canvas.save(); canvas.clip(new Rect(0,0,canvasWidth,canvasHeight));
    for (shape in shapes) {
      var points = shape.points(), hot = shape.label == hover;
      var color = hot ? Color.rgba(1,0.85,0.2,1) : switch shape.handle {
        case AxisX, PlaneYZ: Color.rgba(1,0.3,0.25,1);
        case AxisY, PlaneXZ: Color.rgba(0.35,0.95,0.4,1);
        case AxisZ, PlaneXY: Color.rgba(0.3,0.6,1,1);
      };
      var path = new PathBuilder().moveTo(points[0].x,points[0].y);
      for (index in 1...points.length) path.lineTo(points[index].x,points[index].y);
      if (points.length == 4) {
        path.close(); canvas.fillTransient(path.build(),Color.rgba(hot?1:0.5,hot?0.85:0.65,hot?0.2:0.8,0.35));
        path = new PathBuilder().moveTo(points[0].x,points[0].y);
        for (index in 1...points.length) path.lineTo(points[index].x,points[index].y); path.close();
      }
      canvas.strokeTransient(path.build(),color,hot?4:2);
      if (points.length == 2) {
        var a = points[0], b = points[1], dx = b.x-a.x, dy = b.y-a.y, length = Math.sqrt(dx*dx+dy*dy);
        dx /= length; dy /= length;
        canvas.fillTransient(new PathBuilder().moveTo(b.x,b.y).lineTo(b.x-8*dx-4*dy,b.y-8*dy+4*dx)
          .lineTo(b.x-8*dx+4*dy,b.y-8*dy-4*dx).close().build(),color);
        if (labelPaint == null) labelPaint = SolidPaint.create(Color.rgba(1,1,1,1));
        var label = labels.get(shape.label);
        if (label == null) { label = TextLayout.create(fonts,shape.label,24,new TextStyle(13)); labels.set(shape.label,label); }
        canvas.setPaint(labelPaint); canvas.drawText(label,b.x+5,b.y-8);
      }
    }
    canvas.restore();
  }
  function paintRotation(canvas:Canvas):Void {
    var rings=rotate;if(rings==null) return;
    canvas.save();canvas.clip(new Rect(0,0,canvasWidth,canvasHeight));
    for(shape in rings.shapes()) {
      var color=shape.label==hover ? Color.rgba(1,0.85,0.2,1) : switch shape.handle {case AxisX:Color.rgba(1,0.3,0.25,1);case AxisY:Color.rgba(0.35,0.95,0.4,1);default:Color.rgba(0.3,0.6,1,1);};
      var points=shape.points();
      var path=new PathBuilder();path.moveTo(points[0].x,points[0].y);for(i in 1...points.length) path.lineTo(points[i].x,points[i].y);
      canvas.strokeTransient(path.build(),color,shape.label==hover ? 3 : 2);
    }
    canvas.restore();
  }
  function paintScale(canvas:Canvas):Void {
    var layout=scale;if(layout==null) return;
    canvas.save();canvas.clip(new Rect(0,0,canvasWidth,canvasHeight));
    for(shape in layout.shapes()) {
      var hot=shape.label==hover,color=hot ? Color.rgba(1,0.85,0.2,1) : switch shape.handle {case ScaleX:Color.rgba(1,0.3,0.25,1);case ScaleY:Color.rgba(0.35,0.95,0.4,1);case ScaleZ:Color.rgba(0.3,0.6,1,1);case Uniform:Color.rgba(0.95,0.95,0.95,1);};
      var points=shape.points(),path=new PathBuilder().moveTo(points[0].x,points[0].y);
      for(i in 1...points.length) path.lineTo(points[i].x,points[i].y);
      if(points.length==4){path.close();canvas.fillTransient(path.build(),color);}else {
        canvas.strokeTransient(path.build(),color,hot ? 4 : 2);var tip=points[1];canvas.fillRect(new Rect(tip.x-4,tip.y-4,8,8),color);
      }
    }
    canvas.restore();
  }
  public function dispose():Void {hide();for(label in labels) label.dispose();labels.clear();if(labelPaint!=null) labelPaint.dispose();labelPaint=null;}
}

/** Detached projected points for accessibility/debug consumers, never authored data. */
typedef OverlayShape = {final handle:String;final points:Array<GizmoPoint>;}
