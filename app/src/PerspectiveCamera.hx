package app;

import nativekit.scene.Transform;

/** Testable orbit camera state shared by perspective input and rendering. */
class PerspectiveCamera {
  public static inline var FOV_Y:Float = 50.0;
  public var targetX(default, null):Float;
  public var targetY(default, null):Float;
  public var targetZ(default, null):Float;
  public var yaw(default, null):Float;
  public var pitch(default, null):Float;
  public var distance(default, null):Float;
  public var revision(default, null):Int = 0;

  public function new() reset();

  public function reset():Void {
    targetX = targetY = targetZ = 0.0;
    distance = 9.354143466934854;
    yaw = -0.9652516631899266;
    pitch = 0.5639426413606289;
    revision++;
  }

  public function orbit(deltaX:Float, deltaY:Float):Void {
    yaw -= deltaX * 0.008;
    pitch = clamp(pitch + deltaY * 0.008, -1.48, 1.48);
    revision++;
  }

  public function pan(deltaX:Float, deltaY:Float, viewportHeight:Float):Void {
    var scale = 2.0 * distance * Math.tan(FOV_Y * Math.PI / 360.0) /
      Math.max(1.0, viewportHeight);
    var rightX = -Math.sin(yaw), rightY = Math.cos(yaw);
    var forwardX = -Math.cos(pitch) * Math.cos(yaw);
    var forwardY = -Math.cos(pitch) * Math.sin(yaw);
    var forwardZ = -Math.sin(pitch);
    var upX = rightY * forwardZ;
    var upY = -rightX * forwardZ;
    var upZ = rightX * forwardY - rightY * forwardX;
    targetX -= (rightX * deltaX + upX * deltaY) * scale;
    targetY -= (rightY * deltaX + upY * deltaY) * scale;
    targetZ -= upZ * deltaY * scale;
    revision++;
  }

  public function zoom(deltaY:Float):Void {
    distance = clamp(distance * Math.pow(2.0, deltaY * 0.002), 0.05, 1000000.0);
    revision++;
  }

  public function frame(centerX:Float, centerY:Float, centerZ:Float,
      width:Float, height:Float, depth:Float, aspect:Float):Void {
    targetX = centerX; targetY = centerY; targetZ = centerZ;
    var vertical = Math.max(height, depth);
    var horizontal = Math.max(width, depth) / Math.max(0.01, aspect);
    var diameter = Math.max(0.1, Math.max(vertical, horizontal));
    distance = clamp((diameter * 0.65) / Math.tan(FOV_Y * Math.PI / 360.0), 0.05, 1000000.0);
    revision++;
  }

  public function viewProjection(aspect:Float):Transform {
    var eye = eyePosition();
    var forward = normalize([targetX - eye[0], targetY - eye[1], targetZ - eye[2]]);
    var side = normalize(cross(forward, [0.0, 0.0, 1.0]));
    var up = cross(side, forward);
    var view = [side[0], up[0], -forward[0], 0.0,
      side[1], up[1], -forward[1], 0.0,
      side[2], up[2], -forward[2], 0.0,
      -dot(side, eye), -dot(up, eye), dot(forward, eye), 1.0];
    var near = Math.max(0.001, distance / 10000.0), far = Math.max(100.0, distance * 100.0);
    var scale = 1.0 / Math.tan(FOV_Y * Math.PI / 360.0);
    var projection = [scale / Math.max(0.01, aspect), 0.0, 0.0, 0.0,
      0.0, scale, 0.0, 0.0,
      0.0, 0.0, (far + near) / (near - far), -1.0,
      0.0, 0.0, 2.0 * far * near / (near - far), 0.0];
    var combined:Array<Float> = [];
    for (column in 0...4) for (row in 0...4) {
      var value = 0.0;
      for (index in 0...4) value += projection[index * 4 + row] * view[column * 4 + index];
      combined.push(value);
    }
    var result = Transform.identity();
    for (index in 0...16) result.set(index, combined[index]);
    return result;
  }

  /** Rotate a camera-space direction toward a light into world coordinates. */
  public function studioDirection(x:Float, y:Float, z:Float):Array<Float> {
    var eye = eyePosition();
    var forward = normalize([targetX - eye[0], targetY - eye[1], targetZ - eye[2]]);
    var right = normalize(cross(forward, [0.0, 0.0, 1.0]));
    var up = cross(right, forward);
    return [right[0] * x + up[0] * y - forward[0] * z,
      right[1] * x + up[1] * y - forward[1] * z,
      right[2] * x + up[2] * y - forward[2] * z];
  }

  /** Builds a world-space camera ray through a viewport pixel. */
  public function screenRay(x:Float, y:Float, width:Float, height:Float):PerspectiveRay {
    width = Math.max(1.0, width); height = Math.max(1.0, height);
    var eye = eyePosition();
    var forward = normalize([targetX - eye[0], targetY - eye[1], targetZ - eye[2]]);
    var right = normalize(cross(forward, [0.0, 0.0, 1.0]));
    var up = cross(right, forward);
    var scale = Math.tan(FOV_Y * Math.PI / 360.0);
    var screenX = (2.0 * x / width - 1.0) * scale * width / height;
    var screenY = (1.0 - 2.0 * y / height) * scale;
    var direction = normalize([forward[0] + right[0] * screenX + up[0] * screenY,
      forward[1] + right[1] * screenX + up[1] * screenY,
      forward[2] + right[2] * screenX + up[2] * screenY]);
    return new PerspectiveRay(eye[0], eye[1], eye[2],
      direction[0], direction[1], direction[2]);
  }

  public function project(x:Float,y:Float,z:Float,width:Float,height:Float):Null<PerspectiveScreenPoint>{
    var matrix=viewProjection(width/Math.max(1.0,height)),source=[x,y,z,1.0],clip:Array<Float> = [];
    for(row in 0...4){var value=0.0;for(column in 0...4)value+=matrix.element(column*4+row)*source[column];clip.push(value);}
    if(clip[3]<=0.000001)return null;
    return screenPoint(clip, width, height);
  }

  /** Projects the visible part of a world-space line after clipping it to the camera frustum. */
  public function projectSegment(x0:Float, y0:Float, z0:Float, x1:Float, y1:Float, z1:Float,
      width:Float, height:Float):Null<Array<PerspectiveScreenPoint>> {
    width = Math.max(1.0, width);
    height = Math.max(1.0, height);
    return projectSegmentWithMatrix(viewProjection(width / height),
      x0, y0, z0, x1, y1, z1, width, height);
  }

  /** Uses one frame's projection for multiple lines in the same viewport. */
  public function projectSegmentWithMatrix(matrix:Transform,
      x0:Float, y0:Float, z0:Float, x1:Float, y1:Float, z1:Float,
      width:Float, height:Float):Null<Array<PerspectiveScreenPoint>> {
    width = Math.max(1.0, width);
    height = Math.max(1.0, height);
    var start = clipPoint(matrix, x0, y0, z0);
    var end = clipPoint(matrix, x1, y1, z1);
    var first = 0.0, last = 1.0;
    for (plane in 0...6) {
      var startDistance = clipPlaneDistance(start, plane);
      var endDistance = clipPlaneDistance(end, plane);
      if (startDistance < 0.0 && endDistance < 0.0) return null;
      if (startDistance < 0.0 || endDistance < 0.0) {
        var crossing = startDistance / (startDistance - endDistance);
        if (startDistance < 0.0) first = Math.max(first, crossing);
        else last = Math.min(last, crossing);
      }
    }
    if (first > last) return null;
    var clippedStart = interpolateClip(start, end, first);
    var clippedEnd = interpolateClip(start, end, last);
    return [screenPoint(clippedStart, width, height), screenPoint(clippedEnd, width, height)];
  }

  public function intersectPlaneZ(x:Float, y:Float, width:Float, height:Float,
      planeZ:Float):Null<PerspectivePlanePoint> {
    var ray = screenRay(x, y, width, height);
    if (Math.abs(ray.directionZ) < 0.000001) return null;
    var distance = (planeZ - ray.originZ) / ray.directionZ;
    if (distance < 0.0) return null;
    return new PerspectivePlanePoint(ray.originX + ray.directionX * distance,
      ray.originY + ray.directionY * distance, planeZ);
  }

  function eyePosition():Array<Float> {
    var cosPitch = Math.cos(pitch);
    return [targetX + distance * cosPitch * Math.cos(yaw),
      targetY + distance * cosPitch * Math.sin(yaw), targetZ + distance * Math.sin(pitch)];
  }

  static function clipPoint(matrix:Transform, x:Float, y:Float, z:Float):Array<Float> {
    var source = [x, y, z, 1.0], clip:Array<Float> = [];
    for (row in 0...4) {
      var value = 0.0;
      for (column in 0...4) value += matrix.element(column * 4 + row) * source[column];
      clip.push(value);
    }
    return clip;
  }

  static inline function clipPlaneDistance(point:Array<Float>, plane:Int):Float
    return switch plane {
      case 0: point[3] + point[0];
      case 1: point[3] - point[0];
      case 2: point[3] + point[1];
      case 3: point[3] - point[1];
      case 4: point[3] + point[2];
      default: point[3] - point[2];
    }

  static function interpolateClip(start:Array<Float>, end:Array<Float>, amount:Float):Array<Float> {
    return [for (index in 0...4) start[index] + (end[index] - start[index]) * amount];
  }

  static function screenPoint(clip:Array<Float>, width:Float, height:Float):PerspectiveScreenPoint {
    return new PerspectiveScreenPoint((clip[0] / clip[3] * 0.5 + 0.5) * width,
      (0.5 - clip[1] / clip[3] * 0.5) * height, clip[2] / clip[3]);
  }

  static inline function clamp(value:Float, low:Float, high:Float):Float
    return Math.max(low, Math.min(high, value));
  static function normalize(value:Array<Float>):Array<Float> {
    var length = Math.pow(dot(value, value), 0.5);
    return [value[0] / length, value[1] / length, value[2] / length];
  }
  static function dot(left:Array<Float>, right:Array<Float>):Float
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
  static function cross(left:Array<Float>, right:Array<Float>):Array<Float>
    return [left[1] * right[2] - left[2] * right[1],
      left[2] * right[0] - left[0] * right[2], left[0] * right[1] - left[1] * right[0]];
}

class PerspectiveRay {
  public final originX:Float;
  public final originY:Float;
  public final originZ:Float;
  public final directionX:Float;
  public final directionY:Float;
  public final directionZ:Float;
  public function new(originX:Float, originY:Float, originZ:Float,
      directionX:Float, directionY:Float, directionZ:Float) {
    this.originX = originX; this.originY = originY; this.originZ = originZ;
    this.directionX = directionX; this.directionY = directionY; this.directionZ = directionZ;
  }
}

class PerspectivePlanePoint {
  public final x:Float;
  public final y:Float;
  public final z:Float;
  public function new(x:Float, y:Float, z:Float) {
    this.x = x; this.y = y; this.z = z;
  }
}
class PerspectiveScreenPoint {
  public final x:Float;public final y:Float;public final depth:Float;
  public function new(x:Float,y:Float,depth:Float){this.x=x;this.y=y;this.depth=depth;}
}
