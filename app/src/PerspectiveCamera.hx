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

  function eyePosition():Array<Float> {
    var cosPitch = Math.cos(pitch);
    return [targetX + distance * cosPitch * Math.cos(yaw),
      targetY + distance * cosPitch * Math.sin(yaw), targetZ + distance * Math.sin(pitch)];
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
