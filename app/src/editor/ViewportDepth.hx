package app.editor;

import haxe.io.Bytes;

/**
 * One captured depth image of a perspective viewport and the projection that
 * produced it. Window depth is the depth-tested pick pass's value per pixel
 * (0 near .. 1 far; 1 is background), rows from the top down.
 */
class ViewportDepth {
  public final width:Int;
  public final height:Int;
  public final near:Float;
  public final far:Float;
  public final fovY:Float;
  /** View depth the near plane was sized for (surface under the view centre, else the orbit distance). */
  public final focusDepth:Float;
  public final depthBits:Int;
  final window:Bytes;

  public function new(width:Int, height:Int, window:Bytes, near:Float, far:Float, fovY:Float, focusDepth:Float,
      depthBits:Int) {
    if (window.length != width * height * 4) throw "Depth capture size does not match its image";
    this.width = width;
    this.height = height;
    this.window = window;
    this.near = near;
    this.far = far;
    this.fovY = fovY;
    this.focusDepth = focusDepth;
    this.depthBits = depthBits;
  }

  /** The raw window depths, float32 per pixel, rows from the top down. */
  public function windowBytes():Bytes return window;

  public function windowDepth(x:Int, y:Int):Float return window.getFloat((y * width + x) * 4);

  public function covered(x:Int, y:Int):Bool return windowDepth(x, y) < 1.0;

  /** Distance along the view axis; inverts the GL projection (NDC z = 2 * window - 1). */
  public function viewDepth(x:Int, y:Int):Float {
    var ndc = windowDepth(x, y) * 2.0 - 1.0;
    return 2.0 * far * near / ((far + near) - ndc * (far - near));
  }

  /** World size of one depth-buffer step at view depth `depth`. */
  public function depthStepAt(depth:Float):Float
    return depth * depth * (far - near) / (far * near) / Math.pow(2.0, depthBits);

  /** World size of one pixel at view depth `depth`. */
  public function pixelSizeAt(depth:Float):Float
    return 2.0 * depth * Math.tan(fovY * Math.PI / 360.0) / height;

  /** Depth-buffer step at this pixel's surface, in that pixel's own footprints. */
  public function stepPixels(x:Int, y:Int):Float {
    var depth = viewDepth(x, y);
    return depthStepAt(depth) / pixelSizeAt(depth);
  }

  /** Coverage and step-size statistics over pixels showing geometry. */
  public function summary(limitPixels:Float):ViewportDepthSummary {
    var steps:Array<Float> = [], nearest = Math.POSITIVE_INFINITY, farthest = 0.0, over = 0;
    for (y in 0...height) for (x in 0...width) if (covered(x, y)) {
      var depth = viewDepth(x, y), step = stepPixels(x, y);
      nearest = Math.min(nearest, depth);
      farthest = Math.max(farthest, depth);
      steps.push(step);
      if (step > limitPixels) over++;
    }
    steps.sort((a, b) -> a < b ? -1 : a > b ? 1 : 0);
    function percentile(p:Float):Float return steps.length == 0 ? 0.0 : steps[Std.int(Math.min(steps.length - 1, p * steps.length))];
    var focus = focusDepth > 0.0 ? depthStepAt(focusDepth) / pixelSizeAt(focusDepth) : 0.0;
    var cx = Std.int(width / 2), cy = Std.int(height / 2);
    return {
      width: width,
      height: height,
      depthBits: depthBits,
      near: near,
      far: far,
      fovY: fovY,
      focusDepth: focusDepth,
      focusStepPixels: focus,
      centreDepth: covered(cx, cy) ? viewDepth(cx, cy) : 0.0,
      coveredPixels: steps.length,
      nearestSurface: steps.length == 0 ? 0.0 : nearest,
      farthestSurface: farthest,
      medianStepPixels: percentile(0.5),
      p95StepPixels: percentile(0.95),
      maxStepPixels: steps.length == 0 ? 0.0 : steps[steps.length - 1],
      limitStepPixels: limitPixels,
      pixelsOverLimit: over
    };
  }
}

typedef ViewportDepthSummary = {
  var width:Int;
  var height:Int;
  var depthBits:Int;
  var near:Float;
  var far:Float;
  var fovY:Float;
  var focusDepth:Float;
  /** Depth step at the focus, in pixel footprints there. */
  var focusStepPixels:Float;
  /** View depth captured at the centre pixel; zero when it shows background. Compare with focusDepth. */
  var centreDepth:Float;
  var coveredPixels:Int;
  var nearestSurface:Float;
  var farthestSurface:Float;
  var medianStepPixels:Float;
  var p95StepPixels:Float;
  var maxStepPixels:Float;
  var limitStepPixels:Float;
  var pixelsOverLimit:Int;
}
