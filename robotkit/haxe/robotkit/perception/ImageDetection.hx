package robotkit.perception;

/** One immutable axis-aligned box in source-image pixels. */
class ImageDetection {
  public final label:String;
  public final score:Float;
  public final x:Float;
  public final y:Float;
  public final width:Float;
  public final height:Float;

  public function new(label:String, score:Float, x:Float, y:Float, width:Float, height:Float) {
    if (label == null || label.length == 0 || !Math.isFinite(score) || score < 0 || score > 1 ||
        !Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(width) || !Math.isFinite(height) ||
        x < 0 || y < 0 || width <= 0 || height <= 0)
      throw "ImageDetection needs a label, score in [0,1], and a positive pixel box";
    this.label = label; this.score = score;
    this.x = x; this.y = y; this.width = width; this.height = height;
  }
}
