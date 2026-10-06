package motionkit.kinematics;

/** Planned process speed on each outgoing sample span, and physical drive
 * acceleration limits. A fixed process feed is a shape constraint; an air
 * feed is a cap and may be lowered by its physical time law. */
class PathDriveLimits {
  public final acceleration:Array<Float>;
  public final feed:Array<Float>;
  public final feedGradient:Array<Float>;
  public final fixedFeed:Array<Bool>;
  public function new(acceleration:Array<Float>,feed:Array<Float>,?feedGradient:Array<Float>,?fixedFeed:Array<Bool>) {
    if(acceleration==null || acceleration.length==0 || feed==null || feed.length<2 ||
        feedGradient!=null && feedGradient.length!=feed.length || fixedFeed!=null && fixedFeed.length!=feed.length)throw "Drive refinement needs aligned physical limits and feeds";
    for(value in acceleration)if(!Math.isFinite(value) || value<=0)throw "Drive acceleration limits must be finite and positive";
    for(value in feed)if(!Math.isFinite(value) || value<=0)throw "Planned refinement feed must be finite and positive";
    this.acceleration=acceleration.copy();this.feed=feed.copy();
    this.fixedFeed=fixedFeed==null ? [for(_ in feed)true] : fixedFeed.copy();
    this.feedGradient=feedGradient==null ? [for(_ in feed)0.0] : feedGradient.copy();
    for(value in this.feedGradient)if(!Math.isFinite(value) || value<0)throw "Feed gradient bounds must be finite and nonnegative";
  }
}
