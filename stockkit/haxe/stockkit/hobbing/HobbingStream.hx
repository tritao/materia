package stockkit.hobbing;

import stockkit.RingStock;

/** Bounded measured-motion window. Finishes each complete gash window exactly once,
    retaining the bracketing controller samples until all overlapping windows are cut. */
class HobbingStream {
  public final stock:RingStock;
  public final sweep:HobbingSweep;
  public final halfAngle:Float;
  public var passes(default,null):Int=0;
  public var poses(default,null):Int=0;
  public var peakFrames(default,null):Int=0;
  public var lastTime(default,null):Float=0;
  final frames:Array<HobbingKeyframe> = [];
  public var onPass:Null<(GashPass,HobbingAxes,HobbingAxes)->Void> = null;
  var next:Int=0;
  var first:Int=0;
  var initialized=false;
  var finished=false;
  public function new(stock:RingStock,sweep:HobbingSweep,halfAngle:Float) {
    if(!Math.isFinite(halfAngle)||halfAngle<=0||halfAngle>=Math.PI)throw "Invalid streaming gash window";
    this.stock=stock;this.sweep=sweep;this.halfAngle=halfAngle;
  }
  public function push(time:Float,axes:HobbingAxes):Void {
    if(finished)throw "Hobbing stream is finished";
    if(frames.length>0){var last=frames[frames.length-1];
      if(time<=last.time)throw "Hobbing sample clock must increase";
      if(axes.b<last.axes.b-1e-8)throw "Hob master reversed during the cut";
      if(axes.b<=last.axes.b+1e-12){
        // A stopped spindle has no phase crossings. Retain its latest pose before it starts.
        if(!initialized){frames[frames.length-1]=new HobbingKeyframe(time,axes);lastTime=time;return;}
        return;
      }
    }
    frames.push(new HobbingKeyframe(time,axes));lastTime=time;
    peakFrames=Std.int(Math.max(peakFrames,frames.length));
    if(frames.length>20000)throw "Hobbing measured window exceeds allocation budget";
    if(frames.length<2)return;
    var step=2*Math.PI/sweep.hob.gashes;
    if(!initialized){first=Std.int(Math.floor((frames[0].axes.b-halfAngle)/step));next=first;initialized=true;}
    drain(false);
  }
  public function finish():Void {if(finished)return;if(frames.length>=2&&initialized)drain(true);finished=true;}
  function drain(finalWindow:Bool):Void {
    var step=2*Math.PI/sweep.hob.gashes,last=frames[frames.length-1],motion=new HobbingMotion(frames);
    var endK=finalWindow?Std.int(Math.ceil((last.axes.b+halfAngle)/step)):Std.int(Math.floor((last.axes.b-halfAngle)/step));
    while(next<=endK){
      var phase=next*step;
      var start=Math.max(frames[0].time,motion.timeAtMaster(phase-halfAngle));
      var end=Math.min(last.time,motion.timeAtMaster(phase+halfAngle));
      if(start<end){var gash=(-next)%sweep.hob.gashes;if(gash<0)gash+=sweep.hob.gashes;
        var pass=new GashPass(gash,next-first,start,motion.timeAtMaster(phase),end);
        poses+=sweep.cutPass(stock,motion.at,pass,sweep.tolerance/16,0.001,motion.times());passes++;
        if(onPass!=null)onPass(pass,motion.at(start),motion.at(end));}
      next++;
    }
    var keep=next*step-halfAngle;
    while(frames.length>2&&frames[1].axes.b<keep)frames.shift();
  }
}
