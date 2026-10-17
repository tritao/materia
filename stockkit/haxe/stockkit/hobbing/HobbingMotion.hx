package stockkit.hobbing;

/** Linear interpolation of measured/controller axis samples, without coupling C
    rigidly to B. Gash events can therefore use measured master phase while C keeps
    its independent actual following error. */
class HobbingMotion {
  final frames:Array<HobbingKeyframe>;
  public function new(frames:Array<HobbingKeyframe>) {
    if(frames.length<2)throw "hobbing motion needs at least two samples";
    this.frames=frames.copy();
    for(i in 1...frames.length)if(frames[i].time<=frames[i-1].time)throw "controller sample times must increase";
  }
  public function samples():Array<HobbingKeyframe> return frames.copy();
  public function times():Array<Float> return [for(f in frames)f.time];
  public function at(time:Float):HobbingAxes {
    if(!Math.isFinite(time)||time<frames[0].time||time>frames[frames.length-1].time)throw "sample time outside hobbing motion";
    var i=0,j=frames.length-1;
    while(j-i>1){var middle=(i+j)>>1;if(frames[middle].time<=time)i=middle;else j=middle;}
    var a=frames[i],b=frames[j],t=(time-a.time)/(b.time-a.time);
    function mix(x:Float,y:Float):Float return x+(y-x)*t;
    var offset:Null<HobbingPoseOffset> = null;
    if(a.axes.offset!=null||b.axes.offset!=null){
      var oa=a.axes.offset==null?new HobbingPoseOffset():a.axes.offset;
      var ob=b.axes.offset==null?new HobbingPoseOffset():b.axes.offset;
      offset=new HobbingPoseOffset();offset.workX=mix(oa.workX,ob.workX);offset.workY=mix(oa.workY,ob.workY);offset.workZ=mix(oa.workZ,ob.workZ);
      offset.toolX=mix(oa.toolX,ob.toolX);offset.toolY=mix(oa.toolY,ob.toolY);offset.toolZ=mix(oa.toolZ,ob.toolZ);
    }
    return new HobbingAxes(mix(a.axes.x,b.axes.x),mix(a.axes.y,b.axes.y),mix(a.axes.z,b.axes.z),
      mix(a.axes.a,b.axes.a),mix(a.axes.b,b.axes.b),mix(a.axes.c,b.axes.c),offset);
  }
  /** Monotone measured master; extrapolation only locates the centres of partial
      engagement windows outside the run, never poses outside sampled time. */
  public function timeAtMaster(phase:Float):Float {
    var i=0;
    for(k in 1...frames.length)if(frames[k].axes.b<=frames[k-1].axes.b)throw "master must increase for gash scheduling";
    while(i+1<frames.length-1&&frames[i+1].axes.b<phase)i++;
    var a=frames[i],b=frames[i+1];
    return a.time+(phase-a.axes.b)*(b.time-a.time)/(b.axes.b-a.axes.b);
  }
}
