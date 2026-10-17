package stockkit.hobbing;

/** Opt-in per-gash history, separate from the bounded online force observer.
    Reconstructs the same overlapping rectangular window-mean radial loads.
    This is not instantaneous tooth force. Times seconds, forces N. */
class HobbingForceHistory {
  public final samples:Array<HobbingForceSample> = [];
  var times:Array<Float> = [];
  var radial:Array<Float> = [];
  var prepared=false;
  public function new() {}
  public function add(sample:HobbingForceSample):Void {samples.push(sample);prepared=false;}
  function prepare():Void {
    if(prepared)return;
    var events:Array<{time:Float,delta:Float}> = [];
    for(s in samples){events.push({time:s.pass.start,delta:s.radialForce});events.push({time:s.pass.end,delta:-s.radialForce});}
    events.sort((a,b)->a.time<b.time?-1:a.time>b.time?1:0);
    times=[];radial=[];var i=0,force=0.0;
    while(i<events.length){var t=events[i].time;
      while(i<events.length&&events[i].time==t){force+=events[i].delta;i++;}
      times.push(t);radial.push(Math.max(0,force));
    }
    prepared=true;
  }
  /** Right-continuous radial window load for a later wall-source sweep. */
  public function radialAt(time:Float):Float {
    prepare();if(!Math.isFinite(time))throw "non-finite force lookup time";
    if(times.length==0||time>=times[times.length-1])return 0;
    var lo=0,hi=times.length;
    while(lo<hi){var mid=(lo+hi)>>1;if(times[mid]<=time)lo=mid+1;else hi=mid;}
    return lo==0?0:radial[lo-1];
  }
  public function statistics(start:Float,end:Float):Dynamic {
    if(!Math.isFinite(start)||!Math.isFinite(end)||start>=end)throw "invalid force history interval";
    prepare();var integral=0.0,square=0.0,activeTime=0.0,min=1e300,max=0.0;
    var t=start,f=radialAt(start);
    for(i in 0...times.length){if(times[i]<=start)continue;if(times[i]>=end)break;
      var dt=times[i]-t;integral+=f*dt;square+=f*f*dt;if(f>1e-9)activeTime+=dt;
      min=Math.min(min,f);max=Math.max(max,f);t=times[i];f=radial[i];}
    var dt=end-t;integral+=f*dt;square+=f*f*dt;if(f>1e-9)activeTime+=dt;
    min=Math.min(min,f);max=Math.max(max,f);
    var mean=integral/(end-start),stepSquare=0.0,maxStep=0.0,passMin=1e300,passMax=0.0,previous=0.0,count=0;
    for(s in samples)if(s.pass.start>=start&&s.pass.start<end){
      if(count>0){var delta=s.radialForce-previous;maxStep=Math.max(maxStep,Math.abs(delta));stepSquare+=delta*delta;}
      previous=s.radialForce;passMin=Math.min(passMin,previous);passMax=Math.max(passMax,previous);count++;
    }
    return {radialMeanN:mean,radialMinN:min,radialPeakN:max,radialRangeN:max-min,
      radialStdDevN:Math.sqrt(Math.max(0,square/(end-start)-mean*mean)),
      radialPeakDeviationFromMeanN:Math.max(max-mean,mean-min),activeSeconds:activeTime,
      activeMeanRadialN:activeTime>0?integral/activeTime:0,
      perPassRadialMinN:count>0?passMin:0,perPassRadialMaxN:passMax,
      perPassRadialMaxStepN:maxStep,perPassRadialRmsStepN:count>1?Math.sqrt(stepSquare/(count-1)):0,
      windows:count,startSeconds:start,endSeconds:end,
      basis:"radial force; time statistics of overlapping rectangular gash-window means, including air; per-pass steps use successive individual gash means; no instantaneous tooth peaks"};
  }
  public function writeCsv(path:String):Void {
    var out=new StringBuf();out.add("start_s,end_s,gash,pass,radial_N,tangential_N,removed_mm3\n");
    for(s in samples)out.add('${s.pass.start},${s.pass.end},${s.pass.gash},${s.pass.pass},${s.radialForce},${s.tangentialForce},${s.removedVolume}\n');
    sys.io.File.saveContent(path,out.toString());
  }
}
