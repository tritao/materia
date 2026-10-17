import gearkit.*;
import stockkit.*;
import stockkit.hobbing.*;

typedef ForceSpeedResult={summary:Dynamic,stockHash:String,loadHash:String,seconds:Float,stockValues:Array<Float>,stockSources:Array<Int>,loadValues:Array<Float>};
/** Reference geometry, short virgin engagement and a common-precut mid-face band.
    No coarse result is used for grading. Seed stock is identical for every trial. */
class ForceSpeedChecks {
  static final fine=0.00005;
  static function signature(stock:RingStock):{hash:String,values:Array<Float>,sources:Array<Int>} {
    var text=new StringBuf(),values:Array<Float> = [],sources:Array<Int> = [];
    for(z in 0...stock.grid.axialCount)for(r in 0...stock.grid.radialCount){
      var intervals=stock.circle(r,z);sources.push(intervals.length);
      for(i in intervals){text.add('${z}/${r}:${i.lo},${i.hi},${i.loGash},${i.loPass},${i.hiGash},${i.hiPass};');
        values.push(i.lo);values.push(i.hi);for(id in [i.loGash,i.loPass,i.hiGash,i.hiPass])sources.push(id);}}
    return {hash:haxe.crypto.Sha256.encode(text.toString()),values:values,sources:sources};
  }
  static function cut(spec:GearSpec,hob:HobDesign,motion:IdealHobbingJob,passes:Array<GashPass>,tol:Float,threads:Int,?seed:RingSnapshot):ForceSpeedResult {
    var stock=RingStock.fromGearBlank(spec,0.6,0.05,1);stock.setThreads(threads);if(seed!=null)stock.restore(seed);
    var sweep=new HobbingSweep(hob,-10,10,tol),loads=new HobbingForces(stock,spec.pitchRadius(),spec.pressureAngle,1);
    var history=new HobbingForceHistory(),started=Sys.time();
    try {
      for(p in passes){sweep.cutPass(stock,motion.axesAt,p,tol/16);history.add(loads.afterPass(p,motion.axesAt(p.start),motion.axesAt(p.end),16));}
      var elapsed=Sys.time()-started,stats=history.statistics(passes[0].start,passes[passes.length-1].end);
      var text=new StringBuf(),loadValues:Array<Float> = [];
      for(s in history.samples){text.add('${s.removedVolume},${s.radialForce},${s.tangentialForce},${s.power};');
        for(value in [s.removedVolume,s.radialForce,s.tangentialForce,s.power,s.hobTorque,s.tableTorqueBound,s.energy])loadValues.push(value);}
      var geometry=signature(stock);
      var result:ForceSpeedResult={summary:{peakTangentialN:loads.maxTangential,meanRadialN:stats.radialMeanN,
        peakRadialN:loads.maxRadial,radialStdDevN:stats.radialStdDevN,perPassRadialMaxStepN:stats.perPassRadialMaxStepN,
        perPassRadialRmsStepN:stats.perPassRadialRmsStepN,volumeMm3:loads.volume,toleranceMm:tol,pathToleranceMm:tol/16,
        threads:threads,windows:passes.length,wallSeconds:elapsed},stockHash:geometry.hash,stockValues:geometry.values,stockSources:geometry.sources,loadValues:loadValues,loadHash:haxe.crypto.Sha256.encode(text.toString()),seconds:elapsed};
      loads.dispose();sweep.dispose();stock.dispose();return result;
    }catch(e:Dynamic){loads.dispose();sweep.dispose();stock.dispose();throw e;}
  }
  static function sameFloats(a:Array<Float>,b:Array<Float>):Bool {
    if(a.length!=b.length)return false;
    for(i in 0...a.length)if(a[i]!=b[i]||(a[i]==0&&1/a[i]!=1/b[i]))return false;
    return true;
  }
  static function sameInts(a:Array<Int>,b:Array<Int>):Bool {
    if(a.length!=b.length)return false;for(i in 0...a.length)if(a[i]!=b[i])return false;return true;
  }
  public static function run(threadsOnly:Bool=false):Void {
    var spec=new GearSpec(0.3,200,8,20*Math.PI/180),hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),1,32,12);
    var motion=new IdealHobbingJob(spec,hob,3000,1.05,0.5,4);
    var warm=RingStock.fromGearBlank(spec,0.6,0.05,1);warm.setThreads(4);
    var schedule=motion.schedule(warm.grid),split=0;
    var seedTolerance=0.001,warmSweep=new HobbingSweep(hob,-10,10,seedTolerance);
    var seedSeconds=200*60/3000.0,started=Sys.time();
    while(split<schedule.length&&schedule[split].start<seedSeconds){var p=schedule[split];warmSweep.cutPass(warm,motion.axesAt,p,seedTolerance/16);split++;
      if(split%400==0){Sys.println('FORCE_SEED $split windows');Sys.stdout().flush();}}
    var seed=warm.snapshot();warmSweep.dispose();warm.dispose();
    Sys.println('FORCE_SEED_SECONDS ${Sys.time()-started}');Sys.stdout().flush();
    var virgin=new IdealHobbingJob(spec,hob,3000,0.05,0.5,4);
    var grid=RingStock.fromGearBlank(spec,0.6,0.05,1),virginPasses=virgin.schedule(grid.grid);grid.dispose();
    var candidates=[0.0002,0.0005,0.001,0.002];
    var valid=[for(t in candidates)true],results:Array<Dynamic> = [];
    try {
      for(fixture in ["virgin-midface","common-precut-midface"]){
        var passes=fixture=="virgin-midface"?virginPasses:schedule.slice(split),at=fixture=="virgin-midface"?virgin:motion;
        var snapshot=fixture=="virgin-midface"?null:seed;
        var one=cut(spec,hob,at,passes,fine,1,snapshot),many=cut(spec,hob,at,passes,fine,4,snapshot);
        if(one.stockHash!=many.stockHash||one.loadHash!=many.loadHash||!sameFloats(one.stockValues,many.stockValues)||!sameInts(one.stockSources,many.stockSources)||!sameFloats(one.loadValues,many.loadValues))throw 'thread count changes stock or per-pass loads: $fixture';
        Sys.println('FORCE_THREADS '+haxe.Json.stringify({fixture:fixture,one:one.summary,many:many.summary,
          stockHash:one.stockHash,loadHash:one.loadHash,identical:true,exactScalarComparison:true}));Sys.stdout().flush();
        if(threadsOnly)continue;
        for(i in 0...candidates.length){var trial=cut(spec,hob,at,passes,candidates[i],4,snapshot);
          var peak=Math.abs(trial.summary.peakTangentialN/one.summary.peakTangentialN-1);
          var mean=Math.abs(trial.summary.meanRadialN/one.summary.meanRadialN-1);
          var stddev=Math.abs(trial.summary.radialStdDevN/one.summary.radialStdDevN-1);
          var maxStep=Math.abs(trial.summary.perPassRadialMaxStepN/one.summary.perPassRadialMaxStepN-1);
          var rmsStep=Math.abs(trial.summary.perPassRadialRmsStepN/one.summary.perPassRadialRmsStepN-1);
          var ok=peak<0.01&&mean<0.01&&stddev<0.01&&maxStep<0.01&&rmsStep<0.01
            &&Math.isFinite(peak)&&Math.isFinite(mean)&&Math.isFinite(stddev)&&Math.isFinite(maxStep)&&Math.isFinite(rmsStep);
          valid[i]=valid[i]&&ok;
          var row={fixture:fixture,reference:one.summary,candidate:trial.summary,peakRelativeChange:peak,meanRelativeChange:mean,radialStdDevRelativeChange:stddev,perPassMaxStepRelativeChange:maxStep,perPassRmsStepRelativeChange:rmsStep,passesOnePercent:ok};
          results.push(row);Sys.println('FORCE_TOLERANCE '+haxe.Json.stringify(row));Sys.stdout().flush();
        }
      }
      if(threadsOnly){seed.dispose();Sys.println("FORCE_THREAD_ACCEPTANCE exact endpoints, provenance and every load/energy scalar at 1 and 4 threads");return;}
      var selected=fine;for(i in 0...candidates.length)if(valid[i])selected=candidates[i];
      if(selected==fine)throw "no coarser force tolerance passed the 1 percent peak/mean criterion";
      Sys.println('FORCE_SPEED_ACCEPTANCE '+haxe.Json.stringify({recommendedToleranceMm:selected,threads:4,fineToleranceMm:fine,
        relativeForceCriterion:0.01,threadStockAndPerPassLoadsIdentical:true,seedToleranceMm:seedTolerance,
        scope:"short reference-geometry virgin engagement plus common-precut midface; full-stroke peak/mean validation still required; metrology tolerance unchanged"}));
    }catch(e:Dynamic){seed.dispose();throw e;}
    seed.dispose();
  }
}
