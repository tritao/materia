import gearkit.*;
import stockkit.*;
import stockkit.hobbing.*;

/** Diagnostic only: exact reference circles and schedule, with full original tool/probes. */
class HB4ConvergenceProbe {
  public static function run():Void {
    var spec=new GearSpec(0.3,200,8,20*Math.PI/180),profile=new ToothProfile(spec);
    var lo=profile.rootJoinRadius+0.002,hi=profile.tipJoinRadius-0.002;
    var grid=new RingGrid(lo,hi,-0.02,8.02,(hi-lo)/185,0.04);
    var cfg=new GearInspection(spec,lo+0.015*(hi-lo),null,4,4,hi-0.015*(hi-lo));
    var cross=InspectionRingStock.create(grid,cfg);if(cross==null)throw "reference inspection domains unavailable";
    // Worst lead circle at levels 1, 2 and 3, plus the worst profile circle at level 3.
    var coordinates=[{r:cross.stocks[1].grid.radius(1),z:cross.stocks[1].grid.z(186)},
      {r:cross.stocks[1].grid.radius(1),z:cross.stocks[1].grid.z(83)},
      {r:cross.stocks[1].grid.radius(1),z:cross.stocks[1].grid.z(81)},
      {r:cross.stocks[0].grid.radius(63),z:cross.stocks[0].grid.z(0)}];
    cross.dispose();
    var stocks:Array<RingStock> =[];
    function bounds(center:Float):Array<Float> {
      var d=1.0/1024,low=center-d,high=center+d;
      function ulp(v:Float):Float return Math.pow(2,Math.floor(Math.log(Math.abs(v))/Math.log(2))-52);
      for(a in -4...5)for(b in -4...5){var l=low+a*ulp(low),h=high+b*ulp(high);
        if(l+0.5*(h-l)==center)return [l,h];}
      throw 'cannot isolate exact circle coordinate $center';
    }
    for(p in coordinates){var r=bounds(p.r),z=bounds(p.z),g=new RingGrid(r[0],r[1],z[0],z[1],1,1);
      if(g.radialCount!=1||g.axialCount!=1||g.radius(0)!=p.r||g.z(0)!=p.z)throw "probe coordinates are not bit-identical";
      var stock=RingStock.annulus(g);stock.setThreads(4);stocks.push(stock);}
    var hob=new HobDesign(ConjugateRack.of(profile),1,32,12),motion=new IdealHobbingJob(spec,hob,3000,1.1,0,4);
    var sweep=new HobbingSweep(hob,-28,28,0.00005),passes=motion.schedule(grid),began=Sys.time();
    var output=Sys.getEnv("HB4_PROBE_EVIDENCE");if(output==null)throw "HB4_PROBE_EVIDENCE required";
    var events:Array<Dynamic> =[],previous:Array<Array<RingInterval>> =[];
    function save(event:Dynamic):Void {
      events.push(event);sys.io.File.saveContent(output+"/progress.json",haxe.Json.stringify(events));
      Sys.println("HB4_PROBE "+haxe.Json.stringify(event));Sys.stdout().flush();
    }
    save({phase:"settings",coordinates:coordinates,threads:4,pairedDomains:Sys.getEnv("STOCKKIT_RAKE_BATCH_DOMAINS")=="1",
      sweepToleranceMm:0.00005,maxRefinements:5,windows:passes.length,toolFirstTooth:-28,toolLastTooth:28,
      scope:"diagnostic circles only; original complete schedule and tool; no reference acceptance claim"});
    try {
      var report=sweep.cutMany(stocks,motion.axesAt,passes,5,0.001,function(event:Dynamic):Void {
        if(event.phase=="cut"&&event.completed==passes.length){
          var current=[for(stock in stocks)stock.circle(0,0)],changes:Array<Dynamic> =[];
          if(previous.length>0)for(i in 0...current.length){var a=previous[i],b=current[i],peak=0.0,total=0.0,worst=-1;
            if(a.length==b.length)for(j in 0...a.length){
              var low=Math.abs(a[j].lo-b[j].lo)*coordinates[i].r,high=Math.abs(a[j].hi-b[j].hi)*coordinates[i].r;
              total+=low+high;if(Math.max(low,high)>peak){peak=Math.max(low,high);worst=j;}}
            changes.push({domain:i,previousIntervals:a.length,intervals:b.length,sameTopology:a.length==b.length,
              maximumEndpointShiftMm:peak,sumEndpointShiftsMm:total,worstInterval:worst,
              previousWorst:worst<0?null:a[worst],currentWorst:worst<0?null:b[worst]});
          }
          save({phase:"endpoints",level:event.level,changes:changes,poses:event.poses,seconds:event.seconds});
          sys.io.File.saveContent(output+'/circles-level-${event.level}.json',haxe.Json.stringify(current));previous=current;
        }
        if(event.phase=="comparison")save(event);
      });
      save({phase:"result",converged:true,refinement:report.refinement,maxArcChangeMm:report.maxArcChange,seconds:Sys.time()-began});
    }catch(error:Dynamic){save({phase:"result",converged:false,error:Std.string(error),seconds:Sys.time()-began});dispose(stocks,sweep);throw error;}
    dispose(stocks,sweep);
  }
  static function dispose(stocks:Array<RingStock>,sweep:HobbingSweep):Void {for(stock in stocks)stock.dispose();sweep.dispose();}
}
