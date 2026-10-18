import gearkit.*;
import stockkit.*;
import stockkit.hobbing.*;

private typedef BoundaryProbeInput={radius:Float,z:Float,targetAngle:Float,radialLow:Float,radialHigh:Float,zLow:Float,zHigh:Float,
  passes:Array<Int>,expected:Array<{lo:Float,hi:Float,loPass:Int,hiPass:Int}>,source:String};

/** Local diagnostic, validated against full-stroke endpoint snapshots before refinement. */
class HB4BoundaryProbe {
  public static function run():Void {
    var output=Sys.getEnv("HB4_BOUNDARY_EVIDENCE");if(output==null)throw "HB4_BOUNDARY_EVIDENCE required";
    var inputPath=Sys.getEnv("HB4_BOUNDARY_INPUT");if(inputPath==null)inputPath=output+"/input.json";
    var raw:Dynamic=haxe.Json.parse(sys.io.File.getContent(inputPath));
    var rawPasses:Array<Dynamic> =cast raw.passes,rawExpected:Array<Dynamic> =cast raw.expected;
    var input:BoundaryProbeInput={radius:raw.radius,z:raw.z,targetAngle:raw.targetAngle,
      radialLow:raw.radialLow,radialHigh:raw.radialHigh,zLow:raw.zLow,zHigh:raw.zHigh,
      passes:[for(p in rawPasses)Std.int(p)],expected:[for(e in rawExpected){lo:(e.lo:Float),hi:(e.hi:Float),loPass:Std.int(e.loPass),hiPass:Std.int(e.hiPass)}],source:raw.source};
    var spec=new GearSpec(0.3,200,8,20*Math.PI/180),p=new ToothProfile(spec),lo=p.rootJoinRadius+0.002,hi=p.tipJoinRadius-0.002;
    var original=new RingGrid(lo,hi,-0.02,8.02,(hi-lo)/185,0.04);
    var hob=new HobDesign(ConjugateRack.of(p),1,32,12),motion=new IdealHobbingJob(spec,hob,3000,1.1,0,4);
    var grid=new RingGrid(input.radialLow,input.radialHigh,input.zLow,input.zHigh,1,1);
    if(grid.radialCount!=1||grid.axialCount!=1||grid.radius(0)!=input.radius||grid.z(0)!=input.z)throw "boundary probe circle moved";
    var passes=[for(pass in motion.schedule(original))if(input.passes.indexOf(pass.pass)>=0)pass];
    if(passes.length!=input.passes.length)throw "boundary probe pass ids unavailable";
    var stock=RingStock.annulus(grid),sweep=new HobbingSweep(hob,-28,28,0.00005),initial=stock.snapshot();stock.setThreads(4);
    var results:Array<Dynamic> =[],previous:Null<RingInterval> =null,began=Sys.time();
    try {
      for(level in 0...6){stock.restore(initial);var poses=0,step=0.00005/(4*Math.pow(4,level)),started=Sys.time();
        for(pass in passes)poses+=sweep.cutPass(stock,motion.axesAt,pass,step,0.001);
        var interval:Null<RingInterval> =null;
        for(candidate in stock.circle(0,0))if(candidate.lo<=input.targetAngle&&candidate.hi>=input.targetAngle){interval=candidate;break;}
        if(interval==null)throw "target tooth interval missing";
        // Require exact endpoints and source provenance at both completed full-stroke levels.
        if(level<input.expected.length){var expected=input.expected[level];
          if(interval.lo!=expected.lo||interval.hi!=expected.hi||interval.loPass!=expected.loPass||interval.hiPass!=expected.hiPass)
            throw 'localized window does not reproduce full stroke at level $level';}
        var row={level:level,passes:passes.length,poses:poses,pathToleranceMm:step,seconds:Sys.time()-started,
          interval:interval,maximumEndpointShiftMm:previous==null?null:Math.max(Math.abs(interval.lo-previous.lo),Math.abs(interval.hi-previous.hi))*input.radius,
          sumEndpointShiftsMm:previous==null?null:(Math.abs(interval.lo-previous.lo)+Math.abs(interval.hi-previous.hi))*input.radius,
          fullStrokeEndpointValidated:level<input.expected.length};
        results.push(row);sys.io.File.saveContent(output+"/levels.json",haxe.Json.stringify(results));
        Sys.println("HB4_BOUNDARY "+haxe.Json.stringify(row));Sys.stdout().flush();previous=interval;
      }
      sys.io.File.saveContent(output+"/result.json",haxe.Json.stringify({levels:results,seconds:Sys.time()-began,
        radius:input.radius,z:input.z,threads:4,sweepToleranceMm:0.00005,source:input.source,
        scope:"one boundary neighbourhood, diagnostic only; no full-circle convergence or reference acceptance claim"}));
    }catch(error:Dynamic){initial.dispose();stock.dispose();sweep.dispose();throw error;}
    initial.dispose();stock.dispose();sweep.dispose();
  }
}
