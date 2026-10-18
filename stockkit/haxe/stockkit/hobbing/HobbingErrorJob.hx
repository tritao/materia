package stockkit.hobbing;
import gearkit.*;
import stockkit.RingStock;
import stockkit.RingGrid;

/** Repeats physical rake subtraction and HB4 inspection from a fresh blank for
    every budget run. at() may be ideal motion or recorded HB7 actual observations;
    measured sync must not be injected a second time. No target-boundary shortcuts. */
class HobbingErrorJob {
  public final sources:Array<HobbingErrorSource>;
  final spec:GearSpec;final hob:HobDesign;final at:Float->HobbingAxes;
  final passes:Array<GashPass>;final wall:Float;final radialStep:Float;final axialStep:Float;
  final firstTooth:Int;final lastTooth:Int;final tolerance:Float;final inspection:Null<GearInspection>;
  final motionKnots:Null<Array<Float>>;
  public final threads:Int;
  final inspectionCross:Bool;final faceEndpoints:Bool;final stockGrid:Null<RingGrid>;
  public function new(spec:GearSpec,hob:HobDesign,at:Float->HobbingAxes,passes:Array<GashPass>,
      sources:Array<HobbingErrorSource>,wall:Float,radialStep:Float,axialStep:Float,
      firstTooth:Int,lastTooth:Int,tolerance:Float,?inspection:GearInspection,?motionKnots:Array<Float>,threads:Int=4,inspectionCross:Bool=false,faceEndpoints:Bool=false,?stockGrid:RingGrid) {
    if(threads<1||threads>128)throw "hobbing error threads must be 1..128";
    this.threads=threads;this.inspectionCross=inspectionCross;this.faceEndpoints=faceEndpoints;this.stockGrid=stockGrid;
    this.spec=spec;this.hob=hob;this.at=at;this.passes=passes.copy();this.sources=sources.copy();
    this.motionKnots=motionKnots==null?null:motionKnots.copy();
    if(this.motionKnots!=null){for(t in this.motionKnots)if(!Math.isFinite(t))throw "motion knots must be finite";
      // Large measured traces must not use the SDK's insertion-sort lowering.
      var a=this.motionKnots,b=a.copy(),width=1,n=a.length;
      while(width<n){var start=0;
        while(start<n){var middle=Std.int(Math.min(start+width,n)),end=Std.int(Math.min(start+2*width,n)),i=start,j=middle,k=start;
          while(k<end){if(i<middle&&(j>=end||a[i]<=a[j]))b[k++]=a[i++];else b[k++]=a[j++];}start=end;}
        var swap=a;a=b;b=swap;width*=2;
      }
      for(i in 0...n)this.motionKnots[i]=a[i];}
    this.wall=wall;this.radialStep=radialStep;this.axialStep=axialStep;
    this.firstTooth=firstTooth;this.lastTooth=lastTooth;this.tolerance=tolerance;this.inspection=inspection;
    var ids:Array<String> = [];for(source in sources){if(ids.indexOf(source.id)>=0)throw "duplicate hobbing error source";ids.push(source.id);}
  }
  public function run(enabled:Array<String>):GearMeasurement {
    var selected:Array<HobbingErrorSource> = [];
    var seen:Array<String> = [];
    for(id in enabled){if(seen.indexOf(id)>=0)throw "duplicate enabled error";seen.push(id);var found=false;for(source in sources)if(source.id==id){selected.push(source);found=true;}if(!found)throw 'unknown hobbing error $id';}
    function warp(p:Point3):Point3 {var out=p;for(source in selected){var fn=source.warp;if(fn!=null)out=fn(out);}return out;}
    function motion(t:Float):HobbingAxes {var a=at(t);for(source in selected)a=source.apply(t,a);return a;}
    var blank=RingStock.fromGearBlank(spec,wall,radialStep,axialStep);
    if(stockGrid!=null){
      if(stockGrid.radiusMin<blank.grid.radiusMin||stockGrid.radiusMax>blank.grid.radiusMax||stockGrid.zMin<0||stockGrid.zMax>spec.faceWidth){blank.dispose();throw "inspection stock domain outside physical blank";}
      blank.dispose();blank=RingStock.annulus(stockGrid);
    }
    var cross=inspectionCross&&inspection!=null?InspectionRingStock.create(blank.grid,inspection):null;
    var stocks=cross==null?[blank]:cross.stocks;
    if(cross!=null)blank.dispose();
    var rings:GearRings=cross==null?cast blank:cast cross;
    if(faceEndpoints){try{var ends=new FaceEndpointStock(rings,stocks,blank.grid,spec.faceWidth);rings=ends;stocks=ends.stocks;}
      catch(e:Dynamic){for(stock in stocks)stock.dispose();throw e;}}
    for(stock in stocks)stock.setThreads(threads);
    var sampling=cross==null?"dense":"exact inspection circles; unobserved circles omitted";
    var computedCircles=0;for(stock in stocks)computedCircles+=stock.grid.radialCount*stock.grid.axialCount;
    Sys.println("HOBBING_ERROR_SETTINGS "+haxe.Json.stringify({sources:enabled,threads:threads,sweepToleranceMm:tolerance,pathToleranceMm:tolerance/16,radialStepMm:radialStep,axialStepMm:axialStep,sampling:sampling,computedCircles:computedCircles,faceEndpoints:faceEndpoints,scope:"physical metrology source cut; grading tolerance unchanged"}));
    var sweep:Null<HobbingSweep> = null;
    var started=Sys.time();
    try {
      sweep=new HobbingSweep(hob,firstTooth,lastTooth,tolerance,warp);
      var done=0;
      for(pass in passes){
        var local:Null<Array<Float>> = null;
        if(motionKnots!=null){var lo=0,hi=motionKnots.length;
          while(lo<hi){var mid=(lo+hi)>>1;if(motionKnots[mid]<=pass.start)lo=mid+1;else hi=mid;}
          local=[];while(lo<motionKnots.length&&motionKnots[lo]<pass.end){local.push(motionKnots[lo]);lo++;}}
        sweep.cutPassMany(stocks,motion,pass,tolerance/16,0.001,local);done++;
        if(done%100==0){Sys.println("HOBBING_ERROR_PROGRESS "+haxe.Json.stringify({sources:enabled,windows:done,total:passes.length,wallSeconds:Sys.time()-started}));Sys.stdout().flush();}}

      if(cross!=null)cross.assertOverlap();
      var measured=GearMeasurement.fromRings(rings,spec,inspection);
      Sys.println("HOBBING_ERROR_RESULT "+haxe.Json.stringify({sources:enabled,threads:threads,sweepToleranceMm:tolerance,pathToleranceMm:tolerance/16,windows:passes.length,wallSeconds:Sys.time()-started,isoClass:measured.isoClass,sampling:sampling,computedCircles:computedCircles,faceEndpoints:faceEndpoints,scope:"physical source cut; grading tolerance unchanged"}));
      sweep.dispose();for(stock in stocks)stock.dispose();return measured;
    }catch(e:Dynamic){if(sweep!=null)sweep.dispose();for(stock in stocks)stock.dispose();throw 'hobbing error run [${enabled.length==0?"baseline":enabled.join(",")}] failed: ${Std.string(e)}';}
  }
  public function budget(relativeTolerance:Float=0.25,absoluteTolerance:Float=0.1):ErrorBudget
    return new ErrorBudget([for(source in sources){id:source.id,part:source.part,basis:source.basis}],run,relativeTolerance,absoluteTolerance);
}
