package stockkit.hobbing;
import gearkit.*;
import stockkit.RingStock;

/** Repeats physical rake subtraction and HB4 inspection from a fresh blank for
    every budget run. at() may be ideal motion or recorded HB7 actual observations;
    measured sync must not be injected a second time. No target-boundary shortcuts. */
class HobbingErrorJob {
  public final sources:Array<HobbingErrorSource>;
  final spec:GearSpec;final hob:HobDesign;final at:Float->HobbingAxes;
  final passes:Array<GashPass>;final wall:Float;final radialStep:Float;final axialStep:Float;
  final firstTooth:Int;final lastTooth:Int;final tolerance:Float;final inspection:Null<GearInspection>;
  final motionKnots:Null<Array<Float>>;
  public function new(spec:GearSpec,hob:HobDesign,at:Float->HobbingAxes,passes:Array<GashPass>,
      sources:Array<HobbingErrorSource>,wall:Float,radialStep:Float,axialStep:Float,
      firstTooth:Int,lastTooth:Int,tolerance:Float,?inspection:GearInspection,?motionKnots:Array<Float>) {
    this.spec=spec;this.hob=hob;this.at=at;this.passes=passes.copy();this.sources=sources.copy();
    this.motionKnots=motionKnots==null?null:motionKnots.copy();
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
    var stock=RingStock.fromGearBlank(spec,wall,radialStep,axialStep);stock.setThreads(1);
    var sweep:Null<HobbingSweep> = null;
    try {
      sweep=new HobbingSweep(hob,firstTooth,lastTooth,tolerance,warp);
      for(pass in passes)sweep.cutPass(stock,motion,pass,tolerance/16,0.001,motionKnots);
      var measured=GearMeasurement.fromRings(stock,spec,inspection);sweep.dispose();stock.dispose();return measured;
    }catch(e:Dynamic){if(sweep!=null)sweep.dispose();stock.dispose();throw e;}
  }
  public function budget(relativeTolerance:Float=0.25,absoluteTolerance:Float=0.1):ErrorBudget
    return new ErrorBudget([for(source in sources){id:source.id,part:source.part,basis:source.basis}],run,relativeTolerance,absoluteTolerance);
}
