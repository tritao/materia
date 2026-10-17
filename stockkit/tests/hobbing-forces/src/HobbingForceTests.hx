import gearkit.*;
import stockkit.*;
import stockkit.hobbing.*;
class HobbingForceTests {
  static var checks=0;
  static function check(ok:Bool,label:String):Void {checks++;if(!ok)throw label;}
  static function near(a:Float,b:Float,label:String):Void check(Math.abs(a-b)<1e-9*Math.max(1,Math.abs(b)),label);
  static function referenceSweep(full:Bool=false):Void {
    var spec=new GearSpec(0.3,200,8,20*Math.PI/180);
    var hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),1,32,12);
    var cam=new camkit.HobbingJob(spec,hob,40),path=cam.passes[0];
    var job=new IdealHobbingJob(spec,hob,cam.rpm,full?path.revolutions:0.01,0.5,full?path.zStart:4);
    var stock=RingStock.fromGearBlank(spec,0.6,0.05,1);stock.setThreads(1);
    var sweep=new HobbingSweep(hob,-10,10,0.00005);
    var load=new HobbingForces(stock,spec.pitchRadius(),spec.pressureAngle,1);
    var initial=RingStock.annulus(stock.grid);
    try {
      var passes=job.schedule(stock.grid);
      for(pass in passes){sweep.cutPass(stock,job.axesAt,pass,sweep.tolerance/16);
        load.afterPass(pass,job.axesAt(pass.start),job.axesAt(pass.end),16);}
      near(load.volume,initial.compareStock(stock).leftoverVolume(),"gash chip volumes telescope to stock difference");
      check(load.volume>0&&load.maxTangential>0,"physical reference rake sweep gives nonzero loads");
      if(full){
        check(load.maxTangential>=10&&load.maxTangential<100,"full reference overlapping-window loads in tens of N");
        check(load.maxPower<1500*0.8&&load.maxHobTorque<(1500/(4000*2*Math.PI/60))*0.8,"full reference spindle 20% reserve");
      }else check(load.maxPower>1500*0.8||load.maxHobTorque>(1500/(4000*2*Math.PI/60))*0.8,"mid-face plunge correctly exposes overload");
      check(load.peakActiveWindows<32,"overlap accumulator bounded");
      Sys.println('REFERENCE_FORCE '+haxe.Json.stringify({seconds:job.duration,volumeMm3:load.volume,
        forceN:load.maxTangential,radialN:load.maxRadial,torqueNm:load.maxHobTorque,
        tableTorqueBoundNm:load.maxTableTorqueBound,powerW:load.maxPower,windows:load.passes,
        scope:full?"complete ideal HB8 reference feed stroke, coarse ring grid":"short mid-face initial engagement; not a full reference cycle"}));
    }catch(e:Dynamic){initial.dispose();load.dispose();sweep.dispose();stock.dispose();throw e;}
    initial.dispose();load.dispose();sweep.dispose();stock.dispose();
  }
  public static function main():Void {
    if(Sys.args().indexOf("reference")>=0){referenceSweep(true);return;}
    var material=KienzleMaterial.referenceSteel();
    near(material.force(0,1),0,"air cut is zero");
    near(material.force(0.02,1)/material.force(0.01,1),Math.pow(2,1-material.mc),"feed exponent at fixed edge width/travel");
    var p=new GashPass(0,0,0,0.5,1);
    var sample=new HobbingForceSample(p,0.1,10,16,30,1,20*Math.PI/180,material);
    near(sample.chipArea,0.01,"volume / travel");near(sample.power,sample.hobTorque*10/16,"torque times omega");
    near(sample.radialForce,sample.tangentialForce*Math.tan(20*Math.PI/180),"separating force assumption");
    check(sample.fits(1500,1500/(4000*2*Math.PI/60),20),"assumed reference ratings with 20% reserve");
    check(!sample.fits(1,1,1),"overloaded rating fails");
    // Known ring-sector removal is independent of the cutter implementation.
    var stock=RingStock.annulus(new RingGrid(29,30,0,1,1,1));
    var observer=new HobbingForces(stock,30,20*Math.PI/180,1);
    try {
      stock.cutRegion([new RingCut(0,0,1,0,0.001,0,0)]);
      var a=new HobbingAxes(46,0,0,0,0,0),b=new HobbingAxes(46,0,0,0,0.625,0);
      var measured=observer.afterPass(p,a,b,16);
      near(measured.removedVolume,29.5*0.001,"exact sector-volume removal");
      near(measured.tangentialForce,material.force(0.0295/10,1),"actual removed intervals drive load");
      var air=observer.afterPass(new GashPass(1,1,1,1.5,2),a,b,16);
      near(air.power,0,"repeated pass removes nothing");near(observer.volume,0.0295,"volume counted once");
      stock.cutRegion([new RingCut(0,0,1,0.001,0.001,0,2)]);
      var overlap1=observer.afterPass(new GashPass(0,2,2,3,4),a,b,16);
      stock.cutRegion([new RingCut(0,0,1,0.002,0.001,1,3)]);
      var overlap2=observer.afterPass(new GashPass(1,3,3,4,5),a,b,16);
      near(observer.maxTangential,overlap1.tangentialForce+overlap2.tangentialForce,"overlapping gash means add");
      check(observer.peakActiveWindows==2,"expired windows leave bounded active set");
      near(observer.energy,measured.energy+overlap1.energy+overlap2.energy,"chip energies add once");
    }catch(e:Dynamic){observer.dispose();stock.dispose();throw e;}observer.dispose();stock.dispose();
    referenceSweep();
    Sys.println('Hobbing forces: $checks assertions passed');
  }
}
