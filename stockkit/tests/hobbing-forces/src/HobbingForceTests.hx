import gearkit.*;
import stockkit.*;
import stockkit.hobbing.*;
class HobbingForceTests {
  static var checks=0;
  static function check(ok:Bool,label:String):Void {checks++;if(!ok)throw label;}
  static function near(a:Float,b:Float,label:String):Void check(Math.abs(a-b)<1e-9*Math.max(1,Math.abs(b)),label);
  static function forceThreads():Int {
    var raw=Sys.getEnv("HOBBER_FORCE_THREADS"),value=raw==null?4:Std.parseInt(raw);
    if(value==null||value<1||value>128)throw "HOBBER_FORCE_THREADS must be 1..128";
    return value;
  }
  static function forceTolerance():Float {
    var raw=Sys.getEnv("HOBBER_FORCE_TOLERANCE_MM"),value=raw==null?0.00005:Std.parseFloat(raw);
    if(!Math.isFinite(value)||value<=0)throw "invalid force sweep tolerance";
    return value;
  }
  static function referenceSweep(full:Bool=false):Void {
    var spec=new GearSpec(0.3,200,8,20*Math.PI/180);
    var hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),1,32,12);
    var cam=new camkit.HobbingJob(spec,hob,40),path=cam.passes[0];
    var job=new IdealHobbingJob(spec,hob,cam.rpm,full?path.revolutions:0.01,0.5,full?path.zStart:4);
    var threads=forceThreads(),tolerance=forceTolerance();
    var stock=RingStock.fromGearBlank(spec,0.6,0.05,1);stock.setThreads(threads);
    var sweep=new HobbingSweep(hob,-10,10,tolerance);
    var load=new HobbingForces(stock,spec.pitchRadius(),spec.pressureAngle,1);
    var initial=RingStock.annulus(stock.grid),history=new HobbingForceHistory(),started=Sys.time();
    try {
      var passes=job.schedule(stock.grid);
      for(pass in passes){sweep.cutPass(stock,job.axesAt,pass,sweep.tolerance/16);
        history.add(load.afterPass(pass,job.axesAt(pass.start),job.axesAt(pass.end),16));
        if(full&&load.passes%100==0){Sys.println('REFERENCE_PROGRESS ${load.passes}/${passes.length} windows; Fc_peak=${load.maxTangential} N; Fr_peak=${load.maxRadial} N');Sys.stdout().flush();}}
      near(load.volume,initial.compareStock(stock).leftoverVolume(),"gash chip volumes telescope to stock difference");
      check(load.volume>0&&load.maxTangential>0,"physical reference rake sweep gives nonzero loads");
      var stats=history.statistics(0,job.duration);
      near(stats.radialMeanN,load.energy/(cam.rpm*2*Math.PI/60*0.016*job.duration)*Math.tan(spec.pressureAngle),"radial time integral agrees with chip energy at constant cutting speed");
      var historyPath=full?Sys.getEnv("HOBBER_FORCE_HISTORY"):null;
      if(full&&historyPath==null)historyPath="stockkit/tests/hobbing-forces/build/host/reference-force-passes.csv";
      var report={seconds:job.duration,wallSeconds:Sys.time()-started,volumeMm3:load.volume,
        forceN:load.maxTangential,radialN:load.maxRadial,energyJ:load.energy,
        meanRadialN:stats.radialMeanN,radialVariation:stats,radialRangeBoundN:stats.radialRangeN,
        meanBasis:stats.basis,radialRangeBasis:"measured range of overlapping rectangular radial window means; see per-pass history, not instantaneous tooth peaks",
        forceComponents:"forceN=tangential; radialN and all deflection inputs=radial=Fc*tan(pressure angle)",
        moduleMm:spec.module,teeth:spec.teeth,faceWidthMm:spec.faceWidth,gashes:hob.gashes,
        tableRpm:cam.tableRpm,feedMmPerWorkRevolution:0.5,zStartMm:job.zStart,zEndMm:job.axesAt(job.duration).z,
        threads:threads,sweepToleranceMm:tolerance,pathToleranceMm:tolerance/16,radialStepMm:0.05,axialStepMm:1,
        historyCsv:historyPath,torqueNm:load.maxHobTorque,tableTorqueBoundNm:load.maxTableTorqueBound,
        powerW:load.maxPower,windows:load.passes,
        scope:full?"complete ideal HB8 reference feed stroke, coarse ring grid":"short mid-face initial engagement; not a full reference cycle"};
      if(historyPath!=null){history.writeCsv(historyPath);sys.io.File.saveContent(historyPath+".json",haxe.Json.stringify(report));}
      Sys.println('REFERENCE_FORCE '+haxe.Json.stringify(report));Sys.stdout().flush();
      if(full){
        check(load.maxTangential>=10&&load.maxTangential<100,"full reference overlapping-window loads in tens of N");
        check(load.maxPower<1500*0.8&&load.maxHobTorque<(1500/(4000*2*Math.PI/60))*0.8,"full reference spindle 20% reserve");
      }else check(load.maxPower>1500*0.8||load.maxHobTorque>(1500/(4000*2*Math.PI/60))*0.8,"mid-face plunge correctly exposes overload");
      check(load.peakActiveWindows<32,"overlap accumulator bounded");
    }catch(e:Dynamic){initial.dispose();load.dispose();sweep.dispose();stock.dispose();throw e;}
    initial.dispose();load.dispose();sweep.dispose();stock.dispose();
  }
  static function quickCut():Void {
    var spec=new GearSpec(0.3,40,1,20*Math.PI/180),hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),1,32,4);
    var settings=new camkit.HobbingSettings();settings.feedPerRevolution=5;
    var cam=new camkit.HobbingJob(spec,hob,40,settings),path=cam.passes[0];
    var motion=new IdealHobbingJob(spec,hob,cam.rpm,path.revolutions,5,path.zStart);
    var threads=forceThreads();
    var stock=RingStock.fromGearBlank(spec,0.6,0.075,0.5);stock.setThreads(threads);
    var initial=RingStock.annulus(stock.grid),sweep=new HobbingSweep(hob,-10,10,0.0001);
    var loads=new HobbingForces(stock,spec.pitchRadius(),spec.pressureAngle,1);
    try {
      for(pass in motion.schedule(stock.grid)){sweep.cutPass(stock,motion.axesAt,pass,sweep.tolerance/16);
        loads.afterPass(pass,motion.axesAt(pass.start),motion.axesAt(pass.end),16);}
      Sys.println("QUICK_FORCE "+haxe.Json.stringify({cycleSeconds:motion.duration,windows:loads.passes,
        threads:threads,sweepToleranceMm:0.0001,pathToleranceMm:0.0001/16,volumeMm3:loads.volume,forceN:loads.maxTangential,radialN:loads.maxRadial,powerW:loads.maxPower,
        hobTorqueNm:loads.maxHobTorque,scope:"m0.3 z40 width1 coarse complete ideal cut, feed5; not reference acceptance"}));
      check(loads.volume>0&&Math.isFinite(loads.maxPower),"complete quick cut gives finite nonzero loads");
      near(loads.volume,initial.compareStock(stock).leftoverVolume(),"quick cut chip volume conservation");
      check(loads.maxPower<1200&&loads.maxHobTorque<1500/(4000*2*Math.PI/60)*0.8,"quick cut 20% spindle reserve");
    }catch(e:Dynamic){loads.dispose();sweep.dispose();initial.dispose();stock.dispose();throw e;}
    loads.dispose();sweep.dispose();initial.dispose();stock.dispose();
    Sys.println('Quick hobbing force cut: $checks checks passed');
  }
  public static function main():Void {
    if(Sys.args().indexOf("threads")>=0){ForceSpeedChecks.run(true);return;}
    if(Sys.args().indexOf("speed")>=0){ForceSpeedChecks.run();return;}
    if(Sys.args().indexOf("quick")>=0){quickCut();return;}
    if(Sys.args().indexOf("reference")>=0){referenceSweep(true);return;}
    var material=KienzleMaterial.referenceSteel();
    near(material.force(0,1),0,"air cut is zero");
    near(material.force(0.02,1)/material.force(0.01,1),Math.pow(2,1-material.mc),"feed exponent at fixed edge width/travel");
    var p=new GashPass(0,0,0,0.5,1);
    var sample=new HobbingForceSample(p,0.1,10,16,30,1,20*Math.PI/180,material);
    near(sample.chipArea,0.01,"volume / travel");near(sample.power,sample.hobTorque*10/16,"torque times omega");
    near(sample.radialForce,sample.tangentialForce*Math.tan(20*Math.PI/180),"separating force assumption");
    var history=new HobbingForceHistory();history.add(sample);
    history.add(new HobbingForceSample(new GashPass(1,1,0.5,1,1.5),0.1,10,16,30,1,20*Math.PI/180,material));
    history.add(new HobbingForceSample(new GashPass(0,2,1.5,1.75,2),0,10,16,30,1,20*Math.PI/180,material));
    var stats=history.statistics(0,2),f=sample.radialForce;
    near(stats.radialMeanN,f,"overlapping radial window time mean");
    near(stats.radialStdDevN,f/Math.sqrt(2),"radial window time variance");
    near(stats.radialRangeN,2*f,"radial variation range excludes mean offset");
    near(stats.perPassRadialMaxStepN,f,"successive per-pass radial variation");
    near(stats.perPassRadialRmsStepN,f/Math.sqrt(2),"successive per-pass radial RMS step");
    near(stats.activeMeanRadialN,4*f/3,"mean over engaged time is separate from whole-stroke mean");
    near(history.radialAt(0.75),2*f,"measured radial history lookup sums overlaps");
    near(history.radialAt(3),0,"completed load history has zero radial force after the last window");
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
