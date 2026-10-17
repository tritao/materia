import gearkit.*;
import stockkit.*;
import stockkit.hobbing.*;
class HobbingStreamTests {
  public static function main():Void {
    var spec=new GearSpec(1,40,2,20*Math.PI/180),hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),1,32,4);
    var ideal=new IdealHobbingJob(spec,hob,3000,1.05,0,1);
    var grid=new RingGrid(spec.rootRadius()-0.6,spec.tipRadius(),0,2,0.1,1);
    var actual=RingStock.annulus(grid),oracle=RingStock.annulus(grid),sweep=new HobbingSweep(hob,-12,12,0.000025);
    actual.setThreads(1);oracle.setThreads(1);
    var half=Math.acos((ideal.centerDistance-grid.radiusMax)/16)+0.01;
    var stream=new HobbingStream(actual,sweep,half),frames:Array<HobbingKeyframe> = [];
    var forces=new HobbingForces(actual,spec.pitchRadius(),spec.pressureAngle,1);
    stream.onPass=(pass,start,end)->{forces.afterPass(pass,start,end,16);};
    try{
      var count=Std.int(Math.ceil(ideal.duration*4000));
      for(i in 0...count+1){var time=ideal.duration*i/count,axes=ideal.axesAt(time);frames.push(new HobbingKeyframe(time,axes));stream.push(time,axes);}
      stream.finish();
      if(forces.passes!=stream.passes||Math.abs(forces.volume-oracle.compareStock(actual).leftoverVolume())>1e-8)
        throw "force observer missed or double-counted streamed material removal";
      var motion=new HobbingMotion(frames),passes=GashPass.fromMotion(motion,hob.gashes,half);
      for(pass in passes)sweep.cutPass(oracle,motion.at,pass,sweep.tolerance/16,0.001,motion.times());
      var c=actual.compareStock(oracle),worst=0.0;
      for(z in 0...grid.axialCount)for(r in 0...grid.radialCount){var index=z*grid.radialCount+r;worst=Math.max(worst,(c.leftover[index]+c.gouge[index])*grid.radius(r));}
      if(worst>0.0001)throw 'stream differs from HB3 by $worst mm';
      if(stream.peakFrames>100||actual.bytes()>8000000)throw "stream allocation budget";
      var worker=new RingPreviewWorker(actual);try{if(!worker.refresh())throw "preview refresh";worker.wait();var mesh=worker.take();if(mesh==null||mesh.triangleCount==0)throw "missing ring preview";}
      catch(e:Dynamic){worker.dispose();throw e;}worker.dispose();
      Sys.println('Hobbing stream: HB3 difference ${worst*1000} um; ${stream.passes} exact windows; peak ${stream.peakFrames} samples; ${actual.bytes()} bytes');
    }catch(e:Dynamic){forces.dispose();sweep.dispose();actual.dispose();oracle.dispose();throw e;}
    forces.dispose();sweep.dispose();actual.dispose();oracle.dispose();
  }
}
