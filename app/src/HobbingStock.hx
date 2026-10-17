package app;

import haxe.io.Bytes;
import gearkit.*;
import stockkit.*;
import stockkit.hobbing.*;
import nativekit.scene.GeometryData;
import materia.project.SceneArtifact.SceneArtifactHobbing;

/** Actual measured six-axis stock removal, with a worker-owned display snapshot. */
class HobbingStock {
  public final spec:GearSpec;
  public final hob:HobDesign;
  public final stock:RingStock;
  public final sweep:HobbingSweep;
  public final stream:HobbingStream;
  public final preview:RingPreviewWorker;
  public var measurement(default,null):Null<GearMeasurement> = null;
  public var measurementFailure(default,null):Null<String> = null;
  public var maxBytes(default,null):Float=0;
  public var collisions(default,null):Int=0;
  public var report(default,null):String="Cutting";
  final center:Array<Float>;
  final config:SceneArtifactHobbing;
  var previewPasses=-1;
  var lastPreviewTime=-1e30;
  var finalPreview=false;
  var disposed=false;
  var lastMaster=0.0;
  var streamStopped=false;
  public function new(config:SceneArtifactHobbing,center:Array<Float>) {
    this.config=config;this.center=center.copy();
    spec=new GearSpec(config.module,config.teeth,config.width,config.pressureAngle);
    hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),config.starts,config.outside,config.gashes);
    stock=RingStock.fromGearBlank(spec,config.wall,config.radialStep,config.axialStep);stock.setThreads(1);
    var span=Std.int(Math.ceil(config.length/(2*hob.axialPitch)))+2;
    sweep=new HobbingSweep(hob,-span,span,config.tolerance);
    var distance=spec.pitchRadius()+hob.pitchDiameter/2;
    var half=Math.acos(Math.max(-1,Math.min(1,(distance-stock.grid.radiusMax)/(config.outside/2))))+Math.abs(hob.rake)+0.01;
    stream=new HobbingStream(stock,sweep,Math.min(Math.PI/2-1e-6,half));
    preview=new RingPreviewWorker(stock);maxBytes=stock.bytes();
  }
  public function follow(time:Float,axes:HobbingAxes):Void {
    if(!streamStopped){
      if(axes.b<lastMaster-1e-8&&axes.x>stock.grid.radiusMax+config.outside/2){stream.finish();streamStopped=true;}
      else stream.push(time,axes);
      lastMaster=axes.b;
    }
    maxBytes=Math.max(maxBytes,stock.bytes());
    // Independent circular fixture separating planes, evaluated at measured coordinates.
    if(axes.x-config.outside/2<spec.rootRadius()-config.wall-0.025)collisions++;
    var lowest=axes.z-Math.abs(axes.y*Math.sin(axes.a))-config.length/2*Math.abs(Math.sin(axes.a))-config.outside/2*Math.abs(Math.cos(axes.a));
    if(lowest < -55)collisions++;
  }
  public function finish(seconds:Float,sync:Float):Void {
    stream.finish();finalPreview=true;maxBytes=Math.max(maxBytes,stock.bytes());
    try{measurement=GearMeasurement.fromRings(stock,spec);}catch(e:Dynamic){measurementFailure=Std.string(e);}
    var m=measurement;
    report='Cycle ${Math.round(seconds*1000)/1000} s · sync ${Math.round(sync*648000/Math.PI*1000)/1000} arcsec · fixture contacts $collisions\n'
      +'${stream.passes} gash windows · ${stream.poses} poses · peak ${stream.peakFrames} buffered samples · ${Math.round(maxBytes/1024)} KiB stock';
    if(m==null)report+='\nInspection incomplete: '+measurementFailure;
    else {
      report+='\nCandidate grade ${m.overallGrade==null?"worse than 11 / incomplete":Std.string(m.overallGrade)}; ISO class ${m.isoClass==null?"not qualified":Std.string(m.isoClass)}\n'+m.warnings.concat(m.extrapolation).join("\n");
      for(trace in m.profiles.slice(0,2))report+='\nProfile tooth ${trace.tooth} flank ${trace.flank} [um]: '+traceText(trace);
      for(trace in m.helices.slice(0,2))report+='\nLead tooth ${trace.tooth} flank ${trace.flank} [um]: '+traceText(trace);
    }
  }
  static function traceText(trace:GearTrace):String {
    var values:Array<String> = [];var stride=Std.int(Math.max(1,Math.floor(trace.filtered.length/16)));
    var i=0;while(i<trace.filtered.length){values.push(Std.string(Math.round(trace.filtered[i]*1000)/1000));i+=stride;}
    return values.join(" ");
  }
  public function geometry():Null<GeometryData> {
    if(stream.passes!=previewPasses&&(finalPreview||stream.lastTime-lastPreviewTime>=0.1)&&preview.refresh()){previewPasses=stream.passes;lastPreviewTime=stream.lastTime;finalPreview=false;}
    var mesh=preview.take();if(mesh==null)return null;
    var positions=Bytes.alloc(mesh.vertexCount*12);
    for(i in 0...mesh.vertexCount)for(axis in 0...3)positions.setFloat(i*12+axis*4,mesh.positions.getFloat(i*12+axis*4)/1000-center[axis]);
    var g=new GeometryData();g.addStream(1,2,positions,mesh.vertexCount,12);g.addStream(2,2,mesh.normals,mesh.vertexCount,12);
    g.addStream(6,4,mesh.colors,mesh.vertexCount,4);g.setIndexBuffer(mesh.indices,mesh.triangleCount*3);
    g.setBounds(-stock.grid.radiusMax/1000-center[0],-stock.grid.radiusMax/1000-center[1],stock.grid.zMin/1000-center[2],
      stock.grid.radiusMax/1000-center[0],stock.grid.radiusMax/1000-center[1],stock.grid.zMax/1000-center[2]);return g;
  }
  public function dispose():Void{if(disposed)return;disposed=true;preview.dispose();sweep.dispose();stock.dispose();}
}
