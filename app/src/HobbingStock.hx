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
  public final forces:HobbingForces;
  public final preview:RingPreviewWorker;
  public var measurement(default,null):Null<GearMeasurement> = null;
  public var measurementFailure(default,null):Null<String> = null;
  public var maxBytes(default,null):Float=0;
  public var collisions(default,null):Int=0;
  public var report(default,null):String="Cutting";
  final centerMetres:Array<Float>;
  final config:SceneArtifactHobbing;
  final initialBand:nativekit.scene.GeometryData;
  final cupBody:Null<nativekit.scene.GeometryData>;
  var displayedBand:nativekit.scene.GeometryData;
  var lastLift=1e30;
  var lastBlank=false;
  var previewPasses=-1;
  var lastPreviewTime=-1e30;
  var finalPreview=false;
  var disposed=false;
  var lastMaster=0.0;
  var streamStopped=false;
  /** RingStock coordinates are mm; the centred scene-geometry offset is explicitly metres. */
  public function new(config:SceneArtifactHobbing,centerMetres:Array<Float>) {
    this.config=config;this.centerMetres=centerMetres.copy();
    spec=new GearSpec(config.module,config.teeth,config.width,config.pressureAngle);
    hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),config.starts,config.outside,config.gashes);
    stock=RingStock.fromGearBlank(spec,config.wall,config.radialStep,config.axialStep);stock.setThreads(1);
    var span=Std.int(Math.ceil(config.length/(2*hob.axialPitch)))+2;
    sweep=new HobbingSweep(hob,-span,span,config.tolerance);
    var distance=spec.pitchRadius()+hob.pitchDiameter/2;
    var half=Math.acos(Math.max(-1,Math.min(1,(distance-stock.grid.radiusMax)/(config.outside/2))))+Math.abs(hob.rake)+0.01;
    stream=new HobbingStream(stock,sweep,Math.min(Math.PI/2-1e-6,half));
    // HB10 assumed effective edge width; changing it requires force calibration.
    forces=new HobbingForces(stock,spec.pitchRadius(),spec.pressureAngle,1);
    stream.onPass=(pass,start,end)->{forces.afterPass(pass,start,end,config.outside/2);};
    preview=new RingPreviewWorker(stock);maxBytes=stock.bytes()+forces.retainedBytes();
    initialBand=ringGeometry(stock.contour());displayedBand=initialBand;
    var height=config.cupLength,thickness=config.cupEndThickness;
    if(height==null)cupBody=null;
    else {
      var cupHeight:Float=cast height,end:Float=thickness==null?2:cast thickness;
      if(!Math.isFinite(cupHeight)||end<=0||cupHeight<=config.width+end)throw "Invalid cup body dimensions";
      cupBody=HobbingCupDisplay.body(stock.grid.radiusMin,spec.rootRadius(),config.width,cupHeight,end,centerMetres);
    }
  }
  public function follow(time:Float,axes:HobbingAxes):Void {
    if(!streamStopped){
      if(axes.b<lastMaster-1e-8&&axes.x>stock.grid.radiusMax+config.outside/2){stream.finish();streamStopped=true;}
      else stream.push(time,axes);
      lastMaster=axes.b;
    }
    maxBytes=Math.max(maxBytes,stock.bytes()+forces.retainedBytes());
    // Independent circular fixture separating planes, evaluated at measured coordinates.
    if(axes.x-config.outside/2<spec.rootRadius()-config.wall-0.025)collisions++;
    var lowest=axes.z-Math.abs(axes.y*Math.sin(axes.a))-config.length/2*Math.abs(Math.sin(axes.a))-config.outside/2*Math.abs(Math.cos(axes.a));
    if(lowest < -55)collisions++;
  }
  public function finish(seconds:Float,sync:Float):Void {
    stream.finish();finalPreview=true;maxBytes=Math.max(maxBytes,stock.bytes()+forces.retainedBytes());
    try{
      // The coarse preview grid cannot resolve the generating root transition.
      // This is an explicitly limited diagnostic profile, never a full active-profile acceptance.
      var diagnostic=config.tolerance>=0.001;
      var inspection:Null<GearInspection> = diagnostic?new GearInspection(spec,spec.pitchRadius()):null;
      measurement=GearMeasurement.fromRings(stock,spec,inspection);
      if(diagnostic)measurement.warnings.push("Coarse app diagnostic: profile inspected from pitch to tip only; full active profile unchecked");
    }catch(e:Dynamic){measurementFailure=Std.string(e);}
    var m=measurement;
    report='Cycle ${Math.round(seconds*1000)/1000} s · sync ${Math.round(sync*648000/Math.PI*1000)/1000} arcsec · fixture contacts $collisions\n'
      +'${stream.passes} gash windows · ${stream.poses} poses · peak ${stream.peakFrames} buffered samples · ${Math.round(maxBytes/1024)} KiB stock';
    report+='\nAssumed Kienzle window loads: ${Math.round(forces.maxTangential)} N cutting, ${Math.round(forces.maxRadial)} N radial; '
      +'${Math.round(forces.maxPower)} W; B ${Math.round(forces.maxHobTorque*1000)/1000} Nm; '
      +'C bound ${Math.round(forces.maxTableTorqueBound*1000)/1000} Nm\n'+forces.material.basis
      +'; effective edge width 1 mm assumed; overlap summed, instantaneous peaks unresolved';
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
  public function geometry(liftMm:Float=0,blank:Bool=false):Null<GeometryData> {
    if(stream.passes!=previewPasses&&(finalPreview||stream.lastTime-lastPreviewTime>=0.1)&&preview.refresh()){previewPasses=stream.passes;lastPreviewTime=stream.lastTime;finalPreview=false;}
    var mesh=preview.take();
    if(mesh==null&&lastLift==liftMm&&lastBlank==blank)return null;
    if(mesh!=null)displayedBand=ringGeometry(mesh);
    lastLift=liftMm;lastBlank=blank;
    var band=blank?initialBand:displayedBand;
    return cupBody==null?HobbingCupDisplay.shifted(band,liftMm/1000):HobbingCupDisplay.combine(band,cupBody,centerMetres,spec.rootRadius(),config.width,liftMm);
  }
  function ringGeometry(mesh:stockkit.StockMesh):GeometryData {
    var positions=Bytes.alloc(mesh.vertexCount*12);
    for(i in 0...mesh.vertexCount)for(axis in 0...3)positions.setFloat(i*12+axis*4,mesh.positions.getFloat(i*12+axis*4)/1000-centerMetres[axis]);
    var g=new GeometryData();g.addStream(1,2,positions,mesh.vertexCount,12);g.addStream(2,2,mesh.normals,mesh.vertexCount,12);
    g.addStream(6,4,mesh.colors,mesh.vertexCount,4);g.setIndexBuffer(mesh.indices,mesh.triangleCount*3);
    g.setBounds(-stock.grid.radiusMax/1000-centerMetres[0],-stock.grid.radiusMax/1000-centerMetres[1],stock.grid.zMin/1000-centerMetres[2],
      stock.grid.radiusMax/1000-centerMetres[0],stock.grid.radiusMax/1000-centerMetres[1],stock.grid.zMax/1000-centerMetres[2]);return g;
  }
  public function dispose():Void{if(disposed)return;disposed=true;preview.dispose();forces.dispose();sweep.dispose();stock.dispose();}
}
