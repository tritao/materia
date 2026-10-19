import gearkit.*;
import stockkit.*;
import stockkit.hobbing.*;
import machinekit.hobbing.MandrelRadialSupport;

/** Reproducible physical source studies. Ideal baseline, independent simulated
    HB7 sync residual, and frozen HB10 radial loads: neither is injected twice. */
class FlexsplineHobberBudget {
  static function lookup(rows:Array<Array<Float>>,column:Int,t:Float):Float {
    if(t<=rows[0][0])return rows[0][column];
    var last=rows.length-1;if(t>=rows[last][0])return rows[last][column];
    var lo=0,hi=last;while(hi-lo>1){var mid=(lo+hi)>>1;if(rows[mid][0]<=t)lo=mid;else hi=mid;}
    return rows[lo][column]+(rows[hi][column]-rows[lo][column])*(t-rows[lo][0])/(rows[hi][0]-rows[lo][0]);
  }
  static function measurement(m:GearMeasurement):Dynamic {
    var values:Dynamic={},grades:Dynamic={};for(key in m.values.keys())Reflect.setField(values,key,m.value(key));
    for(key in m.grades.keys())Reflect.setField(grades,key,m.grades.get(key));
    return {values:values,grades:grades,isoClass:m.isoClass,overallGrade:m.overallGrade,
      warnings:m.warnings,extrapolation:m.extrapolation,profileSourcePoints:m.profileSourcePoints,helixSourcePoints:m.helixSourcePoints,
      thicknessMm:m.thickness,profile:[for(t in m.profiles)t.filtered],lead:[for(t in m.helices)t.filtered],
      pitch:m.cumulativePitch,runoutMm:m.radialMeasurements};
  }
  public static function run(args:Array<String>):Void {
    if(args.length!=3)throw "reference-budget INPUTS.json OUTPUT_DIRECTORY pilot|engaged|baseline|SOURCE|all";
    var inputs:Dynamic=haxe.Json.parse(sys.io.File.getContent(args[0]));
    var output=args[1],mode=args[2],machine=new FlexsplineHobber(),s=machine.spec,cam=FlexsplineHobberCam.job(machine),path=cam.passes[0];
    if(inputs.force.referenceForceAccepted!=true||inputs.force.forceToleranceValidated!=true)throw "HB10 accepted full-reference force inputs required";
    if(Math.abs(inputs.force.seconds-cam.cuttingSeconds)>1e-6||inputs.force.teeth!=s.teeth||inputs.force.moduleMm!=s.module||inputs.force.faceWidthMm!=s.faceWidth)
      throw "force history does not match reference CAM";
    var sync:Array<Array<Float>> = inputs.sync,force:Array<Array<Float>> = inputs.radial;
    for(rows in [sync,force]){if(rows.length<2||rows[0][0]>0||rows[rows.length-1][0]<cam.cuttingSeconds)throw "incomplete history";
      for(i in 0...rows.length){for(v in rows[i])if(!Math.isFinite(v))throw "non-finite history";
        if(i>0&&rows[i][0]<=rows[i-1][0])throw "history times must increase";}}
    if(inputs.masterCounts!=machine.spindle.encoder.parameterCount("countsPerRevolution"))throw "sync trace master encoder differs from installed part";
    var support=FlexsplineHobberChecks.installedSupport(machine);
    if(!support.supportsRim)throw "reference sleeve does not support tooth band";
    var wall=new RingCompliance(s.rootRadius()-0.3,0.6,s.faceWidth,210000,support.stiffness,"Assumed sleeve/contact estimate; supported rim checked; oil n=2 stiffness zero");
    var syncSource=HobbingErrorSource.measuredSync("HB7 electronic gearbox",t->lookup(sync,1,t),inputs.syncBasis);
    var sources=FlexsplineHobberErrors.sources(machine,t->Math.max(0,lookup(force,1,t)),wall,syncSource);
    var motion=new IdealHobbingJob(s,machine.hob.design,cam.rpm,path.revolutions,0.5,path.zStart);
    var profile=new ToothProfile(s),inspection=new GearInspection(s,profile.rootJoinRadius+0.01*(profile.tipJoinRadius-profile.rootJoinRadius));
    var radialStep=0.0015,axialStep=0.04,tolerance=0.000025,threads=8;
    var configuredThreads=Sys.getEnv("HOBBER_BUDGET_THREADS");
    if(configuredThreads!=null){var parsed=Std.parseInt(configuredThreads);if(parsed==null||parsed<1||parsed>20)throw "threads must be 1..20";threads=parsed;}
    var faceEndpoints=Sys.getEnv("HOBBER_BUDGET_LEGACY_ENDS")!="1";
    var grid=RingStock.fromGearBlank(s,wall.wall,radialStep,axialStep),passes=motion.schedule(grid.grid);grid.dispose();
    var knots=[for(row in sync)row[0]].concat([for(row in force)row[0]]);
    var metadata={tableSignature:machine.tableSpec.signature(),tableFields:machine.tableSpec.report(),tableScope:"Installed rotary data; syncBasis identifies the independent assumed plant, not measured bought-table servo response",moduleMm:s.module,teeth:s.teeth,faceWidthMm:s.faceWidth,threads:threads,sweepToleranceMm:tolerance,pathToleranceMm:tolerance/16,
      radialStepMm:radialStep,axialStepMm:axialStep,windows:passes.length,sourceIds:[for(source in sources)source.id],
      forceSettings:inputs.force,forceInterpolation:inputs.forceInterpolation,syncBasis:inputs.syncBasis,
      supportStiffnessNPerMm:support.stiffness,scope:"complete HB8 feed stroke; fixed baseline HB10 load history; simulated HB7 servo, assumed plant; m0.3 ISO extrapolation only"};
    sys.io.File.saveContent(output+"/"+(mode=="all"?"settings":mode+"-settings")+".json",haxe.Json.stringify(metadata));
    Sys.println("REFERENCE_BUDGET_SETTINGS "+haxe.Json.stringify(metadata));Sys.stdout().flush();
    if(mode=="geometry-speed"){
      var reference=InspectionRingStock.create(new RingGrid(s.rootRadius()-wall.wall,s.tipRadius(),0,s.faceWidth,radialStep,axialStep),inspection);
      var candidate=InspectionRingStock.create(new RingGrid(s.rootRadius()-wall.wall,s.tipRadius(),0,s.faceWidth,radialStep,axialStep),inspection);
      if(reference==null||candidate==null)throw "reference inspection domains unavailable";
      var a=new FaceEndpointStock(reference,reference.stocks,reference.grid,s.faceWidth),b=new FaceEndpointStock(candidate,candidate.stocks,candidate.grid,s.faceWidth);
      for(part in a.stocks.concat(b.stocks))part.setThreads(threads);
      var previous=Sys.getEnv("STOCKKIT_RAKE_BATCH_DOMAINS");
      Sys.putEnv("STOCKKIT_RAKE_BATCH_DOMAINS","1");
      var span=Std.int(Math.ceil(machine.hob.parameterNumber("height")/(2*machine.hob.design.axialPitch)))+2;
      var seed=new HobbingSweep(machine.hob.design,-span,span,0.002),first=30000,count=100,started=Sys.time();
      for(pass in passes.slice(first-2400,first))seed.cutPassMany(a.stocks,motion.axesAt,pass,0.002/16);seed.dispose();
      for(i in 0...a.stocks.length){var saved=a.stocks[i].snapshot();b.stocks[i].restore(saved);saved.dispose();}
      Sys.println("GEOMETRY_SEED_SECONDS "+(Sys.time()-started));Sys.stdout().flush();
      Sys.putEnv("STOCKKIT_RAKE_BATCH_DOMAINS","0");
      var baselineSweep=new HobbingSweep(machine.hob.design,-span,span,tolerance);
      Sys.putEnv("STOCKKIT_RAKE_BATCH_DOMAINS","1");var candidateSweep=new HobbingSweep(machine.hob.design,-span,span,tolerance);
      Sys.putEnv("STOCKKIT_RAKE_BATCH_DOMAINS",previous==null?"":previous);
      var sorted:Array<Float> = [],i=0,j=0;
      while(i<sync.length||j<force.length){var t=j>=force.length||(i<sync.length&&sync[i][0]<force[j][0])?sync[i++][0]:force[j++][0];
        if(sorted.length==0||t>sorted[sorted.length-1])sorted.push(t);}
      var baselineSeconds=0.0,candidateSeconds=0.0,baselinePoses=0,candidatePoses=0,index=0;
      for(pass in passes.slice(first,first+count)){
        var lo=0,hi=sorted.length;while(lo<hi){var mid=(lo+hi)>>1;if(sorted[mid]<=pass.start)lo=mid+1;else hi=mid;}
        var local:Array<Float> = [];while(lo<sorted.length&&sorted[lo]<pass.end)local.push(sorted[lo++]);
        function baseline():Void {var begin=Sys.time();baselinePoses+=baselineSweep.cutPassMany(a.stocks,motion.axesAt,pass,tolerance/16,0.001,local);baselineSeconds+=Sys.time()-begin;}
        function trial():Void {var begin=Sys.time();candidatePoses+=candidateSweep.cutPassMany(b.stocks,motion.axesAt,pass,tolerance/16,0.001,local);candidateSeconds+=Sys.time()-begin;}
        if(index++%2==0){baseline();trial();}else{trial();baseline();}
      }
      var exact=true,maxArc=0.0;
      for(i in 0...a.stocks.length){var left=a.stocks[i],right=b.stocks[i],difference=left.compareStock(right);
        for(z in 0...left.grid.axialCount)for(r in 0...left.grid.radialCount){var k=z*left.grid.radialCount+r;
          maxArc=Math.max(maxArc,(difference.leftover[k]+difference.gouge[k])*left.grid.radius(r));
          var aa=left.circle(r,z),bb=right.circle(r,z);if(aa.length!=bb.length){exact=false;continue;}
          for(k in 0...aa.length)if(aa[k].lo!=bb[k].lo||aa[k].hi!=bb[k].hi||aa[k].loGash!=bb[k].loGash||aa[k].hiGash!=bb[k].hiGash||aa[k].loPass!=bb[k].loPass||aa[k].hiPass!=bb[k].hiPass)exact=false;}}
      reference.assertOverlap();candidate.assertOverlap();
      if(!exact||maxArc!=0)throw "shared geometry changes sampled stock/provenance";
      var report={mode:mode,firstWindow:first,windows:count,threads:threads,sweepToleranceMm:tolerance,pathToleranceMm:tolerance/16,
        radialStepMm:radialStep,axialStepMm:axialStep,faceEndpoints:true,baselineSeconds:baselineSeconds,candidateSeconds:candidateSeconds,
        speedup:baselineSeconds/candidateSeconds,baselinePoses:baselinePoses,candidatePoses:candidatePoses,exactEndpointsAndProvenance:exact,
        maxArcDifferenceMm:maxArc,
        scope:"paired interleaved engaged reference-geometry fixture with common coarse-precut seed; all inspected domains and physical ends; not full-stroke acceptance"};
      sys.io.File.saveContent(output+"/"+mode+".json",haxe.Json.stringify(report));Sys.println("GEOMETRY_SPEED "+haxe.Json.stringify(report));
      baselineSweep.dispose();candidateSweep.dispose();for(part in a.stocks.concat(b.stocks))part.dispose();return;
    }
    if(mode=="lead-ends"||StringTools.startsWith(mode,"lead-")){
      // Only the two radius samples needed by lead/pitch metrology. Complete
      // stroke and source physics remain unchanged; profile is intentionally absent.
      var blank=RingStock.fromGearBlank(s,wall.wall,radialStep,axialStep);
      var cross=InspectionRingStock.create(blank.grid,inspection);
      if(cross==null){blank.dispose();throw "exact reference lead sampling unavailable";}
      var leadGrid=cross.stocks[1].grid;cross.dispose();blank.dispose();
      var span=Std.int(Math.ceil(machine.hob.parameterNumber("height")/(2*machine.hob.design.axialPitch)))+2;
      var endJob=new HobbingErrorJob(s,machine.hob.design,motion.axesAt,passes,sources,wall.wall,radialStep,axialStep,-span,span,tolerance,inspection,knots,threads,false,true,leadGrid);
      if(mode!="lead-ends"){
        var label=mode.substr(5),ids=label=="baseline"?[]:label=="combined"?[for(source in sources)source.id]:[label];
        var lead=endJob.run(ids);sys.io.File.saveContent(output+"/lead-"+label+".json",haxe.Json.stringify(measurement(lead)));return;
      }
      var base=endJob.run([]);sys.io.File.saveContent(output+"/face-end-baseline.json",haxe.Json.stringify(measurement(base)));
      var combined=endJob.run([for(source in sources)source.id]);sys.io.File.saveContent(output+"/face-end-combined.json",haxe.Json.stringify(measurement(combined)));
      for(m in [base,combined])if(m.warnings.indexOf("Face endpoints not sampled; end excess material unchecked")>=0)throw "face-end coverage was not measured";
      var leadKeys=["Fbeta","ffbeta","fHbeta"],failed=[for(key in leadKeys)if(combined.value(key)>Iso1328.tolerance(s,key,6))key];
      sys.io.File.saveContent(output+"/face-end-result.json",haxe.Json.stringify({baseline:measurement(base),combined:measurement(combined),failedClass6LeadParameters:failed,
        threads:threads,sweepToleranceMm:tolerance,pathToleranceMm:tolerance/16,endpointZMm:[0.0,s.faceWidth],scope:"complete HB8 stroke; lead-only baseline and combined physical inspections with exact face-end samples; profile budget and per-source end superposition are separate"}));
      Sys.println("FACE_END_ACCEPTANCE "+haxe.Json.stringify({failedClass6LeadParameters:failed,baselineFbetaUm:base.value("Fbeta"),combinedFbetaUm:combined.value("Fbeta"),scope:"lead-only physical face-end coverage; no profile regrading"}));return;
    }
    if(mode=="engaged"||mode=="engaged-cross"){
      // Common pre-cut stock for a performance/equality fixture. This is NOT
      // a substitute for the complete HB8 history or a metrology acceptance.
      var stock=RingStock.fromGearBlank(s,wall.wall,radialStep,axialStep);stock.setThreads(threads);
      var span=Std.int(Math.ceil(machine.hob.parameterNumber("height")/(2*machine.hob.design.axialPitch)))+2;
      var seed=new HobbingSweep(machine.hob.design,-span,span,0.002);
      var cross=mode=="engaged-cross"?InspectionRingStock.create(stock.grid,inspection):null;
      if(mode=="engaged-cross"&&cross==null)throw "reference grid has no exact inspection cross";
      var domains=cross==null?[stock]:[stock].concat(cross.stocks);
      if(cross!=null)for(part in cross.stocks)part.setThreads(threads);
      var first=8400,count=100,started=Sys.time();
      try{for(pass in passes.slice(first-2400,first))seed.cutPassMany(domains,motion.axesAt,pass,0.002/16);
      }catch(e:Dynamic){seed.dispose();stock.dispose();if(cross!=null)cross.dispose();throw e;}seed.dispose();
      Sys.println("ENGAGED_SEED_SECONDS "+(Sys.time()-started));Sys.stdout().flush();
      var sorted:Array<Float> = [],i=0,j=0;
      while(i<sync.length||j<force.length){
        var t=j>=force.length||(i<sync.length&&sync[i][0]<force[j][0])?sync[i++][0]:force[j++][0];
        if(sorted.length==0||t>sorted[sorted.length-1])sorted.push(t);
      }
      var sweep=new HobbingSweep(machine.hob.design,-span,span,tolerance);started=Sys.time();var denseSeconds=0.0,crossSeconds=0.0;
      try{for(pass in passes.slice(first,first+count)){
        var lo=0,hi=sorted.length;while(lo<hi){var mid=(lo+hi)>>1;if(sorted[mid]<=pass.start)lo=mid+1;else hi=mid;}
        var local:Array<Float> = [];while(lo<sorted.length&&sorted[lo]<pass.end){local.push(sorted[lo]);lo++;}
        var began=Sys.time();sweep.cutPass(stock,motion.axesAt,pass,tolerance/16,0.001,local);denseSeconds+=Sys.time()-began;
        if(cross!=null){began=Sys.time();sweep.cutPassMany(cross.stocks,motion.axesAt,pass,tolerance/16,0.001,local);crossSeconds+=Sys.time()-began;}
      }}catch(e:Dynamic){sweep.dispose();stock.dispose();if(cross!=null)cross.dispose();throw e;}
      var elapsed=Sys.time()-started,hashes=new StringBuf();
      for(r in 0...stock.grid.radialCount){var row=new StringBuf();
        for(z in 0...stock.grid.axialCount){if(cross!=null&&!cross.contains(r,z))continue;
          var circles=stock.circle(r,z);
          if(cross!=null){var other=cross.circle(r,z);if(circles.length!=other.length)throw "reference inspection cross topology differs";
            for(k in 0...circles.length){var a=circles[k],b=other[k];
              if(a.lo!=b.lo||a.hi!=b.hi||a.loGash!=b.loGash||a.hiGash!=b.hiGash||a.loPass!=b.loPass||a.hiPass!=b.hiPass)throw "reference inspection cross endpoint/provenance differs";}}
          row.add('$z:${circles.length};');
          for(a in circles)row.add('${a.lo},${a.hi},${a.loGash},${a.loPass},${a.hiGash},${a.hiPass};');}
        hashes.add(haxe.crypto.Sha256.encode(row.toString()));
      }
      var report={firstWindow:first,windows:count,wallSeconds:elapsed,threads:threads,sweepToleranceMm:tolerance,pathToleranceMm:tolerance/16,
        radialStepMm:radialStep,axialStepMm:axialStep,denseSeconds:denseSeconds,crossSeconds:crossSeconds,crossExact:cross!=null,hashScope:cross==null?"all circles":"all inspected circles verified against dense stock",stockHash:haxe.crypto.Sha256.encode(hashes.toString()),intervalCount:stock.intervalCount(),
        scope:"common-precut entry band on full fine grid; all endpoints/provenance hashed; performance/equality fixture, not complete-stroke acceptance"};
      sys.io.File.saveContent(output+"/engaged.json",haxe.Json.stringify(report));Sys.println("ENGAGED_RESULT "+haxe.Json.stringify(report));
      if(cross!=null){cross.assertOverlap();cross.dispose();}sweep.dispose();stock.dispose();return;
    }
    if(mode=="pilot"){
      // First 100 actual full-stroke windows on the SAME fine grid, no inspection
      // or source acceptance inferred from incomplete tooth coverage.
      var stock=RingStock.fromGearBlank(s,wall.wall,radialStep,axialStep);stock.setThreads(threads);
      var sweep=new HobbingSweep(machine.hob.design,-24,24,tolerance),started=Sys.time();
      var pilotKnots=[for(t in knots)if(t>=0&&t<=passes[99].end)t];
      try{for(pass in passes.slice(0,100))sweep.cutPass(stock,motion.axesAt,pass,tolerance/16,0.001,pilotKnots);
        Sys.println("REFERENCE_BUDGET_PILOT "+haxe.Json.stringify({windows:100,totalWindows:passes.length,wallSeconds:Sys.time()-started,scope:"timing only; entry windows; no acceptance or full-run extrapolation"}));
      }catch(e:Dynamic){sweep.dispose();stock.dispose();throw e;}sweep.dispose();stock.dispose();return;
    }
    var span=Std.int(Math.ceil(machine.hob.parameterNumber("height")/(2*machine.hob.design.axialPitch)))+2;
    var useCross=Sys.getEnv("HOBBER_BUDGET_DENSE")!="1";
    var job=new HobbingErrorJob(s,machine.hob.design,motion.axesAt,passes,sources,wall.wall,radialStep,axialStep,-span,span,tolerance,inspection,knots,threads,useCross,faceEndpoints);
    function runCut(ids:Array<String>):GearMeasurement {
      var label=ids.length==0?"baseline":ids.length==1?ids[0]:"combined",measured=job.run(ids);
      sys.io.File.saveContent(output+"/"+label+".json",haxe.Json.stringify(measurement(measured)));
      Sys.stdout().flush();return measured;
    }
    if(mode!="all"){runCut(mode=="baseline"?[]:mode=="combined"?[for(source in sources)source.id]:[mode]);return;}
    var budget=new ErrorBudget([for(source in sources){id:source.id,part:source.part,basis:source.basis}],runCut);
    sys.io.File.saveContent(output+"/budget.txt",budget.report());
    var dominant:Dynamic={},interaction:Dynamic={};for(k in budget.dominant.keys())Reflect.setField(dominant,k,budget.dominant.get(k));
    for(k in budget.interaction.keys())Reflect.setField(interaction,k,budget.interaction.get(k));
    var failures=[for(k in Iso1328.parameters)if(budget.combined.value(k)>Iso1328.tolerance(s,k,6))k];
    sys.io.File.saveContent(output+"/result.json",haxe.Json.stringify({additive:budget.additive,dominant:dominant,interactionUm:interaction,
      warnings:budget.warnings,failedClass6Parameters:failures,combined:measurement(budget.combined),
      scope:metadata.scope,acceptance:failures.length==0?"extrapolated class-6 parameter limits fit; qualification warnings and assumptions remain":"open: combined parameters exceed class-6 limits"}));
    Sys.println(budget.report());
  }
}
