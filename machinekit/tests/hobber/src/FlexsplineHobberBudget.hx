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
    if(args.length!=3)throw "reference-budget INPUTS.json OUTPUT_DIRECTORY pilot|baseline|SOURCE|all";
    var inputs:Dynamic=haxe.Json.parse(sys.io.File.getContent(args[0]));
    var output=args[1],mode=args[2],machine=new FlexsplineHobber(),s=machine.spec,cam=FlexsplineHobberCam.job(machine),path=cam.passes[0];
    if(inputs.force.referenceForceAccepted!=true||inputs.force.forceToleranceValidated!=true)throw "HB10 accepted full-reference force inputs required";
    if(Math.abs(inputs.force.seconds-cam.cuttingSeconds)>1e-6||inputs.force.teeth!=s.teeth||inputs.force.moduleMm!=s.module||inputs.force.faceWidthMm!=s.faceWidth)
      throw "force history does not match reference CAM";
    var sync:Array<Array<Float>> = inputs.sync,force:Array<Array<Float>> = inputs.radial;
    for(rows in [sync,force]){if(rows.length<2||rows[0][0]>0||rows[rows.length-1][0]<cam.cuttingSeconds)throw "incomplete history";
      for(i in 0...rows.length){for(v in rows[i])if(!Math.isFinite(v))throw "non-finite history";
        if(i>0&&rows[i][0]<=rows[i-1][0])throw "history times must increase";}}
    if(inputs.masterCounts!=machine.spindle.encoder.count("countsPerRevolution"))throw "sync trace master encoder differs from installed part";
    var support=FlexsplineHobberChecks.installedSupport(machine);
    if(!support.supportsRim)throw "reference sleeve does not support tooth band";
    var wall=new RingCompliance(s.rootRadius()-0.3,0.6,s.faceWidth,210000,support.stiffness,"Assumed sleeve/contact estimate; supported rim checked; oil n=2 stiffness zero");
    var syncSource=HobbingErrorSource.measuredSync("HB7 electronic gearbox",t->lookup(sync,1,t),inputs.syncBasis);
    var sources=FlexsplineHobberErrors.sources(machine,t->Math.max(0,lookup(force,1,t)),wall,syncSource);
    var motion=new IdealHobbingJob(s,machine.hob.design,cam.rpm,path.revolutions,0.5,path.zStart);
    var profile=new ToothProfile(s),inspection=new GearInspection(s,profile.rootJoinRadius+0.01*(profile.tipJoinRadius-profile.rootJoinRadius));
    var radialStep=0.0015,axialStep=0.04,tolerance=0.000025,threads=8;
    var grid=RingStock.fromGearBlank(s,wall.wall,radialStep,axialStep),passes=motion.schedule(grid.grid);grid.dispose();
    var knots=[for(row in sync)row[0]].concat([for(row in force)row[0]]);
    var metadata={moduleMm:s.module,teeth:s.teeth,faceWidthMm:s.faceWidth,threads:threads,sweepToleranceMm:tolerance,pathToleranceMm:tolerance/16,
      radialStepMm:radialStep,axialStepMm:axialStep,windows:passes.length,sourceIds:[for(source in sources)source.id],
      forceSettings:inputs.force,forceInterpolation:inputs.forceInterpolation,syncBasis:inputs.syncBasis,
      supportStiffnessNPerMm:support.stiffness,scope:"complete HB8 feed stroke; fixed baseline HB10 load history; simulated HB7 servo, assumed plant; m0.3 ISO extrapolation only"};
    sys.io.File.saveContent(output+"/settings.json",haxe.Json.stringify(metadata));
    Sys.println("REFERENCE_BUDGET_SETTINGS "+haxe.Json.stringify(metadata));Sys.stdout().flush();
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
    var span=Std.int(Math.ceil(machine.hob.n("height")/(2*machine.hob.design.axialPitch)))+2;
    var job=new HobbingErrorJob(s,machine.hob.design,motion.axesAt,passes,sources,wall.wall,radialStep,axialStep,-span,span,tolerance,inspection,knots,threads);
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
