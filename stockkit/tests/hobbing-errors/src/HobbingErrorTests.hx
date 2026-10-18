import gearkit.*;
import stockkit.hobbing.*;
import MetrologyTests.SyntheticGearRings;
class HobbingErrorTests {
  static var checks=0;
  static function check(ok:Bool,label:String):Void {checks++;if(!ok)throw label;}
  static function near(a:Float,b:Float,tol:Float,label:String):Void check(Math.abs(a-b)<=tol,'$label: $a / $b');
  static function physical(tolerance:Float=0.0001,threads:Int=4):Void {
    var spec=new GearSpec(1,20,2,20*Math.PI/180),hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),1,32,4);
    var ideal=new IdealHobbingJob(spec,hob,3000,1.1,0,1);
    var stock=stockkit.RingStock.fromGearBlank(spec,0.6,0.1,0.2);
    var passes=ideal.schedule(stock.grid);stock.dispose();
    var sources=[HobbingErrorSource.workEccentricity("mandrel","mandrel",0.004),
      HobbingErrorSource.hobRadial("arbor",0.004),HobbingErrorSource.hobAxial("arbor",0.004)];
    var job=new HobbingErrorJob(spec,hob,ideal.axesAt,passes,sources,0.6,0.1,0.2,-6,6,tolerance,new GearInspection(spec,spec.pitchRadius()),null,threads);
    var budget=job.budget();
    Sys.println(budget.report());
    check(budget.contributions.length==3,"physical sweep per source and all-source inspection");
    var mandrel=budget.contributions.filter(row->row.source.id=="mandrel")[0].measurement;
    near(mandrel.value("Fr"),8,budget.baseline.value("Fr")+2,"physical eccentricity Fr=2e within baseline discretization plus 2 um");
    var axial=budget.contributions.filter(row->row.source.id=="hob-axial")[0].measurement;
    check(Math.abs(axial.value("ffalpha")-budget.baseline.value("ffalpha"))>0.00001,"axial hob error changes measured profile form");
    check(budget.additive,"physical small-error traces meet 25% + 0.1 um superposition criterion");
    Sys.println("QUICK_ERRORS "+haxe.Json.stringify({threads:threads,sweepToleranceMm:tolerance,pathToleranceMm:tolerance/16,runoutUm:budget.combined.value("Fr"),pitchUm:budget.combined.value("Fp"),
      profileFormUm:budget.combined.value("ffalpha"),additive:budget.additive,scope:"m1 z20 face2 coarse physical source budget; no ISO qualification"}));
    Sys.println('Physical hobbing errors: $checks assertions passed');
  }
  static function knotParity():Void {
    var spec=new GearSpec(1,20,2,20*Math.PI/180),hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),1,32,4);
    var ideal=new IdealHobbingJob(spec,hob,3000,1.1,0,1),inspection=new GearInspection(spec,spec.pitchRadius());
    var knots=[1.0,0.23,-1,0.17,0.23,0.0,ideal.duration];
    var source=HobbingErrorSource.measuredSync("test runtime",t->t<0.17?t*0.0001:t<0.23?0.000017-(t-0.17)*0.0002:0.000005,"piecewise linear measured residual with exact knots");
    var stock=stockkit.RingStock.fromGearBlank(spec,0.6,0.1,0.2);stock.setThreads(4);
    var passes=ideal.schedule(stock.grid),sweep=new HobbingSweep(hob,-6,6,0.0001);
    var direct:GearMeasurement;
    try{for(pass in passes)sweep.cutPass(stock,t->source.apply(t,ideal.axesAt(t)),pass,0.0001/16,0.001,knots);
      direct=GearMeasurement.fromRings(stock,spec,inspection);
    }catch(e:Dynamic){sweep.dispose();stock.dispose();throw e;}sweep.dispose();stock.dispose();
    var job=new HobbingErrorJob(spec,hob,ideal.axesAt,passes,[source],0.6,0.1,0.2,-6,6,0.0001,inspection,knots,4);
    var localized=job.run([source.id]);
    for(k in direct.values.keys())check(direct.value(k)==localized.value(k),'localized motion knots preserve $k exactly');
    for(pair in [{a:direct.profiles,b:localized.profiles},{a:direct.helices,b:localized.helices}])
      for(i in 0...pair.a.length)for(j in 0...pair.a[i].filtered.length)
        if(pair.a[i].filtered[j]!=pair.b[i].filtered[j])throw "localized knots change a measured trace";
    check(true,"localized unsorted/duplicate/outside knots preserve every profile and lead trace");
    Sys.println('Motion knot parity: $checks assertions passed');
  }
  static function crossParity():Void {
    var spec=new GearSpec(1,20,2,20*Math.PI/180),hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),1,32,4);
    var motion=new IdealHobbingJob(spec,hob,3000,1.1,0,1),cfg=new GearInspection(spec,spec.pitchRadius());
    var blank=stockkit.RingStock.fromGearBlank(spec,0.6,0.1,0.2),cross=InspectionRingStock.create(blank.grid,cfg);
    check(cross!=null,"exact inspection cross exists for physical source fixture");
    var rejected=false;try{cross.boundaries(0,0);}catch(e:Dynamic){rejected=true;}
    check(rejected,"uncomputed circles cannot be read as cut geometry");cross.dispose();
    var passes=motion.schedule(blank.grid);blank.dispose();
    var sources=[HobbingErrorSource.workEccentricity("mandrel","mandrel",0.004),
      HobbingErrorSource.hobRadial("arbor",0.004),HobbingErrorSource.hobAxial("arbor",0.004),
      HobbingErrorSource.profile("hob",0.001,hob.axialPitch)];
    var scenarios:Array<Array<String>> = [[],[for(source in sources)source.id]];
    for(enabled in scenarios){
      var dense=new HobbingErrorJob(spec,hob,motion.axesAt,passes,sources,0.6,0.1,0.2,-6,6,0.000025,cfg,null,4).run(enabled);
      var sparse=new HobbingErrorJob(spec,hob,motion.axesAt,passes,sources,0.6,0.1,0.2,-6,6,0.000025,cfg,null,4,true).run(enabled);
      for(k in dense.values.keys())check(dense.value(k)==sparse.value(k),'inspection cross preserves $k exactly');
      check(dense.overallGrade==sparse.overallGrade&&dense.isoClass==sparse.isoClass,"same grading qualification");
      check(dense.warnings.join(";")==sparse.warnings.join(";"),"same coverage/filter warnings");
      for(pair in [{a:dense.profiles,b:sparse.profiles},{a:dense.helices,b:sparse.helices}])
        for(i in 0...pair.a.length)for(j in 0...pair.a[i].filtered.length)
          if(pair.a[i].filtered[j]!=pair.b[i].filtered[j])throw "inspection cross changes a measured trace";
      for(pair in [{a:dense.thickness,b:sparse.thickness},{a:dense.radialMeasurements,b:sparse.radialMeasurements}])
        for(i in 0...pair.a.length)if(pair.a[i]!=pair.b[i])throw "inspection cross changes thickness or runout";
      check(true,"all profile/lead/thickness/runout traces identical, baseline and combined warped cutter");
    }
    Sys.println('Inspection cross parity: $checks assertions passed');
  }
  static function faceEnds():Void {
    var spec=new GearSpec(1,20,2,20*Math.PI/180),hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),1,32,4);
    var motion=new IdealHobbingJob(spec,hob,3000,1.1,0,1),cfg=new GearInspection(spec,spec.pitchRadius());
    var reports:Array<GearMeasurement> = [],legacy:Array<GearMeasurement> = [];
    for(excess in [false,true]){
      var interior=stockkit.RingStock.fromGearBlank(spec,0.6,0.1,0.2);
      var covered=new FaceEndpointStock(interior,[interior],interior.grid,spec.faceWidth);
      for(part in covered.stocks)part.setThreads(4);
      var positions=covered.axialPositions();check(positions[0]==0&&positions[positions.length-1]==2,"face endpoints are exact measured planes");
      var sweep=new HobbingSweep(hob,-6,6,0.000025),passes=motion.schedule(interior.grid);
      function endMotion(t:Float):HobbingAxes {var a=motion.axesAt(t);return new HobbingAxes(a.x+0.02,a.y,a.z,a.a,a.b,a.c,a.offset);}
      try{for(pass in passes){
        if(excess){sweep.cutPassMany(covered.stocks.slice(0,covered.stocks.length-1),motion.axesAt,pass,0.000025/16);
          sweep.cutPass(covered.stocks[covered.stocks.length-1],endMotion,pass,0.000025/16);}
        else sweep.cutPassMany(covered.stocks,motion.axesAt,pass,0.000025/16);
      }
      legacy.push(GearMeasurement.fromRings(interior,spec,cfg));
      var report=GearMeasurement.fromRings(covered,spec,cfg);reports.push(report);
      check(report.warnings.indexOf("Face endpoints not sampled; end excess material unchecked")<0,"real endpoint cuts resolve face coverage warning");
      }catch(e:Dynamic){sweep.dispose();for(part in covered.stocks)part.dispose();throw e;}
      sweep.dispose();for(part in covered.stocks)part.dispose();
    }
    Sys.println('FACE_END_FIXTURE '+haxe.Json.stringify({baselineFbeta:reports[0].value("Fbeta"),excessFbeta:reports[1].value("Fbeta"),baselineForm:reports[0].value("ffbeta"),excessForm:reports[1].value("ffbeta")}));Sys.stdout().flush();
    check(reports[1].value("Fbeta")>reports[0].value("Fbeta")+0.25*20*Math.sin(spec.pressureAngle),"physical 20 um radial recession at one end exposes excess material in lead inspection");
    near(legacy[1].value("Fbeta"),legacy[0].value("Fbeta"),1e-12,"interior-only inspection misses the injected end-only excess");
    check(legacy[0].warnings.indexOf("Face endpoints not sampled; end excess material unchecked")>=0,"legacy inset inspection retains its coverage warning");
    near(reports[1].value("Fp"),reports[0].value("Fp"),1e-12,"face-end excess leaves interior pitch measurement unchanged");
    near(reports[1].value("Falpha"),reports[0].value("Falpha"),1e-12,"face-end excess leaves interior profile measurement unchanged");
    var blank=stockkit.RingStock.fromGearBlank(spec,0.6,0.1,0.2),passes=motion.schedule(blank.grid);
    var cross=InspectionRingStock.create(blank.grid,cfg);if(cross==null)throw "exact radial strip for focused end check";
    var leadGrid=cross.stocks[1].grid;cross.dispose();blank.dispose();
    var focused=new HobbingErrorJob(spec,hob,motion.axesAt,passes,[],0.6,0.1,0.2,-6,6,0.000025,cfg,null,4,false,true,leadGrid).run([]);
    for(key in ["fp","Fp","Fr","Fbeta","ffbeta","fHbeta"])near(focused.value(key),reports[0].value(key),1e-12,'focused lead/end domain preserves $key');
    check(focused.warnings.indexOf("Face endpoints not sampled; end excess material unchecked")<0,"focused domain has actual face-end coverage");
    var sparse=new HobbingErrorJob(spec,hob,motion.axesAt,passes,[],0.6,0.1,0.2,-6,6,0.000025,cfg,null,4,true,true).run([]);
    for(key in reports[0].values.keys())near(sparse.value(key),reports[0].value(key),1e-12,'sparse end planes preserve $key');
    Sys.println('Face-end inspection: $checks assertions passed; baseline Fbeta=${reports[0].value("Fbeta")} um; end-excess Fbeta=${reports[1].value("Fbeta")} um');
  }
  static function multiConvergence():Void {
    var spec=new GearSpec(1,20,2,20*Math.PI/180),hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),1,32,4);
    var a=stockkit.RingStock.annulus(new stockkit.RingGrid(spec.pitchRadius()-0.1,spec.pitchRadius()+0.1,0.9,1.1,0.1,0.1)),b=stockkit.RingStock.annulus(a.grid),dense=stockkit.RingStock.annulus(a.grid);
    a.setThreads(1);b.setThreads(4);dense.setThreads(4);
    var motion=new IdealHobbingJob(spec,hob,3000,1.1,0,1),passes=motion.schedule(a.grid),sweep=new HobbingSweep(hob,-6,6,0.00005);
    var one=sweep.cut(dense,motion.axesAt,passes),many=sweep.cutMany([a,b],motion.axesAt,passes);
    check(one.refinement==many.refinement&&one.maxArcChange==many.maxArcChange,"shared-domain convergence matches existing dense sweep");
    for(part in [a,b]){var comparison=part.compareStock(dense);
      for(v in comparison.leftover)if(v!=0)throw "multi-domain stock differs";
      for(v in comparison.gouge)if(v!=0)throw "multi-domain stock differs";}
    check(true,"one/four threads and shared poses preserve converged stock exactly");
    sweep.dispose();a.dispose();b.dispose();dense.dispose();
    Sys.println('Multi-domain convergence: $checks assertions passed');
  }
  public static function main():Void {
    if(Sys.args().indexOf("multi-convergence")>=0){multiConvergence();return;}
    if(Sys.args().indexOf("face-ends")>=0){faceEnds();return;}
    if(Sys.args().indexOf("cross-parity")>=0){crossParity();return;}
    if(Sys.args().indexOf("knot-parity")>=0){knotParity();return;}
    if(Sys.args().indexOf("physical-fine")>=0){physical(0.000025,8);return;}
    if(Sys.args().indexOf("physical")>=0){physical();return;}
    var axis=new HobbingAxes(46,0,4,0,0,0),e=0.004;
    var mandrel=HobbingErrorSource.workEccentricity("mandrel","mandrel",e);
    var p=mandrel.apply(0,axis).pose();near(p.get_translation(0),46-e,1e-12,"work eccentricity uses work frame");
    var hob=HobbingErrorSource.hobRadial("arbor",0.004);
    var a=hob.apply(0,axis).pose(),b=hob.apply(0,new HobbingAxes(46,0,4,0,Math.PI,0)).pose();
    near(a.get_translation(0),46-0.002,1e-12,"radial hob offset first phase");
    near(b.get_translation(0),46+0.002,1e-12,"radial hob offset half turn");
    var c=hob.apply(0,new HobbingAxes(46,0,4,0,2*Math.PI,0)).pose();
    near(c.get_translation(0),a.get_translation(0),1e-12,"hob error repeats each B revolution");
    near(hob.apply(0,new HobbingAxes(46,0,4,0,Math.PI/2,0)).pose().get_translation(2),3.998,1e-12,"hob eccentricity rotates in its radial plane");
    var axial=HobbingErrorSource.hobAxial("arbor",0.004);
    near(axial.apply(0,axis).pose().get_translation(1),-0.002,1e-12,"axial hob error follows tool axis");
    var encoder=HobbingErrorSource.encoder("encoder",0.001,60);
    var q=new HobbingAxes(46,0,4,0,0,Math.PI/2);
    near(encoder.apply(0,q).c-q.c,-0.001/60,1e-12,"encoder eccentricity first harmonic");
    var lag=HobbingErrorSource.latency("driver",0.001,1/200,t->100*t,t->100,false);
    near(lag.apply(1,axis).c,-0.0005,1e-12,"constant-speed latency phase offset");
    var predicted=HobbingErrorSource.latency("driver",0.001,1/200,t->100*t,t->100,true);
    near(predicted.apply(1,axis).c,0,1e-12,"constant-speed latency prediction");
    var ramp=HobbingErrorSource.latency("driver",0.001,1/200,t->100*t*t,t->200*t,false);
    check(Math.abs(ramp.apply(1,axis).c-ramp.apply(0.5,axis).c)>0.0004,"varying speed produces varying sync error");
    near(HobbingErrorSource.wall("blank",1000,t->20).apply(0,axis).x-axis.x,0.02,1e-12,"wall recession uses supported stiffness");
    var ring=new RingCompliance(30,0.6,8,210000,0,"unsupported diagnostic");
    var thick=new RingCompliance(30,1.2,8,210000,0,"unsupported diagnostic");
    near(thick.stiffness/ring.stiffness,8,1e-12,"ring bending stiffness scales with wall cubed");
    var supported=new RingCompliance(30,0.6,8,210000,1000,"explicit test support");
    check(supported.deflection(20)<ring.deflection(20),"mandrel support reduces ring deflection");
    near(supported.deflection(40),2*supported.deflection(20),1e-12,"linear quasistatic force response");
    var shape=HobbingErrorSource.profile("hob",0.001,1),warp=shape.warp;
    if(warp==null)throw "missing profile waveform";
    near(warp(new Point3(16,0,0.25)).x,16.001,1e-12,"profile warp modifies rake geometry before cutting");
    var toolSpec=new GearSpec(1,40,4,20*Math.PI/180),design=new HobDesign(ConjugateRack.of(new ToothProfile(toolSpec)),1,32,4);
    var nativeProfile=new HobbingSweep(design,-2,2,0.00005,warp);nativeProfile.dispose();
    check(true,"profile-distorted rake regions accepted by native tool builder");
    near(HobbingErrorSource.measuredSync("driver",t->0.0002,"test measured samples").apply(0,axis).c,0.0002,1e-12,"measured runtime sync is injected without an ideal reconstruction");
    // Independent translated involutes test HB4's expected eccentricity signature.
    var spec=new GearSpec(1,40,4,20*Math.PI/180),profile=new ToothProfile(spec);
    var control=profile.rootJoinRadius+0.01*(profile.tipJoinRadius-profile.rootJoinRadius);
    var inspection=new GearInspection(spec,control,null,null,null,null,0,0);
    var measured=GearMeasurement.fromRings(new SyntheticGearRings(spec,control,e,0,0,65,65),spec,inspection);
    near(measured.value("Fr"),2*e*1000,0.1,"translated involutes Fr=2e");
    var cs=0.0,sn=0.0;
    for(i in 0...spec.teeth){var v=measured.cumulativePitch[0][i];cs+=2*v*Math.cos(2*Math.PI*i/spec.teeth)/spec.teeth;sn+=2*v*Math.sin(2*Math.PI*i/spec.teeth)/spec.teeth;}
    near(Math.sqrt(cs*cs+sn*sn),e*1000*spec.pitchRadius()/spec.baseRadius(),0.02,"eccentric pitch first harmonic");
    var budget=new ErrorBudget([{id:"mandrel",part:"mandrel",basis:"test eccentricity"},{id:"lead",part:"test support",basis:"test twist"}],ids->{
      return GearMeasurement.fromRings(new SyntheticGearRings(spec,control,ids.indexOf("mandrel")>=0?e:0,0,ids.indexOf("lead")>=0?2:0,65,65),spec,inspection);
    });
    check(budget.contributions.length==2&&budget.additive,"small-error trace superposition within 25% + 0.1 um");
    check(budget.dominant.get("Fr")=="mandrel","budget names dominant part");
    check(budget.report().indexOf("test eccentricity")>=0,"report retains assumptions");
    var nonlinear=new ErrorBudget([{id:"a",part:"fixture",basis:"test"},{id:"b",part:"fixture",basis:"test"}],ids->{
      var twist=ids.length+(ids.length==2?3:0);
      return GearMeasurement.fromRings(new SyntheticGearRings(spec,control,0,0,twist,65,65),spec,inspection);
    });
    check(!nonlinear.additive&&nonlinear.warnings.length>0,"budget flags nonlinear interaction rather than summing grades");
    var interpolated=new HobbingMotion([new HobbingKeyframe(0,axis),new HobbingKeyframe(1,mandrel.apply(1,new HobbingAxes(46,0,4,0,1,0)))]);
    near(interpolated.at(0.5).pose().get_translation(0),46-e/2,1e-12,"recorded pose offsets survive interpolation");
    Sys.println('Hobbing errors: $checks assertions passed');
  }
}
