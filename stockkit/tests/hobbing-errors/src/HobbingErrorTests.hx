import gearkit.*;
import stockkit.hobbing.*;
import MetrologyTests.SyntheticGearRings;
class HobbingErrorTests {
  static var checks=0;
  static function check(ok:Bool,label:String):Void {checks++;if(!ok)throw label;}
  static function near(a:Float,b:Float,tol:Float,label:String):Void check(Math.abs(a-b)<=tol,'$label: $a / $b');
  static function physical():Void {
    var spec=new GearSpec(1,40,2,20*Math.PI/180),hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),1,32,4);
    var ideal=new IdealHobbingJob(spec,hob,3000,1.1,0,1);
    var stock=stockkit.RingStock.fromGearBlank(spec,0.6,0.05,0.5);
    var passes=ideal.schedule(stock.grid);stock.dispose();
    var sources=[HobbingErrorSource.workEccentricity("mandrel","mandrel",0.004),
      HobbingErrorSource.hobRadial("arbor",0.004),HobbingErrorSource.hobAxial("arbor",0.004)];
    var job=new HobbingErrorJob(spec,hob,ideal.axesAt,passes,sources,0.6,0.05,0.5,-12,12,0.000025);
    var budget=job.budget();
    Sys.println(budget.report());
    check(budget.contributions.length==3,"physical sweep per source and all-source inspection");
    var mandrel=budget.contributions.filter(row->row.source.id=="mandrel")[0].measurement;
    near(mandrel.value("Fr"),8,budget.baseline.value("Fr")+2,"physical eccentricity Fr=2e within baseline discretization plus 2 um");
    var axial=budget.contributions.filter(row->row.source.id=="hob-axial")[0].measurement;
    check(Math.abs(axial.value("ffalpha")-budget.baseline.value("ffalpha"))>0.00001,"axial hob error changes measured profile form");
    check(budget.additive,"physical small-error traces meet 25% + 0.1 um superposition criterion");
    Sys.println('Physical hobbing errors: $checks assertions passed');
  }
  public static function main():Void {
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
