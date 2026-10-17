import gearkit.GearSpec;
import gearkit.ToothProfile;
import gearkit.ConjugateRack;
import gearkit.HobDesign;
import stockkit.RingGrid;
import stockkit.RingStock;
import stockkit.hobbing.HobbingAxes;
import stockkit.hobbing.HobbingKeyframe;
import stockkit.hobbing.HobbingMotion;
import stockkit.hobbing.HobbingSweep;
import stockkit.hobbing.IdealHobbingJob;
import stockkit.hobbing.GashPass;

typedef ProfileResult = {depth:Float,facets:Int,oracleRatio:Float};
typedef FacetSample = {radius:Float,error:Float,source:Int};

class HobbingTests {
  static var assertions=0;
  static function check(value:Bool,message:String):Void {assertions++;if(!value)throw message;}
  static function near(a:Float,b:Float,tolerance:Float,message:String):Void
    check(Math.isFinite(a)&&Math.abs(a-b)<=tolerance,'$message: $a versus $b');
  static function rejects(f:Void->Void,message:String):Void {
    var failed=false;try f()catch(_:Dynamic)failed=true;check(failed,message);
  }

  static function scheduleChecks():Void {
    var omega=100*Math.PI,frames:Array<HobbingKeyframe> = [];
    for(k in 0...4) {
      var t=k*0.004;
      frames.push(new HobbingKeyframe(t,new HobbingAxes(40,0,0,-0.01,omega*t,0.023*t)));
    }
    var motion=new HobbingMotion(frames),events=GashPass.fromMotion(motion,12,0.3);
    var constant=GashPass.constantSpeed(12,3000,0.012,0.3),between=0;
    check(events.length==constant.length,"tick interpolation retains every gash event");
    for(i in 0...events.length) {
      near(events[i].center,constant[i].center,1e-15,"integer master-phase crossing time");
      near(events[i].start,constant[i].start,1e-15,"engagement start between ticks");
      near(events[i].end,constant[i].end,1e-15,"engagement end between ticks");
      check(events[i].gash==constant[i].gash,"physical gash index");
      if(events[i].center>0&&events[i].center<0.012) {
        var a=motion.at(events[i].center);
        near(a.c,0.023*events[i].center,1e-16,"C retains independent measured motion");
        var ticks=events[i].center/0.004;
        if(Math.abs(ticks-Math.round(ticks))>1e-9)between++;
      }
    }
    check(between>=5,"events are evaluated inside controller ticks");
    var pose=new HobbingAxes(10,0,3,0,0,0).pose();
    near(pose.get_translation(0)+2*pose.get_rotation(0)+4*pose.get_rotation(1)+6*pose.get_rotation(2),8,0,"chain X");
    near(pose.get_translation(1)+2*pose.get_rotation(3)+4*pose.get_rotation(4)+6*pose.get_rotation(5),-6,0,"chain Y");
    near(pose.get_translation(2)+2*pose.get_rotation(6)+4*pose.get_rotation(7)+6*pose.get_rotation(8),-1,0,"chain Z");
    rejects(()->{new HobbingMotion([frames[0],frames[0]]);},"duplicate ticks rejected");
    rejects(()->{GashPass.constantSpeed(12,Math.POSITIVE_INFINITY,1,0.3);},"invalid speed rejected");
    var reverse=new HobbingMotion([frames[1],new HobbingKeyframe(0.01,frames[0].axes)]);
    rejects(()->{GashPass.fromMotion(reverse,12,0.3);},"reversing master is not silently scheduled");
    var accelerating=new HobbingMotion([
      new HobbingKeyframe(0,new HobbingAxes(40,0,0,0,0,0)),
      new HobbingKeyframe(0.01,new HobbingAxes(40,0,0,0,Math.PI,0.1)),
      new HobbingKeyframe(0.03,new HobbingAxes(40,0,0,0,2*Math.PI,0.2))]);
    near(accelerating.timeAtMaster(1.5*Math.PI),0.02,1e-15,"event timing follows varying master speed");
    Sys.println('hobbing schedule: ${events.length} exact events, $between between ticks');
  }

  static function profileCase(gashes:Int,starts:Int=1):ProfileResult {
    var spec=new GearSpec(1,40,10,20*Math.PI/180),profile=new ToothProfile(spec);
    var hob=new HobDesign(ConjugateRack.of(profile),starts,32,gashes);
    var rlo=profile.rootJoinRadius+0.02,rhi=profile.tipJoinRadius-0.02;
    var stock=RingStock.annulus(new RingGrid(rlo,rhi,-0.000001,0.000001,(rhi-rlo)/192,1));
    var job=new IdealHobbingJob(spec,hob,3000,1.1),sweep=new HobbingSweep(hob,starts==1?-4:-6,starts==1?4:6,0.00002);
    var began=Sys.time(),passes=job.schedule(stock.grid);
    for(pass in passes)sweep.cutPass(stock,job.axesAt,pass,0.000005);
    var min=1e100,max=-1e100,groups:Array<Array<FacetSample>> = [],previous=-1;
    var period=Std.int(gashes*spec.teeth/starts);
    for(r in 0...stock.grid.radialCount) {
      var radius=stock.grid.radius(r),circle=stock.circle(r,0);
      check(circle.length>0&&circle[0].lo==0&&circle[0].hi<Math.PI/spec.teeth,"generated tooth crosses the profile circle");
      var p=profile.flankAtRadius(radius),truth=Math.atan2(p.x,p.y);
      var error=(circle[0].hi-truth)*profile.spec.baseRadius();
      min=Math.min(min,error);max=Math.max(max,error);
      check(circle[0].hiPass>=0,"flank keeps the actual gash pass source");
      var source=circle[0].hiPass%period;
      if(source!=previous){groups.push([]);previous=source;}
      groups[groups.length-1].push({radius:radius,error:error,source:source});
    }
    // A discrete tangent-envelope oracle, independent of the cutting algorithm.
    // Involute curvature radius rho=sqrt(r^2-rb^2); delta=rho*dC^2/8 to second order.
    // See Kotlyar, Gear Technology May/June 1994, Fig.7, and Li et al. 2023, Fig.16.
    var dC=2*Math.PI*starts/(spec.teeth*gashes),ratios=0.0,complete=0;
    for(k in 1...groups.length-1) {
      var group=groups[k],low=1e100,high=-1e100,radius=0.0;
      for(p in group){low=Math.min(low,p.error);high=Math.max(high,p.error);radius+=p.radius;}
      radius/=group.length;
      var rho=Math.sqrt(radius*radius-spec.baseRadius()*spec.baseRadius());
      ratios+=(high-low)/(rho*dC*dC/8);complete++;
    }
    check(complete>=2,"at least two complete generating facets evaluated");
    var ratio=ratios/complete;
    check(ratio>0.75&&ratio<1.25,"sampled facet depth agrees with local curvature oracle");
    function roll(radius:Float):Float {
      var p=profile.flankAtRadius(radius);
      return Math.atan2(p.x,p.y)-Math.acos(spec.baseRadius()/radius);
    }
    var expectedSpan=Math.abs(roll(stock.grid.radius(0))-roll(stock.grid.radius(stock.grid.radialCount-1)))/dC;
    check(groups.length>=Math.floor(expectedSpan)&&groups.length<=Math.ceil(expectedSpan)+2,
      "facet count follows generating roll span, not gashes times starts");
    check(max>0&&max-min>0,"finite-gash scallops emerge from geometry");
    Sys.println('profile: starts=$starts gashes=$gashes facets=${groups.length} normal_depth_mm=${max-min} curvature_ratio=$ratio time_s=${Sys.time()-began}');
    sweep.dispose();stock.dispose();
    return {depth:max-min,facets:groups.length,oracleRatio:ratio};
  }

  static function widthAt(stock:RingStock,radial:Int,axial:Int,center:Float):Float {
    var arcs=stock.circle(radial,axial),tau=2*Math.PI;
    if(center<1e-12||tau-center<1e-12) {
      check(arcs[0].lo==0&&arcs[arcs.length-1].hi==tau,"seam tooth has both halves");
      return arcs[0].hi+tau-arcs[arcs.length-1].lo;
    }
    for(a in arcs)if(a.lo<center&&a.hi>center)return a.hi-a.lo;
    throw 'missing generated tooth at $center';
  }
  static function referenceCase():Void {
    var spec=new GearSpec(0.3,200,8,20*Math.PI/180);
    var hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),1,32,12);
    var stock=RingStock.annulus(new RingGrid(29.925,30.075,-0.000001,0.000001,0.05,1));
    var job=new IdealHobbingJob(spec,hob,3000,1.1),sweep=new HobbingSweep(hob,-8,8,0.00005),began=Sys.time();
    var report=sweep.cut(stock,job.axesAt,job.schedule(stock.grid));
    near(stock.grid.radius(1),30,1e-12,"reference measuring circle");
    var maxError=0.0,sum=0.0;
    for(tooth in 0...spec.teeth) {
      var center=(Math.PI/2+tooth*2*Math.PI/spec.teeth)%(2*Math.PI);
      var width=widthAt(stock,1,0,center)*30;
      maxError=Math.max(maxError,Math.abs(width-spec.pitchToothThickness()));sum+=width;
      near(width,spec.pitchToothThickness(),0.0002,"reference tooth thickness within 0.2 micrometres");
    }
    check(report.refinement>=1&&report.maxArcChange<=0.00005,"sweep converges on all reference slice circles");
    Sys.println('reference: teeth=${spec.teeth} mean_mm=${sum/spec.teeth} max_thickness_error_mm=$maxError convergence_mm=${report.maxArcChange} refinements=${report.refinement} poses=${report.poses} time_s=${Sys.time()-began}');
    // Failure after an actual cutting pass must restore the incoming stock.
    var saved=stock.snapshot(),peer=RingStock.annulus(stock.grid);peer.restore(saved);
    var pass=job.schedule(stock.grid)[0];
    rejects(()->{sweep.cut(stock,job.axesAt,[pass,new GashPass(hob.gashes,1,0,0,0.001)]);},"invalid later pass rejects the job");
    var comparison=stock.compareStock(peer);
    near(comparison.leftoverVolume()+comparison.gougeVolume(),0,0,"failed job is transactional");
    saved.dispose();peer.dispose();sweep.dispose();stock.dispose();
  }
  static function feedCase():Void {
    var spec=new GearSpec(1,40,10,20*Math.PI/180);
    var hob=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),1,32,12);
    var stock=RingStock.annulus(new RingGrid(19.9999,20.0001,0,2,1,0.4));
    var job=new IdealHobbingJob(spec,hob,3000,3,2,-2),sweep=new HobbingSweep(hob,-4,4,0.00005),began=Sys.time();
    for(pass in job.schedule(stock.grid))sweep.cutPass(stock,job.axesAt,pass,0.0000125);
    var minimum=1e100,maximum=-1e100;
    for(z in 0...stock.grid.axialCount) {
      var low=1e100,high=-1e100;
      for(tooth in 1...40) {
        var center=tooth*2*Math.PI/40;
        var boundary=-1.0;
        for(span in stock.circle(0,z))if(span.lo<center&&span.hi>center){boundary=span.hi;break;}
        check(boundary>0,"feed flank is present");
        var error=(boundary-center-Math.PI/80)*20*Math.cos(spec.pressureAngle);
        low=Math.min(low,error);high=Math.max(high,error);
      }
      minimum=Math.min(minimum,low);maximum=Math.max(maximum,high);
      check(high>low,"axial-feed scallops are present on each slice");
    }
    // Guba, Huesemann, Karpuschewski (2020), Eq.1, doi:10.1016/j.cirpj.2020.06.005.
    // Outside-diameter formula is an approximation; the swept flank cuts at its actual radius.
    var oracle=job.feed*job.feed*Math.sin(spec.pressureAngle)/(4*hob.outsideDiameter);
    check((maximum-minimum)/oracle>0.8&&(maximum-minimum)/oracle<1.2,"feed marks agree with outside-diameter oracle");
    Sys.println('feed: measured_depth_mm=${maximum-minimum} approximate_depth_mm=$oracle ratio=${(maximum-minimum)/oracle} time_s=${Sys.time()-began}');
    sweep.dispose();stock.dispose();
  }
  public static function run():Void {
    scheduleChecks();
    var coarse=profileCase(4),fine=profileCase(8),twoStart=profileCase(4,2);
    check(coarse.depth/fine.depth>3.2&&coarse.depth/fine.depth<4.8,"doubling gashes reduces profile scallops about fourfold");
    check(fine.facets>coarse.facets&&twoStart.facets<coarse.facets,"generating-cut density responds to gashes and starts");
    referenceCase();feedCase();
    Sys.println('HobbingTests: $assertions assertions passed');
  }
  public static function main():Void {
    if(Sys.args().length==0){run();return;}
    switch(Sys.args()[0]) {
      case "schedule":scheduleChecks();
      case "profile":profileCase(4);profileCase(8);profileCase(4,2);
      case "reference":referenceCase();
      case "feed":feedCase();
      case _:throw "unknown hobbing test case";
    }
    Sys.println('HobbingTests: $assertions assertions passed');
  }
}
