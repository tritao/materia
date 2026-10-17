import gearkit.*;

class MetrologyTests {
  static var assertions=0;
  static function check(ok:Bool,message:String):Void {assertions++;if(!ok)throw message;}
  static function near(a:Float,b:Float,e:Float,message:String):Void check(Math.abs(a-b)<=e,'$message: $a vs $b');
  static function rejects(fn:Void->Void,message:String):Void {var failed=false;try fn()catch(_:Dynamic)failed=true;check(failed,message);}
  public static function main():Void run();
  public static function run():Void {
    var s=new GearSpec(0.5,100,10,20*Math.PI/180),p=new ToothProfile(s);
    var control=p.rootJoinRadius+0.02*(p.tipJoinRadius-p.rootJoinRadius);
    var cfg=new GearInspection(s,control);
    // Independently calculated decimal fixtures, ISO 2013 equations, NOT the 1995 tables.
    var expected=[5.25,16.339087296526,14.705178566873,6.774077427960,5.275,4.25,8.05221390317087129612066005718,5.91799969390635396648009314846,5.46035057165220652840003492160];
    for(i in 0...expected.length)near(Iso1328.tolerance(s,Iso1328.parameters[i],5,false),expected[i],0.00001,'class 5 equation ${Iso1328.parameters[i]}');
    near(Iso1328.tolerance(s,"fp",1),1.3,1e-12,"class 1 rounding 0.1 um");
    near(Iso1328.tolerance(s,"fp",5),5.5,1e-12,"class 5 rounding 0.5 um");
    near(Iso1328.tolerance(s,"Fp",5),16,1e-12,"integer rounding");
    near(Iso1328.tolerance(s,"fp",7,false),10.5,1e-12,"two classes double unrounded tolerance");
    check(Iso1328.grade(s,"fp",1e6)==null,"beyond class 11 fails");
    rejects(()->{Iso1328.tolerance(s,"thickness",5);},"thickness has no ISO flank class");
    rejects(()->{Iso1328.grade(s,"fp",Math.NaN);},"NaN cannot pass grading");
    var source=new SyntheticGearRings(s,control);
    var perfect=GearMeasurement.fromRings(source,s,cfg);
    check(perfect.isoClass==1,'perfect gear class 1: ${perfect.warnings}');
    for(k in Iso1328.parameters)near(perfect.value(k),0,1e-6,'perfect $k');
    for(w in perfect.thickness)near(w,s.pitchToothThickness(),1e-10,"circular tooth thickness");
    var delta=Math.PI/(2*s.teeth),q=s.pitchRadius()*Math.sin(delta)/Math.cos(s.pressureAngle+delta);
    var ballCentre=Math.sqrt(s.pitchRadius()*s.pitchRadius()+q*q+2*s.pitchRadius()*q*Math.sin(s.pressureAngle));
    for(v in perfect.dimensionsOverBalls)near(v,2*ballCentre+2*q,1e-10,"over balls matches independent tangent-contact construction");
    check(perfect.profiles.length==200&&perfect.helices.length==200,"every tooth and both flanks");
    var ecc=new SyntheticGearRings(s,control,0.005);
    var measured=GearMeasurement.fromRings(ecc,s,cfg);
    near(measured.value("Fr"),10,0.05,"actual translated involutes: Fr=2e");
    // First harmonic of index error from translation: amplitude e*rM/rb, phase flank dependent.
    for(f in 0...2){var v=measured.cumulativePitch[f],cs=0.0,sn=0.0,power=0.0,mean=0.0;
      for(x in v)mean+=x/v.length;
      for(t in 0...s.teeth){var a=2*Math.PI*t/s.teeth;cs+=(v[t]-mean)*Math.cos(a)*2/s.teeth;sn+=(v[t]-mean)*Math.sin(a)*2/s.teeth;}
      for(t in 0...s.teeth)power+=Math.pow(v[t]-mean-cs*Math.cos(2*Math.PI*t/s.teeth)-sn*Math.sin(2*Math.PI*t/s.teeth),2)/s.teeth;
      near(Math.sqrt(cs*cs+sn*sn),5*s.pitchRadius()/s.baseRadius(),0.005,"eccentric pitch first-harmonic amplitude");
      check(Math.sqrt(power)<0.005,"eccentric index error is once per revolution");
    }
    var sloped=new SyntheticGearRings(s,control,0,4);
    var slope=GearMeasurement.fromRings(sloped,s,cfg);
    near(slope.value("fHalpha"),4,0.001,"profile slope extrapolates to tip diameter");
    near(slope.value("ffalpha"),0,1e-6,"linear profile has zero form");
    var twisted=new SyntheticGearRings(s,control,0,0,3);
    var helix=GearMeasurement.fromRings(twisted,s,cfg);
    near(helix.value("fHbeta"),3,1e-6,"helix slope extrapolates to full face width");
    near(helix.value("ffbeta"),0,1e-6,"linear helix has zero form");
    var xs=[for(i in 0...1001)i*0.001],ys=[for(x in xs)Math.cos(2*Math.PI*x/0.1)];
    var filtered=GearTrace.gaussian(xs,ys,0.1);
    near(filtered[500],0.5,0.0002,"Gaussian passes 50% of cutoff sinusoid away from edges");
    var sparse=GearMeasurement.fromRings(new SyntheticGearRings(s,control,0,0,0,10,10),s,cfg);
    check(sparse.isoClass==null&&sparse.warnings.length>=2,"interpolating sparse data cannot qualify ISO assessment");
    var partial=GearMeasurement.fromRings(new SliceRings(source),s,cfg);
    check(partial.overallGrade==null&&partial.isoClass==null&&!partial.values.exists("Fbeta"),"missing helix never passes as zero");
    var fine=new GearSpec(0.3,200,8,20*Math.PI/180),fp=new ToothProfile(fine);
    var fc=fp.rootJoinRadius+0.02*(fp.tipJoinRadius-fp.rootJoinRadius);
    var extrapolated=GearMeasurement.fromRings(new SyntheticGearRings(fine,fc),fine,new GearInspection(fine,fc));
    check(extrapolated.overallGrade==1&&extrapolated.isoClass==null&&extrapolated.extrapolation.length==1,"fine module labelled extrapolated");
    rejects(()->{new GearInspection(s,s.rootRadius());},"control radius below involute rejected");
    var internal=new GearSpec(0.5,240,10,20*Math.PI/180,true);
    var ip=new ToothProfile(internal),ic=ip.rootJoinRadius-0.02*(ip.rootJoinRadius-ip.tipJoinRadius);
    var internalReport=GearMeasurement.fromRings(new SyntheticGearRings(internal,ic),internal,new GearInspection(internal,ic));
    check(internalReport.isoClass==1,'internal perfect gear class 1: ${internalReport.warnings}');
    var x=[for(i in 0...201)i/200.0];
    var endExcess=[for(u in x)u>0.95?20.0:0.0];
    var excess=new GearTrace(0,0,x,endExcess,0,0.1,0.9,1);
    near(excess.total,20,1e-12,"end excess included outside evaluation window");
    var endRelief=[for(u in x)u>0.95?-20.0:0.0];
    near(new GearTrace(0,0,x,endRelief,0,0.1,0.9,1).total,0,1e-12,"end minus material may be ignored");
    rejects(()->{GearMeasurement.fromRings(source,s,new GearInspection(fine,fc));},"wrong GearSpec settings rejected");
    rejects(()->{GearTrace.gaussian([0.0,0.0],[0.0,0.0],0.1);},"invalid filter axis rejected");
    Sys.println('MetrologyTests: $assertions assertions passed');
  }
}

/** Independent analytic fixture: actual translated involutes, not a runout error formula. */
class SyntheticGearRings implements GearRings {
  final s:GearSpec;final rr:Array<Float>;final zz:Array<Float>;
  final eccentricity:Float;final slope:Float;final twist:Float;final control:Float;
  final byRadius:Map<Int,Array<Float>> = [];
  public function new(s:GearSpec,control:Float,eccentricity:Float=0,slope:Float=0,twist:Float=0,nr:Int=181,nz:Int=181){
    this.s=s;this.control=control;this.eccentricity=eccentricity;this.slope=slope;this.twist=twist;
    var p=new ToothProfile(s),lo=Math.min(p.rootJoinRadius,p.tipJoinRadius)+0.00001*s.module,hi=Math.max(p.rootJoinRadius,p.tipJoinRadius)-0.00001*s.module;
    rr=[for(i in 0...nr)lo+(hi-lo)*i/(nr-1)];zz=[for(i in 0...nz)s.faceWidth*i/(nz-1)];
  }
  public function radii():Array<Float>return rr.copy();
  public function axialPositions():Array<Float>return zz.copy();
  public function boundaries(r:Int,z:Int):Array<Float>{
    var tau=2*Math.PI,rb=s.baseRadius();
    if(!byRadius.exists(r)){
      var ends:Array<Float> = [];
      for(t in 0...s.teeth)for(f in 0...2){
        var side=f==0?-1:1;
        function point(rho:Float):Array<Float>{
          var alpha=Math.acos(rb/rho);
          var h=s.pitchToothThickness()/(2*s.pitchRadius())+s.sign()*(Math.tan(s.pressureAngle)-s.pressureAngle-Math.tan(alpha)+alpha);
          var u=GearInspection.roll(rho,rb),start=GearInspection.roll(control,rb),tip=GearInspection.roll(s.tipRadius(),rb);
          var angle=Math.PI/2+t*tau/s.teeth+side*(h+slope*(u-start)/(tip-start)/1000/rb);
          return [rho*Math.cos(angle)+eccentricity,rho*Math.sin(angle)];
        }
        var lo=rr[r]-2*Math.abs(eccentricity)-1e-10,hi=rr[r]+2*Math.abs(eccentricity)+1e-10;
        var iterations=eccentricity==0?0:42;
        for(_ in 0...iterations){var mid=(lo+hi)/2,p=point(mid);if(Math.sqrt(p[0]*p[0]+p[1]*p[1])<rr[r])lo=mid;else hi=mid;}
        var p=point((lo+hi)/2);var a=Math.atan2(p[1],p[0]);if(a<0)a+=tau;ends.push(a);
      }
      // Pair material intervals by winding, including seam teeth.
      var spans:Array<Array<Float>> = [];
      for(t in 0...s.teeth){var lo=ends[2*t],hi=ends[2*t+1];if(hi<lo){spans.push([0,hi]);spans.push([lo,tau]);}else spans.push([lo,hi]);}
      spans.sort((a,b)->a[0]<b[0]?-1:1);
      var result:Array<Float> = [];for(a in spans){result.push(a[0]);result.push(a[1]);}byRadius.set(r,result);
    }
    if(twist==0)return byRadius.get(r).copy();
    var shift=twist/1000/rb*zz[z]/s.faceWidth,spans:Array<Array<Float>> = [];
    // Reassemble seam interval before rotating.
    var v=byRadius.get(r).copy();if(v[0]==0&&v[v.length-1]==tau){v.shift();v.pop();v.push(v.shift()+tau);}
    for(i in 0...Std.int(v.length/2)){
      var lo=(v[2*i]+shift)%tau,hi=(v[2*i+1]+shift)%tau;
      if(hi<lo){spans.push([0,hi]);spans.push([lo,tau]);}else spans.push([lo,hi]);
    }
    spans.sort((a,b)->a[0]<b[0]?-1:1);var result:Array<Float> = [];for(a in spans){result.push(a[0]);result.push(a[1]);}return result;
  }
}
class SliceRings implements GearRings {
  final src:GearRings;public function new(src:GearRings)this.src=src;
  public function radii():Array<Float>return src.radii();
  public function axialPositions():Array<Float>return [5.0];
  public function boundaries(r:Int,z:Int):Array<Float>return src.boundaries(r,0);
}
