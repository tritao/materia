package gearkit;

/** Numerical spur-gear inspection. No datum recentering: eccentricity remains measurable.
    Both flanks of every tooth are inspected; missing coverage cannot earn a grade.
    All deviations/traces in micrometres, circular tooth thickness in mm. */
class GearMeasurement {
  public final values:Map<String,Float> = [];
  public final grades:Map<String,Null<Int>> = [];
  public final profiles:Array<GearTrace> = [];
  public final helices:Array<GearTrace> = [];
  public final pitch:Array<Array<Float>> = [];
  public final cumulativePitch:Array<Array<Float>> = [];
  public final radialMeasurements:Array<Float> = [];
  public final thickness:Array<Float> = [];
  /** Annex E.5 reconstructed ball centres and opposing-ball dimensions, mm.
      External: over balls. Internal: between the inward ball surfaces. */
  public final ballCentres:Array<Point2> = [];
  public final dimensionsOverBalls:Array<Float> = [];
  public var ballDiameter(default,null):Null<Float>;
  public final warnings:Array<String> = [];
  public final extrapolation:Array<String>;
  public final inspection:GearInspection;
  public var overallGrade(default,null):Null<Int>;
  /** Only set for complete, sufficiently sampled, in-scope filtered numerical inspections. */
  public var isoClass(default,null):Null<Int>;
  public var profileSourcePoints(default,null):Int = 0;
  public var helixSourcePoints(default,null):Int = 0;
  function new(spec:GearSpec,inspection:GearInspection) {
    this.inspection=inspection;extrapolation=Iso1328.outsideScope(spec);
  }

  public static function fromRings(rings:GearRings,s:GearSpec,?inspection:GearInspection):GearMeasurement {
    var assumed=inspection==null, p=new ToothProfile(s),rb=s.baseRadius();
    if(inspection==null) inspection=new GearInspection(s,p.rootJoinRadius+s.sign()*0.01*Math.abs(p.tipJoinRadius-p.rootJoinRadius));
    if(inspection.spec!=s)throw "inspection settings belong to another GearSpec";
    var out=new GearMeasurement(s,inspection), cfg=inspection;
    if(assumed)out.warnings.push("Profile control radius assumed from geometric root; specify active profile for ISO assessment");
    if(cfg.profileCutoff==0||cfg.helixCutoff==0)out.warnings.push("Unfiltered diagnostic assessment");
    var rs=rings.radii(),zs=rings.axialPositions();
    validateAxis(rs,"radius");validateAxis(zs,"axial");
    var cache:Map<String,Array<Float>> = [];
    function half(r:Float):Float {var a=Math.acos(rb/r);return s.pitchToothThickness()/(2*s.pitchRadius())+s.sign()*(Math.tan(s.pressureAngle)-s.pressureAngle-Math.tan(a)+a);}
    function errors(ri:Int,zi:Int):Array<Float> {
      var key=ri+":"+zi;if(cache.exists(key))return cache.get(key);
      var arcs=rings.boundaries(ri,zi),ends:Array<Float> = [],tau=2*Math.PI;
      if(arcs.length%2!=0)throw "ring boundaries must be interval pairs";
      var previous=-1.0;
      for(i in 0...arcs.length){var a=arcs[i];
        if(!Math.isFinite(a)||a<0||a>tau||a<=previous)throw "ring boundaries must be finite and strictly ordered";
        previous=a;
        // A material arc crossing the seam has artificial endpoints at 0 and 2pi.
        if((i==0&&a==0&&arcs[arcs.length-1]==tau)||(i==arcs.length-1&&a==tau&&arcs[0]==0))continue;
        ends.push(a);
      }
      if(ends.length!=2*s.teeth)throw "measurement circle must have exactly two boundaries per tooth";
      var found:Array<Float> = [],used:Map<Int,Bool> = [],h=half(rs[ri]);
      for(t in 0...s.teeth)for(f in 0...2){
        var nominal=Math.PI/2+t*tau/s.teeth+(f==0?-h:h),best=Math.POSITIVE_INFINITY,index=-1,delta=0.0;
        var angle=nominal-tau*Math.floor(nominal/tau),left=0,right=ends.length;
        while(left<right){var mid=(left+right)>>1;if(ends[mid]<angle)left=mid+1;else right=mid;}
        for(i in [(left+ends.length-1)%ends.length,left%ends.length]){
          var d=wrap(ends[i]-nominal);if(Math.abs(d)<best){best=Math.abs(d);index=i;delta=d;}
        }
        if(best>=Math.PI/(2*s.teeth)||used.exists(index))throw "ambiguous or missing tooth flank";
        used.set(index,true);found.push(delta);
      }
      cache.set(key,found);return found;
    }
    var rolls=[for(r in rs)GearInspection.roll(r,rb)];
    function at(r:Float,z:Float):Array<Float>{
      var ir=bracket(rs,r),iz=bracket(zs,z);
      var jr=rs.length==1?ir:ir+1,jz=zs.length==1?iz:iz+1;
      var roll=GearInspection.roll(r,rb),fr=ir==jr?0:(roll-rolls[ir])/(rolls[jr]-rolls[ir]);
      var fz=iz==jz?0:(z-zs[iz])/(zs[jz]-zs[iz]);
      var aa=errors(ir,iz),ba=errors(jr,iz),ab=errors(ir,jz),bb=errors(jr,jz);
      return [for(k in 0...aa.length)(aa[k]*(1-fr)+ba[k]*fr)*(1-fz)+(ab[k]*(1-fr)+bb[k]*fr)*fz];
    }
    // Pitch and circular tooth thickness at the specified measurement circle and Z.
    if(covers(rs,cfg.measurementRadius,cfg.measurementRadius)&&covers(zs,cfg.pitchZ,cfg.pitchZ)){
      var measured=at(cfg.measurementRadius,cfg.pitchZ);
      for(f in 0...2){var index=[for(t in 0...s.teeth)measured[2*t+f]*cfg.measurementRadius*1000];
        var datum=index[0];var cumulative=[for(v in index)v-datum];
        var single=[for(t in 0...s.teeth)index[(t+1)%s.teeth]-index[t]];
        out.pitch.push(single);out.cumulativePitch.push(cumulative);
        out.maximum("fp",maxAbs(single));out.maximum("Fp",range(cumulative));
      }
      var h=half(cfg.measurementRadius),nominalWidth=2*h*cfg.measurementRadius;
      for(t in 0...s.teeth)out.thickness.push(nominalWidth+(measured[2*t+1]-measured[2*t])*cfg.measurementRadius);
      // Annex E.5: reconstruct nominal-base-circle involutes through pitch probings,
      // offset each by the virtual ball radius, then intersect in each tooth space.
      var a=Math.acos(rb/cfg.measurementRadius),delta=Math.PI/s.teeth-h;
      var ball=cfg.measurementRadius*Math.sin(delta)/Math.cos(a+s.sign()*delta);
      if(ball<=0||!Math.isFinite(ball))throw "virtual runout probe cannot contact both flanks";
      out.ballDiameter=2*ball;
      var invM=Math.tan(a)-a;
      for(t in 0...s.teeth){var gap=2*delta+measured[2*((t+1)%s.teeth)]-measured[2*t+1];
        var target=invM+s.sign()*(ball/rb-gap/2);
        if(target<0)throw "runout involute intersection below base circle";
        var lo=0.0,hi=Math.PI/2-1e-8;
        for(_ in 0...64){var mid=(lo+hi)/2;if(Math.tan(mid)-mid<target)lo=mid;else hi=mid;}
        var centreRadius=rb/Math.cos((lo+hi)/2);
        out.radialMeasurements.push(centreRadius);
        var centreAngle=Math.PI/2+(t+0.5)*2*Math.PI/s.teeth
          +(measured[2*((t+1)%s.teeth)]+measured[2*t+1])/2;
        out.ballCentres.push(new Point2(centreRadius*Math.cos(centreAngle),centreRadius*Math.sin(centreAngle)));
      }
      for(t in 0...s.teeth){
        var a=out.ballCentres[t],b=out.ballCentres[(t+Std.int(s.teeth/2))%s.teeth];
        out.dimensionsOverBalls.push(Math.sqrt(Math.pow(a.x-b.x,2)+Math.pow(a.y-b.y,2))+s.sign()*2*ball);
      }
      out.values.set("Fr",range(out.radialMeasurements)*1000);
    }else out.warnings.push("Pitch/thickness/runout circle or axial position not covered");
    var rlo=Math.min(cfg.controlRadius,cfg.tipFormRadius),rhi=Math.max(cfg.controlRadius,cfg.tipFormRadius);
    if(covers(rs,rlo,rhi)&&covers(zs,cfg.profileZ,cfg.profileZ)){
      var amin=Math.min(cfg.rollStart,cfg.rollEnd),amax=Math.max(cfg.rollStart,cfg.rollEnd);
      for(u in rolls)if(u>=amin&&u<=amax)out.profileSourcePoints++;
      var rtip=GearInspection.roll(cfg.tipFormRadius,rb),start=Math.min(cfg.rollStart,rtip),end=Math.max(cfg.rollStart,rtip);
      var count=Std.int(Math.max(151,out.profileSourcePoints+2));
      var xs=[for(i in 0...count)start+(end-start)*i/(count-1)];
      var data=[for(u in xs)at(Math.sqrt(rb*rb+u*u),cfg.profileZ)];
      for(t in 0...s.teeth)for(f in 0...2){
        var ys=[for(v in data)v[2*t+f]*(f==0?-1:1)*rb*1000];
        var trace=new GearTrace(t,f,xs,ys,cfg.profileCutoff,amin,amax,cfg.rollTip-cfg.rollStart);
        out.profiles.push(trace);out.maximum("Falpha",trace.total);out.maximum("ffalpha",trace.form);out.maximum("fHalpha",Math.abs(trace.slope));
      }
      if(cfg.profileCutoff>0&&maxStep(rolls,amin,amax)>cfg.profileCutoff/5)
        out.warnings.push("Profile source spacing does not resolve filter cutoff");
      if(out.profileSourcePoints<150)out.warnings.push("Profile has fewer than 150 independent source samples in evaluation range");
    }else out.warnings.push("Profile evaluation radii or axial position not covered");
    // Face endpoints are cell centres, so use the available end samples and disclose
    // their absence. Never invent unseen end excess material by extrapolation.
    if(covers(rs,cfg.measurementRadius,cfg.measurementRadius)&&covers(zs,cfg.axialStart,cfg.axialEnd)){
      var usable=[for(z in zs)if(z>=0&&z<=s.faceWidth)z];
      for(z in usable)if(z>=cfg.axialStart&&z<=cfg.axialEnd)out.helixSourcePoints++;
      var count=Std.int(Math.max(151,out.helixSourcePoints+2));
      var xs=[for(i in 0...count)usable[0]+(usable[usable.length-1]-usable[0])*i/(count-1)];
      var data=[for(z in xs)at(cfg.measurementRadius,z)];
      for(t in 0...s.teeth)for(f in 0...2){
        var ys=[for(v in data)v[2*t+f]*(f==0?-1:1)*rb*1000];
        var trace=new GearTrace(t,f,xs,ys,cfg.helixCutoff,cfg.axialStart,cfg.axialEnd,s.faceWidth);
        out.helices.push(trace);out.maximum("Fbeta",trace.total);out.maximum("ffbeta",trace.form);out.maximum("fHbeta",Math.abs(trace.slope));
      }
      if(cfg.helixCutoff>0&&maxStep(usable,cfg.axialStart,cfg.axialEnd)>cfg.helixCutoff/5)
        out.warnings.push("Helix source spacing does not resolve filter cutoff");
      if(out.helixSourcePoints<Math.ceil(5*s.faceWidth/Math.max(cfg.helixCutoff,1e-12)))
        out.warnings.push("Helix source sampling is below 5*b/cutoff");
      if(usable[0]>0||usable[usable.length-1]<s.faceWidth)out.warnings.push("Face endpoints not sampled; end excess material unchecked");
    }else out.warnings.push("Helix evaluation range not covered");
    var worst=1,complete=true;
    for(parameter in Iso1328.parameters){
      if(!out.values.exists(parameter)){complete=false;continue;}
      var grade=Iso1328.grade(s,parameter,out.value(parameter));out.grades.set(parameter,grade);
      if(grade==null)complete=false;else worst=Std.int(Math.max(worst,grade));
    }
    out.overallGrade=complete?worst:null;
    out.isoClass=complete&&out.warnings.length==0&&out.extrapolation.length==0?worst:null;
    return out;
  }
  public function value(key:String):Float {
    var v=values.get(key);if(v==null)throw 'parameter $key was not measured';return v;
  }
  function maximum(key:String,value:Float):Void {
    if(!values.exists(key)||value>this.value(key))values.set(key,value);
  }
  static function wrap(a:Float):Float {var tau=2*Math.PI;return a-tau*Math.floor((a+Math.PI)/tau);}
  static function covers(a:Array<Float>,lo:Float,hi:Float):Bool return a[0]<=lo+1e-10&&a[a.length-1]>=hi-1e-10;
  static function bracket(a:Array<Float>,v:Float):Int {
    if(!covers(a,v,v))throw "cannot extrapolate an unmeasured ring";
    if(a.length==1)return 0;
    var lo=0,hi=a.length-1;while(hi-lo>1){var mid=(lo+hi)>>1;if(a[mid]<=v)lo=mid;else hi=mid;}return lo;
  }
  static function validateAxis(a:Array<Float>,label:String):Void {
    if(a.length==0)throw 'empty $label axis';
    for(i in 0...a.length)if(!Math.isFinite(a[i])||(i>0&&a[i]<=a[i-1]))throw 'invalid $label axis';
  }
  static function maxStep(a:Array<Float>,lo:Float,hi:Float):Float {
    var step=0.0;for(i in 1...a.length)if(a[i]>=lo&&a[i-1]<=hi)step=Math.max(step,a[i]-a[i-1]);return step;
  }
  static function maxAbs(a:Array<Float>):Float{var v=0.0;for(x in a)v=Math.max(v,Math.abs(x));return v;}
  static function range(a:Array<Float>):Float{var lo=Math.POSITIVE_INFINITY,hi=Math.NEGATIVE_INFINITY;for(x in a){lo=Math.min(lo,x);hi=Math.max(hi,x);}return hi-lo;}
}
