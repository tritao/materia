package stockkit.hobbing;
import gearkit.Point3;

/** Identified error with provenance. Perturbs tool/work motion before cutting,
    never metrology values after the fact. Dimensions mm/rad/s; phases explicit. */
class HobbingErrorSource {
  public final id:String;public final part:String;public final basis:String;
  public final apply:(Float,HobbingAxes)->HobbingAxes;
  public final warp:Null<Point3->Point3>;
  public function new(id:String,part:String,basis:String,apply:(Float,HobbingAxes)->HobbingAxes,?warp:Point3->Point3) {
    if(id==null||id.length==0||part==null||part.length==0||basis==null||basis.length==0)throw "error source needs provenance";
    this.id=id;this.part=part;this.basis=basis;this.apply=apply;this.warp=warp;
  }
  static function amplitude(v:Float):Void {if(!Math.isFinite(v)||v<0)throw "invalid error amplitude";}
  static function shifted(a:HobbingAxes,change:HobbingPoseOffset->Void):HobbingAxes {
    var offset=a.offset==null?new HobbingPoseOffset():a.offset.copy();change(offset);
    return new HobbingAxes(a.x,a.y,a.z,a.a,a.b,a.c,offset);
  }
  public static function workEccentricity(id:String,part:String,e:Float,phase:Float=0):HobbingErrorSource {
    amplitude(e);if(!Math.isFinite(phase))throw "invalid eccentricity phase";
    return new HobbingErrorSource(id,part,'eccentricity $e mm, phase $phase rad; fixed in rotating work frame',(t,a)->
      shifted(a,o->{o.workX+=e*Math.cos(phase);o.workY+=e*Math.sin(phase);}));
  }
  public static function tableAxial(part:String,tir:Float):HobbingErrorSource {
    amplitude(tir);return new HobbingErrorSource("table-axial",part,'assumed axial TIR $tir mm /2, once per C turn',(t,a)->
      shifted(a,o->{o.workZ+=tir/2*Math.cos(a.c);}));
  }
  /** Load-induced tilt, separate from axial TIR. Force is machine-radial N;
      bearing pivot depth/force height mm, moment stiffness Nm/rad. */
  public static function tableTilt(part:String,stiffness:Float,pivotDepth:Float,forceHeight:Float,radialForce:Float->Float):HobbingErrorSource {
    if(!Math.isFinite(stiffness)||stiffness<=0)throw "table moment stiffness required";
    amplitude(pivotDepth);amplitude(forceHeight);
    return new HobbingErrorSource("table-tilt",part,'Assumed load tilt Fr*$forceHeight mm / ($stiffness Nm/rad); bearing pivot $pivotDepth mm below work datum; distinct from axial TIR',(t,a)->{
      var force=radialForce(t);amplitude(force);var theta=force*forceHeight/(1000*stiffness);
      return shifted(a,o->{if((o.workTiltX!=0||o.workTiltY!=0)&&o.workPivotZ!= -pivotDepth)throw "incompatible table tilt pivots";
        o.workPivotZ= -pivotDepth;o.workTiltX-=theta*Math.sin(a.c);o.workTiltY-=theta*Math.cos(a.c);});
    });
  }
  public static function hobRadial(part:String,tir:Float):HobbingErrorSource {
    amplitude(tir);return new HobbingErrorSource("hob-radial",part,'radial TIR $tir mm /2, offset fixed to B rotor',(t,a)->
      shifted(a,o->{o.toolX+=tir/2;}));
  }
  public static function hobAxial(part:String,tir:Float):HobbingErrorSource {
    amplitude(tir);return new HobbingErrorSource("hob-axial",part,'assumed sinusoidal axial TIR $tir mm /2 once per B turn',(t,a)->
      shifted(a,o->{o.toolZ+=tir/2*Math.cos(a.b);}));
  }
  public static function encoder(part:String,e:Float,radius:Float):HobbingErrorSource {
    amplitude(e);if(!Math.isFinite(radius)||radius<=0||e>=radius)throw "invalid encoder radius";
    return new HobbingErrorSource("encoder-eccentricity",part,'single readhead first-order e/R sine: e=$e mm, R=$radius mm; inverse servo angle error',(t,a)->
      new HobbingAxes(a.x,a.y,a.z,a.a,a.b,a.c-e/radius*Math.sin(a.c),a.offset));
  }
  public static function graduation(part:String,peak:Float,order:Int):HobbingErrorSource {
    amplitude(peak);if(order<1||order>10000)throw "invalid graduation harmonic";
    return new HobbingErrorSource("encoder-graduation",part,'assumed peak $peak rad, harmonic $order; not a measured error map',(t,a)->
      new HobbingAxes(a.x,a.y,a.z,a.a,a.b,a.c-peak*Math.sin(order*a.c),a.offset));
  }
  public static function measuredSync(part:String,error:Float->Float,basis:String):HobbingErrorSource {
    return new HobbingErrorSource("sync",part,"measured HB7 C-minus-baseline following error: "+basis,(t,a)->
      new HobbingAxes(a.x,a.y,a.z,a.a,a.b,a.c+error(t),a.offset));
  }
  /** Diagnostic latency model. A runtime study should instead supply the measured
      C error in apply(), avoiding double counting encoder/servo errors already present. */
  public static function latency(part:String,delay:Float,ratio:Float,master:Float->Float,velocity:Float->Float,predict:Bool):HobbingErrorSource {
    amplitude(delay);if(!Math.isFinite(ratio))throw "invalid sync ratio";
    return new HobbingErrorSource("sync",part,predict?"diagnostic delayed master with first-order prediction":"diagnostic delayed master without prediction",(t,a)->{
      var error=ratio*(master(t-delay)+(predict?delay*velocity(t-delay):0)-master(t));
      return new HobbingAxes(a.x,a.y,a.z,a.a,a.b,a.c+error,a.offset);
    });
  }
  /** Assumed radial form waveform on a zero-rake hob's planar rake regions.
      Does not claim a measured profile map or arbitrary-rake planarity. */
  public static function profile(part:String,peak:Float,pitch:Float):HobbingErrorSource {
    amplitude(peak);if(!Math.isFinite(pitch)||pitch<=0)throw "invalid profile error pitch";
    return new HobbingErrorSource("hob-profile",part,'assumed sinusoidal radial form peak $peak mm per axial pitch $pitch mm; zero rake only',
      (t,a)->a,p->{var r=Math.sqrt(p.x*p.x+p.y*p.y);
        var error=peak*Math.sin(2*Math.PI*p.z/pitch),scale=r==0?1:1+error/r;
        return new Point3(p.x*scale,p.y*scale,p.z);});
  }
  public static function wall(part:String,stiffness:Float,radialForce:Float->Float):HobbingErrorSource {
    if(!Math.isFinite(stiffness)||stiffness<=0)throw "supported wall stiffness must be stated in N/mm";
    return new HobbingErrorSource("wall",part,"quasistatic local wall recession F/k; supported stiffness explicitly supplied, not inferred from pressure",(t,a)->{
      var force=radialForce(t);amplitude(force);
      return new HobbingAxes(a.x+force/stiffness,a.y,a.z,a.a,a.b,a.c,a.offset);
    });
  }
}
