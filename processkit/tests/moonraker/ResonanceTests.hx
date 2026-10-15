import processkit.calibration.KlipperResonance;
import materia.assembly.BeltCalibration.ResonanceMeasurement;

/** Synthetic fixtures use the published Klipper column layout and a known continuous-time oscillator. */
class ResonanceTests {
  static function check(ok:Bool,message:String):Void { if (!ok) throw message; }
  public static function spectrum(frequency:Float):String {
    var out=new StringBuf(); out.add("freq,psd_x,psd_y,psd_z,psd_xyz,mzv(50.0)\n");
    for (i in 0...201) {
      var power=1.0/(1+Math.pow((i-frequency)/4,2));
      out.add('$i,$power,$power,0,${2*power},0.1\n');
    }
    return out.toString();
  }
  public static function decay(frequency:Float,damping:Float,secondMode:Bool=false,noise:Float=0):String {
    var out=new StringBuf(); out.add("#time,accel_x,accel_y,accel_z\n");
    for (i in 0...641) {
      var t=i/3200.0, omega=2*Math.PI*frequency;
      var value=120*Math.exp(-damping*omega*t)*Math.cos(omega*Math.sqrt(1-damping*damping)*t+0.43);
      value+=noise*(Math.sin(2*Math.PI*381*t)+Math.cos(2*Math.PI*877*t));
      if (secondMode) value+=80*Math.exp(-0.03*2*Math.PI*83*t)*Math.cos(2*Math.PI*83*t);
      var otherOmega=2*Math.PI*37;
      var other=90*Math.exp(-0.11*otherOmega*t)*Math.cos(otherOmega*Math.sqrt(1-0.11*0.11)*t+0.83);
      out.add('$t,${value+3.2},${other-8.1},9810\n');
    }
    return out.toString();
  }
  public static function measurement(frequency:Float=57,damping:Float=0.085):ResonanceMeasurement
    return KlipperResonance.measure("x",spectrum(frequency),decay(frequency,damping),0,0.2,"config/resonances/x.csv","config/resonances/ring x.csv");
  public static function run():Void {
    var m=measurement();
    check(Math.abs(m.frequencyHz/57-1)<1e-5 && Math.abs(m.dampingRatio/0.085-1)<1e-4,"recover continuous natural frequency and damping, not PSD/shaper peak");
    check(m.fitError<1e-5 && m.decaySource.indexOf("#sha256=")>0,"verified free waveform and content-addressed provenance");
    var y=KlipperResonance.measure("y",spectrum(37),decay(57,0.085),0,0.2,"spectrum","two-channel decay");
    check(Math.abs(y.frequencyHz/37-1)<1e-5 && Math.abs(y.dampingRatio/0.11-1)<1e-4,
      "the selected sensor channel, not the sum or the other axis, identifies the mode");
    var noisy=KlipperResonance.measure("x",spectrum(57),decay(57,0.085,false,0.05),0,0.2,"spectrum","noisy decay");
    check(Math.abs(noisy.frequencyHz/57-1)<0.001 && Math.abs(noisy.dampingRatio/0.085-1)<0.02,
      "continuous waveform fit tolerates sensor noise without recurrence bias");
    for (kind in 0...7) {
      var refused=false;
      try KlipperResonance.measure("x",spectrum(kind==1 ? 100 : 57),
        kind==2 ? spectrum(57) : kind==5 ? decay(57,-0.085) : kind==6 ? StringTools.replace(decay(57,0.085),"9810","nan") : decay(57,0.085,kind==0),0,kind==3 ? 0.04 : kind==4 ? 0.3 : 0.2,"spectrum","decay")
      catch (_:Dynamic) refused=true;
      check(refused,'refuse mixed modes, mismatched artifacts, PSD as damping evidence, too few cycles or out-of-capture interval ($kind)');
    }
    Sys.println("Klipper resonance checks passed");
  }
  public static function main():Void run();
}
