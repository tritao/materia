package processkit.calibration;

import haxe.crypto.Sha256;
import materia.assembly.BeltCalibration.ResonanceMeasurement;

/** A fit to an explicitly identified free-decay interval of a Klipper accelerometer capture.
 * A sweep PSD or recommended shaper frequency alone does not identify damping. */
private typedef Csv = {var header:Array<String>; var rows:Array<Array<Float>>;}

class KlipperResonance {
  static function csv(text:String):Csv {
    if (text.length>16*1024*1024) throw "Klipper calibration artifact exceeds 16 MiB";
    var header:Array<String> = [], rows:Array<Array<Float>> = [];
    var number = ~/^[+-]?([0-9]+(\.[0-9]*)?|\.[0-9]+)([eE][+-]?[0-9]+)?$/;
    for (line in text.split("\n")) {
      var clean = StringTools.trim(line);
      if (clean.length == 0) continue;
      if (header.length == 0) {
        if (StringTools.startsWith(clean,"#")) clean=StringTools.trim(clean.substr(1));
        var names = [for (part in clean.split(",")) StringTools.trim(part)];
        if (names[0]!="time" && names[0]!="freq") continue;
        header=names; continue;
      }
      if (StringTools.startsWith(clean,"#")) continue;
      var columns = clean.split(",");
      if (columns.length!=header.length) throw "Klipper CSV row does not match its header";
      var row:Array<Float> = [];
      for (column in columns) {
        var token=StringTools.trim(column);
        if (!number.match(token)) throw "Invalid numeric Klipper CSV cell";
        var value=Std.parseFloat(token);
        if (!Math.isFinite(value)) throw "Non-finite Klipper CSV value";
        row.push(value);
      }
      if (rows.length>0 && row[0]<=rows[rows.length-1][0]) throw "Klipper CSV coordinates must increase";
      rows.push(row);
      if (rows.length>1000000) throw "Klipper CSV exceeds sample limit";
    }
    if (header.length == 0 || rows.length<3) throw "Klipper CSV has no usable samples";
    return {header:header,rows:rows};
  }

  /** Small least-squares system with pivoting for amplitudes, phase and DC bias. */
  static function solve(rows:Array<Array<Float>>,ys:Array<Float>):Array<Float> {
    var a=[for (_ in 0...3) [0.0,0.0,0.0,0.0]];
    for (i in 0...rows.length) for (r in 0...3) {
      for (c in 0...3) a[r][c]+=rows[i][r]*rows[i][c];
      a[r][3]+=rows[i][r]*ys[i];
    }
    for (i in 0...3) {
      var pivot=i;
      for (j in i+1...3) if (Math.abs(a[j][i])>Math.abs(a[pivot][i])) pivot=j;
      if (!(Math.abs(a[pivot][i])>1e-20)) throw "Resonance trace has no identifiable mode";
      var swap=a[i]; a[i]=a[pivot]; a[pivot]=swap;
      var div=a[i][i]; for (c in i...4) a[i][c]/=div;
      for (j in 0...3) if (j!=i) { var scale=a[j][i]; for (c in i...4) a[j][c]-=scale*a[i][c]; }
    }
    return [for (i in 0...3) a[i][3]];
  }

  /** `start` and `end` are capture timestamps after excitation stopped, while motors hold position.
   * Shaping must be disabled during acquisition. The caller supplies this experimental fact; it is not guessed from a sweep.
   * Klipper's PSD CSV may be TEST_RESONANCES or SHAPER_CALIBRATE output (extra shaper columns are ignored). */
  public static function measure(axis:String,spectrum:String,decay:String,start:Float,end:Float,
      spectrumPath:String,decayPath:String):ResonanceMeasurement {
    if (axis!="x" && axis!="y") throw "Belt calibration measures Cartesian x or y";
    if (!Math.isFinite(start) || !Math.isFinite(end) || end<=start) throw "Free-decay interval must be finite and ordered";
    var raw=csv(decay), channel=raw.header.indexOf("accel_"+axis);
    if (raw.header[0]!="time" || channel<0) throw "Free-decay capture needs Klipper time/accel columns";
    if (start<raw.rows[0][0] || end>raw.rows[raw.rows.length-1][0]) throw "Free-decay interval lies outside the capture";
    var samples=[for (row in raw.rows) if (row[0]>=start && row[0]<=end) row];
    if (samples.length<40) throw "Free decay needs at least forty samples";
    var origin=samples[0][0], duration=samples[samples.length-1][0]-origin, dt=duration/(samples.length-1);
    for (i in 1...samples.length) if (Math.abs((samples[i][0]-samples[i-1][0])/dt-1)>0.02)
      throw "Free-decay sample clock is irregular or has gaps";
    var centre=0.0, scale=0.0;
    for (row in samples) centre+=row[channel]/samples.length;
    for (row in samples) scale=Math.max(scale,Math.abs(row[channel]-centre));
    if (!(scale>1e-9)) throw "Free-decay trace has no motion";
    var y=[for (row in samples) (row[channel]-centre)/scale];
    var psd=csv(spectrum), power=psd.header.indexOf("psd_"+axis);
    if (psd.header[0]!="freq" || power<0) throw "Resonance spectrum needs Klipper freq/psd columns";
    var peak=psd.rows[0], peakIndex=0;
    for (i in 0...psd.rows.length) {
      for (value in psd.rows[i]) if (value<0) throw "Resonance power/frequencies must be nonnegative";
      if (psd.rows[i][power]>peak[power]) { peak=psd.rows[i]; peakIndex=i; }
    }
    if (!(peak[power]>0) || peakIndex==0 || peakIndex==psd.rows.length-1) throw "Spectrum does not enclose a resonance peak";
    var bin=Math.max(peak[0]-psd.rows[peakIndex-1][0],psd.rows[peakIndex+1][0]-peak[0]);
    if (bin>peak[0]*0.1) throw "Spectrum is too coarse to identify the same mode";
    var band=Math.max(2*bin,0.15*peak[0]), low=Math.max(0.001,peak[0]-band), high=peak[0]+band;
    var energy=0.0; for (value in y) energy+=value*value;
    // Fit the continuous waveform, not a noisy autoregressive recurrence or a shaper's assumed damping.
    // For each f/zeta, amplitudes, phase and DC bias are solved linearly (variable projection).
    function errorAt(frequency:Float,damping:Float):Float {
      var omega=2*Math.PI*frequency, alpha=damping*omega, omegaD=omega*Math.sqrt(1-damping*damping);
      var basis=[for (row in samples) {
        var time=row[0]-origin, envelope=Math.exp(-alpha*time);
        [envelope*Math.cos(omegaD*time),envelope*Math.sin(omegaD*time),1.0];
      }];
      var wave=solve(basis,y), squared=0.0;
      for (i in 0...y.length) {
        var predicted=0.0; for (j in 0...3) predicted+=basis[i][j]*wave[j];
        squared+=(y[i]-predicted)*(y[i]-predicted);
      }
      return Math.sqrt(squared/energy);
    }
    var frequency=peak[0], damping=0.1, error=Math.POSITIVE_INFINITY;
    for (i in 0...9) for (zeta in [0.002,0.01,0.03,0.06,0.1,0.15,0.22,0.35,0.49]) {
      var hz=low+(high-low)*i/8, candidate=errorAt(hz,zeta);
      if (candidate<error) { frequency=hz; damping=zeta; error=candidate; }
    }
    var hzStep=(high-low)/8, dampingStep=0.08;
    for (_ in 0...56) {
      var nextFrequency=frequency, nextDamping=damping, best=error;
      for (i in -1...2) for (j in -1...2) {
        var hz=frequency+i*hzStep, zeta=damping+j*dampingStep;
        if (hz<low || hz>high || zeta<0.002 || zeta>=0.5) continue;
        var candidate=errorAt(hz,zeta);
        if (candidate<best) { nextFrequency=hz; nextDamping=zeta; best=candidate; }
      }
      if (best<error) { frequency=nextFrequency; damping=nextDamping; error=best; }
      else { hzStep*=0.5; dampingStep*=0.5; }
    }
    var omega=2*Math.PI*frequency, omegaD=omega*Math.sqrt(1-damping*damping);
    if (duration*omegaD/(2*Math.PI)<4 || omegaD*dt>Math.PI/4 || damping*omega*duration<0.4)
      throw "Free decay needs four resolved cycles, eight samples per cycle and an identifiable amplitude decrease";
    if (!(error<=0.05)) throw 'Free decay is not explained by one mode (relative RMS $error)';
    return {axis:axis,frequencyHz:frequency,dampingRatio:damping,fitError:error,
      spectrumSource:spectrumPath+"#sha256="+Sha256.encode(spectrum),decaySource:decayPath+"#sha256="+Sha256.encode(decay),decayStart:start,decayEnd:end};
  }
}
