package gearkit;

/** Uniformly spaced roll/axial abscissa in mm; material-positive deviations in micrometres.
    Gaussian endpoints use linear residual extension; raw values always retained. */
class GearTrace {
  public final tooth:Int;
  public final flank:Int;
  public final x:Array<Float>;
  public final raw:Array<Float>;
  public final filtered:Array<Float>;
  public final cutoff:Float;
  public final total:Float;
  public final form:Float;
  public final slope:Float;
  public final gradient:Float;
  public final intercept:Float;
  public function new(tooth:Int,flank:Int,x:Array<Float>,y:Array<Float>,cutoff:Float,
      start:Float,end:Float,slopeLength:Float) {
    if (x.length != y.length || x.length < 2) throw "trace requires at least two samples";
    for(i in 0...x.length) if(!Math.isFinite(x[i]) || !Math.isFinite(y[i]) || (i>0 && x[i]<=x[i-1]))
      throw "trace samples must be finite and increasing";
    this.tooth=tooth; this.flank=flank; this.x=x.copy(); raw=y.copy(); this.cutoff=cutoff;
    filtered=gaussian(x,y,cutoff);
    var sx=0.0,sy=0.0,n=0;
    for(i in 0...x.length) if(x[i]>=start-1e-10 && x[i]<=end+1e-10){sx+=x[i];sy+=filtered[i];n++;}
    if(n<2)throw "insufficient points in evaluation window";
    var mx=sx/n,my=sy/n,xx=0.0,xy=0.0;
    for(i in 0...x.length)if(x[i]>=start-1e-10&&x[i]<=end+1e-10){xx+=(x[i]-mx)*(x[i]-mx);xy+=(x[i]-mx)*(filtered[i]-my);}
    gradient=xy/xx;intercept=my-gradient*mx;slope=gradient*slopeLength;
    var lo=Math.POSITIVE_INFINITY,hi=Math.NEGATIVE_INFINITY;
    var flo=Math.POSITIVE_INFINITY,fhi=Math.NEGATIVE_INFINITY;
    for(i in 0...x.length){
      var residual=filtered[i]-intercept-gradient*x[i];
      if(x[i]>=start-1e-10&&x[i]<=end+1e-10){lo=Math.min(lo,filtered[i]);flo=Math.min(flo,residual);}
      // Outside the window, only excess material contributes to the upper envelope.
      if ((x[i]>=start-1e-10&&x[i]<=end+1e-10)||filtered[i]>0) {
        hi=Math.max(hi,filtered[i]);fhi=Math.max(fhi,residual);
      }
    }
    total=hi-lo;form=fhi-flo;
  }
  public static function gaussian(x:Array<Float>, y:Array<Float>, cutoff:Float):Array<Float> {
    if(x.length!=y.length||x.length<2||!Math.isFinite(cutoff)||cutoff<0)throw "invalid Gaussian samples or cutoff";
    for(i in 0...x.length)if(!Math.isFinite(x[i])||!Math.isFinite(y[i])||(i>0&&x[i]<=x[i-1]))throw "invalid Gaussian axis/data";
    if(cutoff==0)return y.copy();
    var dx=(x[x.length-1]-x[0])/(x.length-1);
    for(i in 1...x.length)if(Math.abs((x[i]-x[i-1])-dx)>1e-6*dx)throw "Gaussian needs uniform samples";
    // Fourier response exp(-2*pi^2*sigma^2/lambda^2) = 1/2 at lambda=cutoff.
    var sigma=cutoff*Math.sqrt(Math.log(2))/(Math.sqrt(2)*Math.PI);
    var reach=Std.int(Math.ceil(4.5*sigma/dx));
    if(reach>100000)throw "filter support exceeds limit";
    var weights=[for(k in 0...reach+1)Math.exp(-0.5*Math.pow(k*dx/sigma,2))];
    var fit=regression(x,y),out:Array<Float> = [];
    // Extend the residual with zero rather than reflect the slope at either edge.
    for(i in 0...x.length){var sum=0.0,w=0.0;
      for(k in -reach...reach+1){var j=i+k,weight=weights[Std.int(Math.abs(k))];w+=weight;
        if(j>=0&&j<y.length)sum+=weight*(y[j]-fit[0]-fit[1]*x[j]);}
      out.push(fit[0]+fit[1]*x[i]+sum/w);
    }
    return out;
  }
  static function regression(x:Array<Float>,y:Array<Float>):Array<Float>{
    var mx=0.0,my=0.0;for(i in 0...x.length){mx+=x[i];my+=y[i];}mx/=x.length;my/=x.length;
    var xx=0.0,xy=0.0;for(i in 0...x.length){xx+=(x[i]-mx)*(x[i]-mx);xy+=(x[i]-mx)*(y[i]-my);}
    return [my-xy/xx*mx,xy/xx];
  }
}
