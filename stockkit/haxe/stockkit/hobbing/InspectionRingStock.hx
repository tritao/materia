package stockkit.hobbing;
import gearkit.GearInspection;
import gearkit.GearRings;
import stockkit.RingGrid;
import stockkit.RingStock;
import stockkit.RingInterval;

typedef InspectionSlice={first:Int,last:Int,low:Float,high:Float};

/** Physical rake stock only at the circles read by GearMeasurement: all radial
    samples at profile/pitch Z, all axial samples bracketing measurement radius.
    Circle subtraction is independent; no uncut circle or target boundary is
    supplied to inspection. Every retained coordinate must be bit-identical to
    the dense grid. Unsupported coordinates fail rather than fabricate coverage.
    Not a volume, load, mesh or force-feedback stock representation. */
class InspectionRingStock implements GearRings {
  public final grid:RingGrid;
  public final stocks:Array<RingStock>;
  final radial:InspectionSlice;final axial:InspectionSlice;
  function new(grid:RingGrid,radial:InspectionSlice,axial:InspectionSlice,profile:RingStock,lead:RingStock){
    this.grid=grid;this.radial=radial;this.axial=axial;stocks=[profile,lead];
  }
  static function bracket(values:Array<Float>,value:Float):Int {
    if(value<values[0]||value>values[values.length-1])return -1;
    var lo=0,hi=values.length-1;while(hi-lo>1){var mid=(lo+hi)>>1;if(values[mid]<=value)lo=mid;else hi=mid;}return lo;
  }
  static function slice(min:Float,max:Float,step:Float,values:Array<Float>,first:Int,last:Int):Null<InspectionSlice> {
    // Additional border cells may make the translated grid arithmetic exact.
    // Fall back to dense computation if a compact exact slice is unavailable.
    function ulp(v:Float):Float return v==0?0:Math.pow(2,Math.floor(Math.log(Math.abs(v))/Math.log(2))-52);
    for(padding in 0...9){var lo=Std.int(Math.max(0,first-padding)),hi=Std.int(Math.min(values.length-1,last+padding));
      var baseLow=min+lo*step,baseHigh=hi==values.length-1?max:min+(hi+1)*step;
      // Rounded domain bounds can otherwise move a retained centre by one ULP.
      // Search nearby bounds, accepting ONLY exact original sample coordinates.
      for(da in -2...3)for(db in -2...3){
        var a=baseLow+da*ulp(baseLow),b=baseHigh+db*ulp(baseHigh),delta=(b-a)/(hi-lo+1),same=true;
        for(i in lo...hi+1)if(a+(i-lo+0.5)*delta!=values[i]){same=false;break;}
        if(same)return {first:lo,last:hi,low:a,high:b};
      }
    }
    return null;
  }
  public static function create(grid:RingGrid,cfg:GearInspection):Null<InspectionRingStock> {
    if(grid.radialCount<2||grid.axialCount<2)return null;
    var rs=[for(i in 0...grid.radialCount)grid.radius(i)],zs=[for(i in 0...grid.axialCount)grid.z(i)];
    var r=bracket(rs,cfg.measurementRadius),a=bracket(zs,cfg.profileZ),b=bracket(zs,cfg.pitchZ);
    if(r<0||a<0||b<0)return null;
    var radial=slice(grid.radiusMin,grid.radiusMax,grid.radialStep,rs,r,r+1);
    var axial=slice(grid.zMin,grid.zMax,grid.axialStep,zs,Std.int(Math.min(a,b)),Std.int(Math.max(a,b))+1);
    if(radial==null||axial==null)return null;
    var pg=new RingGrid(grid.radiusMin,grid.radiusMax,axial.low,axial.high,grid.radialStep,(axial.high-axial.low)/(axial.last-axial.first+1));
    var lg=new RingGrid(radial.low,radial.high,grid.zMin,grid.zMax,(radial.high-radial.low)/(radial.last-radial.first+1),grid.axialStep);
    if(pg.radialCount!=grid.radialCount||pg.axialCount!=axial.last-axial.first+1
      ||lg.radialCount!=radial.last-radial.first+1||lg.axialCount!=grid.axialCount)return null;
    for(i in 0...pg.radialCount)if(pg.radius(i)!=rs[i])return null;
    for(i in 0...pg.axialCount)if(pg.z(i)!=zs[i+axial.first])return null;
    for(i in 0...lg.radialCount)if(lg.radius(i)!=rs[i+radial.first])return null;
    for(i in 0...lg.axialCount)if(lg.z(i)!=zs[i])return null;
    var profile=RingStock.annulus(pg);
    try{return new InspectionRingStock(grid,radial,axial,profile,RingStock.annulus(lg));}
    catch(e:Dynamic){profile.dispose();throw e;}
  }
  public function radii():Array<Float> return [for(i in 0...grid.radialCount)grid.radius(i)];
  public function axialPositions():Array<Float> return [for(i in 0...grid.axialCount)grid.z(i)];
  public function contains(r:Int,z:Int):Bool return (z>=axial.first&&z<=axial.last)||(r>=radial.first&&r<=radial.last);
  public function circle(r:Int,z:Int):Array<RingInterval> {
    grid.radius(r);grid.z(z);
    if(z>=axial.first&&z<=axial.last)return stocks[0].circle(r,z-axial.first);
    if(r>=radial.first&&r<=radial.last)return stocks[1].circle(r-radial.first,z);
    throw 'uncomputed inspection circle ($r,$z)';
  }
  public function boundaries(r:Int,z:Int):Array<Float> {
    var out:Array<Float> = [];for(a in circle(r,z)){out.push(a.lo);out.push(a.hi);}return out;
  }
  public function assertOverlap():Void {
    for(r in radial.first...radial.last+1)for(z in axial.first...axial.last+1){
      var a=stocks[0].circle(r,z-axial.first),b=stocks[1].circle(r-radial.first,z);
      if(a.length!=b.length)throw "inspection slice overlap topology differs";
      for(i in 0...a.length)if(a[i].lo!=b[i].lo||a[i].hi!=b[i].hi||a[i].loGash!=b[i].loGash||a[i].hiGash!=b[i].hiGash||a[i].loPass!=b[i].loPass||a[i].hiPass!=b[i].hiPass)
        throw "inspection slice overlap endpoint/provenance differs";
    }
  }
  public function dispose():Void for(stock in stocks)stock.dispose();
}
