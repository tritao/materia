package stockkit.hobbing;

import stockkit.RingStock;

/** Optional per-pass observer. A COW previous stock plus native interval comparison
    gives removed volume, without copying circle intervals into the managed heap.
    Call after EVERY pass in cutting order; unrelated stock edits invalidate attribution.
    Summary storage is bounded. Pass means are rectangular loads over their windows;
    maxima sum overlapping windows at each start. These are NOT tooth-force peaks. */
class HobbingForces {
  public final material:KienzleMaterial;
  public final edgeWidth:Float;
  public final workRadius:Float;
  public final pressureAngle:Float;
  public var passes(default,null):Int=0;
  public var volume(default,null):Float=0;
  public var energy(default,null):Float=0;
  public var maxTangential(default,null):Float=0;
  public var maxRadial(default,null):Float=0;
  public var maxHobTorque(default,null):Float=0;
  public var maxTableTorqueBound(default,null):Float=0;
  public var maxPower(default,null):Float=0;
  public var last(default,null):Null<HobbingForceSample>;
  final stock:RingStock;
  final previous:RingStock;
  var disposed=false;
  var lastStart=-1e300;
  final active:Array<HobbingForceSample> = [];
  public var maxGashTangential(default,null):Float=0;
  public var peakActiveWindows(default,null):Int=0;
  /** edgeWidth is an explicitly assumed effective engaged cutting-edge width, mm;
      interval volume alone cannot identify chip width and thickness separately. */
  public function new(stock:RingStock,workRadius:Float,pressureAngle:Float,edgeWidth:Float,?material:KienzleMaterial) {
    for(v in [workRadius,pressureAngle,edgeWidth])if(!Math.isFinite(v))throw "invalid force setup";
    if(workRadius<=0||pressureAngle<0||pressureAngle>=Math.PI/2||edgeWidth<=0)throw "invalid force setup";
    this.stock=stock;this.workRadius=workRadius;this.pressureAngle=pressureAngle;this.edgeWidth=edgeWidth;
    this.material=material==null?KienzleMaterial.referenceSteel():material;
    previous=RingStock.annulus(stock.grid);save();
  }
  function save():Void {var snapshot=stock.snapshot();try{previous.restore(snapshot);}catch(e:Dynamic){snapshot.dispose();throw e;}snapshot.dispose();}
  public function afterPass(pass:GashPass,start:HobbingAxes,end:HobbingAxes,hobRadius:Float):HobbingForceSample {
    if(disposed)throw "force observer disposed";
    if(pass.start<lastStart)throw "force windows must be observed in start-time order";
    var change=previous.compareStock(stock);
    if(change.gougeVolume()>1e-9)throw "force observer stock gained material";
    var travel=hobRadius*Math.abs(end.b-start.b);
    var sample=new HobbingForceSample(pass,change.leftoverVolume(),travel,hobRadius,workRadius,edgeWidth,pressureAngle,material);
    save();passes++;volume+=sample.removedVolume;energy+=sample.energy;last=sample;
    lastStart=pass.start;
    maxGashTangential=Math.max(maxGashTangential,sample.tangentialForce);
    var i=active.length;while(i>0){i--;if(active[i].pass.end<=pass.start)active.splice(i,1);}
    active.push(sample);peakActiveWindows=Std.int(Math.max(peakActiveWindows,active.length));
    if(active.length>4096)throw "force window budget exceeded";
    var force=0.0,radial=0.0,torque=0.0,table=0.0,power=0.0;
    for(window in active){force+=window.tangentialForce;radial+=window.radialForce;
      torque+=window.hobTorque;table+=window.tableTorqueBound;power+=window.power;}
    maxTangential=Math.max(maxTangential,force);maxRadial=Math.max(maxRadial,radial);
    maxHobTorque=Math.max(maxHobTorque,torque);maxTableTorqueBound=Math.max(maxTableTorqueBound,table);
    maxPower=Math.max(maxPower,power);return sample;
  }
  public function retainedBytes():Float return previous.bytes();
  public function dispose():Void {if(!disposed){disposed=true;previous.dispose();}}
}
