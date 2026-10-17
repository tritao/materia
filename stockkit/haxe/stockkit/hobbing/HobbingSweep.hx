package stockkit.hobbing;
import StockKitNative;
import gearkit.HobDesign;
import gearkit.Point3;
import stockkit.RingStock;

/** Sweeps the actual planar rake regions through sampled rigid poses. A native
    convex-prism sweep supplies circle intervals; no target GearSpec or quality
    formula is passed to the cutter. The sampler must resolve motion between its
    time knots; maxTimeStep also caps sampling of nonlinear/perturbed trajectories. */
class HobbingSweep {
  public final hob:HobDesign;
  public final tolerance:Float;
  final owner:Ownedsk_rake_tool_handle;
  final probes:Array<Point3>;
  var disposed=false;
  public function new(hob:HobDesign,firstTooth:Int,lastTooth:Int,tolerance:Float=0.00005,?warp:Point3->Point3) {
    if(firstTooth>lastTooth||lastTooth-firstTooth>256||!Math.isFinite(tolerance)||tolerance<=0)
      throw "invalid hob sweep extent/tolerance";
    this.hob=hob;this.tolerance=tolerance;
    var vertices:Array<Float> =[],regions:Array<sk_rake_region> =[];
    var xmin=0.0,xmax=0.0,ymin=0.0,ymax=0.0,zmin=0.0,zmax=0.0;
    for(gash in 0...hob.gashes)for(tooth in firstTooth...lastTooth+1) {
      var thread=tooth%hob.starts;if(thread<0)thread+=hob.starts;
      var turn=Std.int((tooth-thread)/hob.starts);
      var raw=hob.rakeFaceRegion(gash,thread,turn);
      if(warp!=null)raw=[for(p in raw){var q=warp(p);
        if(!Math.isFinite(q.x)||!Math.isFinite(q.y)||!Math.isFinite(q.z))throw "non-finite rake profile error";q;}];
      // Keep polygon error below the swept-path convergence tolerance.
      var points=simplify(raw,tolerance/8);
      var region=new sk_rake_region();region.set_struct_size(sk_rake_region.size());
      region.set_first(Std.int(vertices.length/3));region.set_count(points.length);region.set_gash(gash);regions.push(region);
      for(p in points) {
        vertices.push(p.x);vertices.push(p.y);vertices.push(p.z);
        xmin=Math.min(xmin,p.x);xmax=Math.max(xmax,p.x);ymin=Math.min(ymin,p.y);ymax=Math.max(ymax,p.y);zmin=Math.min(zmin,p.z);zmax=Math.max(zmax,p.z);
      }
    }
    probes=[];for(x in [xmin,xmax])for(y in [ymin,ymax])for(z in [zmin,zmax])probes.push(new Point3(x,y,z));
    var created=StockKitNative.sk_rake_tool_create(vertices,regions,hob.gashes);
    check(created.status,"create rake tool");owner=created.out_tool;
  }
  /** Transactional job. Refine the whole swept path until per-circle angular
      symmetric differences, expressed as arc lengths, meet tolerance. On failure
      the incoming stock is restored. Sources identify the exact scheduled pass. */
  public function cut(stock:RingStock,axesAt:Float->HobbingAxes,passes:Array<GashPass>,
      maxRefinements:Int=3,maxTimeStep:Float=0.001,?motionKnots:Array<Float>):HobbingReport {
    if(disposed)throw "hob sweep has been disposed";
    if(maxRefinements<1||maxRefinements>8||!Math.isFinite(maxTimeStep)||maxTimeStep<=0)throw "invalid sweep refinement settings";
    var initial=stock.snapshot(),previous=RingStock.annulus(stock.grid),havePrevious=false;
    try {
      var poseCount=0;
      for(level in 0...maxRefinements+1) {
        stock.restore(initial);poseCount=0;
        var stepTolerance=tolerance/(4*Math.pow(4,level));
        for(pass in passes) {
          if(pass.gash>=hob.gashes)throw "scheduled gash is outside tool";
          var poses=sample(axesAt,pass.start,pass.end,pass.center,stepTolerance,maxTimeStep,motionKnots);
          check(StockKitNative.sk_ring_stock_sweep_rake(stock.borrowForSweep(),owner.borrow(),pass.gash,pass.pass,poses),"rake sweep");
          poseCount+=poses.length;
        }
        if(havePrevious) {
          var comparison=stock.compareStock(previous),worst=0.0;
          for(z in 0...stock.grid.axialCount)for(r in 0...stock.grid.radialCount) {
            var i=z*stock.grid.radialCount+r;
            worst=Math.max(worst,(comparison.leftover[i]+comparison.gouge[i])*stock.grid.radius(r));
          }
          if(worst<=tolerance) {
            initial.dispose();previous.dispose();
            return new HobbingReport(passes.length,poseCount,level,worst);
          }
        }
        var saved=stock.snapshot();previous.restore(saved);saved.dispose();havePrevious=true;
      }
      throw "hob sweep did not converge at the requested arc tolerance";
    } catch(error:Dynamic) {
      stock.restore(initial);initial.dispose();previous.dispose();throw error;
    }
  }
  /** One resolution, useful for convergence studies. Sampling still evaluates
      true motion, including the exact pass-centre time between control ticks. */
  public function cutPass(stock:RingStock,axesAt:Float->HobbingAxes,pass:GashPass,
      pathTolerance:Float,maxTimeStep:Float=0.001,?motionKnots:Array<Float>):Int {
    if(disposed)throw "hob sweep has been disposed";
    if(!Math.isFinite(pathTolerance)||pathTolerance<=0||!Math.isFinite(maxTimeStep)||maxTimeStep<=0)
      throw "invalid sweep path sampling";
    var poses=sample(axesAt,pass.start,pass.end,pass.center,pathTolerance,maxTimeStep,motionKnots);
    check(StockKitNative.sk_ring_stock_sweep_rake(stock.borrowForSweep(),owner.borrow(),pass.gash,pass.pass,poses),"rake sweep");
    return poses.length;
  }
  /** Identical poses for independent sampled stock domains. */
  public function cutPassMany(stocks:Array<RingStock>,axesAt:Float->HobbingAxes,pass:GashPass,
      pathTolerance:Float,maxTimeStep:Float=0.001,?motionKnots:Array<Float>):Int {
    if(disposed||stocks.length==0)throw "invalid multi-stock rake sweep";
    if(!Math.isFinite(pathTolerance)||pathTolerance<=0||!Math.isFinite(maxTimeStep)||maxTimeStep<=0)throw "invalid sweep path sampling";
    var poses=sample(axesAt,pass.start,pass.end,pass.center,pathTolerance,maxTimeStep,motionKnots);
    for(stock in stocks)check(StockKitNative.sk_ring_stock_sweep_rake(stock.borrowForSweep(),owner.borrow(),pass.gash,pass.pass,poses),"rake sweep");
    return poses.length;
  }
  function sample(at:Float->HobbingAxes,start:Float,end:Float,center:Float,tol:Float,maxStep:Float,knots:Null<Array<Float>>):Array<sk_rake_pose> {
    var result:Array<sk_rake_pose> =[];
    var a=at(start);result.push(a.pose());
    function split(t0:Float,s0:HobbingAxes,p0:Array<Float>,t1:Float,s1:HobbingAxes,p1:Array<Float>,depth:Int):Void {
      if(depth>24||result.length>1000000)throw "sweep path refinement exceeded limit";
      var tm=(t0+t1)/2,sm=at(tm),pm=sm.poseValues(),error=0.0;
      // Endpoints are cached through recursion. Identical affine arithmetic and
      // refinement tests, without reconstructing three native poses per node.
      var dr=[for(j in 0...9)pm[j]-(p0[j]+p1[j])/2];
      var dt=[for(k in 0...3)pm[9+k]-(p0[9+k]+p1[9+k])/2];
      for(v in probes) {
        var dx=dt[0]+dr[0]*v.x+dr[1]*v.y+dr[2]*v.z;
        var dy=dt[1]+dr[3]*v.x+dr[4]*v.y+dr[5]*v.z;
        var dz=dt[2]+dr[6]*v.x+dr[7]*v.y+dr[8]*v.z;
        error=Math.max(error,Math.sqrt(dx*dx+dy*dy+dz*dz));
      }
      var angle=Math.max(Math.abs(s1.b-s0.b),Math.max(Math.abs(s1.c-s0.c),Math.abs(s1.a-s0.a)));
      if(t1-t0>maxStep||angle>0.04||error>tol){split(t0,s0,p0,tm,sm,pm,depth+1);split(tm,sm,pm,t1,s1,p1,depth+1);}
      else result.push(HobbingAxes.nativePose(p1));
    }
    var times=[start,end];
    if(center>start&&center<end)times.push(center);
    if(knots!=null)for(t in knots) {
      if(!Math.isFinite(t))throw "motion knots must be finite";
      if(t>start&&t<end)times.push(t);
    }
    times.sort((a,b)->a<b?-1:a>b?1:0);
    for(k in 1...times.length)if(times[k]>times[k-1]){
      var s0=at(times[k-1]),s1=at(times[k]);
      split(times[k-1],s0,s0.poseValues(),times[k],s1,s1.poseValues(),0);
    }
    return result;
  }
  static function simplify(raw:Array<Point3>,tolerance:Float):Array<Point3> {
    var points=raw.copy();points.pop();var split=Std.int(points.length/2);
    function reduce(a:Int,b:Int):Array<Point3> {
      var first=points[a],last=points[b],dx=last.x-first.x,dy=last.y-first.y,dz=last.z-first.z;
      var length=dx*dx+dy*dy+dz*dz,worst=0.0,index=-1;
      for(k in a+1...b) {
        var p=points[k],t=length==0?0:Math.max(0,Math.min(1,((p.x-first.x)*dx+(p.y-first.y)*dy+(p.z-first.z)*dz)/length));
        var ex=p.x-first.x-t*dx,ey=p.y-first.y-t*dy,ez=p.z-first.z-t*dz,d=Math.sqrt(ex*ex+ey*ey+ez*ez);
        if(d>worst){worst=d;index=k;}
      }
      if(worst<=tolerance||index<0)return [first,last];
      var left=reduce(a,index),right=reduce(index,b);left.pop();return left.concat(right);
    }
    var a=reduce(0,split),b=reduce(split,points.length-1);a.pop();return a.concat(b);
  }
  public function dispose():Void {if(!disposed){disposed=true;owner.close();}}
  static function check(status:Int,operation:String):Void {if(status!=0)throw '$operation failed ($status)';}
}
