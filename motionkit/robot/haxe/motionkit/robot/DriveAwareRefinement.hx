package motionkit.robot;

import MotionKitNative;
import TrajectoryCore;
import motionkit.kinematics.PathDriveLimits;

/** Native banded QP over redundancy values and path derivatives. The arm
 * supplies local linear bounds, then exact IK checks the resulting geometry. */
class DriveAwareRefinement {
  public static function fit(distances:Array<Float>,route:Array<Array<Float>>,seed:Array<Array<Float>>,
      lower:Array<Float>,upper:Array<Float>,velocity:Array<Float>,acceleration:Array<Float>,
      drives:PathDriveLimits,bounds:Array<mk_refinement_bound>,?gradient:Array<Array<Float>>):Array<RedundancyCurve> {
    var n=distances.length,d=route.length;
    if(n<2 || d<1 || seed.length!=d || lower.length!=d || upper.length!=d || velocity.length!=d || acceleration.length!=d)
      throw "Constrained refinement requires aligned redundant coordinates";
    var limits=new mk_refinement_limits();limits.set_struct_size(mk_refinement_limits.size());limits.set_coordinate_count(d);
    limits.set_route_weight(1);limits.set_seed_weight(10);limits.set_curvature_weight(0.01);
    for(j in 0...d){
      if(route[j].length!=n || seed[j].length!=n)throw "Refinement values must align with sample knots";
      limits.set_max_velocity(j,velocity[j]);limits.set_max_acceleration(j,acceleration[j]*0.8);
    }
    var samples:Array<mk_refinement_sample> = [];
    for(i in 0...n){
      var sample=new mk_refinement_sample();sample.set_struct_size(mk_refinement_sample.size());sample.set_s(distances[i]);
      sample.set_feed(drives.feed[i]);sample.set_feed_gradient(drives.feedGradient[i]);
      for(j in 0...d){sample.set_route(j,route[j][i]);sample.set_seed(j,seed[j][i]);
        sample.set_lower(j,lower[j]);sample.set_upper(j,upper[j]);sample.set_gradient(j,gradient==null?0:gradient[j][i]);}
      samples.push(sample);
    }
    var solved=MotionKitNative.mk_refine_redundancy(limits,samples,bounds);
    if(solved.status!=TrajectoryCoreConstants.MK_OK){
      for(j in 0...d){var from:Null<Float> = null,to:Null<Float> = null;
        for(bound in bounds)if(bound.get_derivative()==0 && bound.get_lower()==bound.get_upper()){
          var unit=true;for(c in 0...d)if(bound.get_coefficients(c)!=(c==j?1:0))unit=false;
          if(unit && bound.get_sample()==0)from=bound.get_lower();
          if(unit && bound.get_sample()==n-1)to=bound.get_lower();
        }
        if(from!=null && to!=null){var travel=Math.abs(cast(to,Float)-cast(from,Float)),seconds=0.0;
          for(i in 0...n-1)seconds+=(distances[i+1]-distances[i])/drives.feed[i];
          if(travel>velocity[j]*seconds*(1+1e-8))
            throw 'Refinement coordinate $j cannot sustain the planned feed; sustainable feed scale at most ${velocity[j]*seconds/travel}';
        }
      }
      if(solved.status==TrajectoryCoreConstants.MK_ERROR_LIMIT)throw "Drive-aware refinement QP did not converge within its deterministic iteration budget";
      throw 'Drive-aware refinement could not satisfy its constraints at the requested feed (native status ${solved.status}); change the path or feed';
    }
    return [for(j in 0...d)new CubicRedundancyCurve(distances,
      [for(i in 0...n)solved.out_solutions[i].get_value(j)],
      [for(i in 0...n)solved.out_solutions[i].get_first(j)],lower[j],upper[j])];
  }
  public static function bound(sample:Int,coefficients:Array<Float>,lower:Float,upper:Float,derivative:Int=0):mk_refinement_bound {
    var result=new mk_refinement_bound();result.set_struct_size(mk_refinement_bound.size());
    result.set_sample(sample);result.set_derivative(derivative);result.set_lower(lower);result.set_upper(upper);
    for(i in 0...coefficients.length)result.set_coefficients(i,coefficients[i]);return result;
  }
}
