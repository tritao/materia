package motionkit.robot;

import MotionKitNative;
import TrajectoryCore;
import motionkit.robot.CartesianCandidateSampler.LatticeCandidate;

/** Copy one already-generated native layer and release its temporary storage. */
class NativeCandidateBatch {
  public static function decode(batch:Ownedmk_candidate_batch_handle,count:Int,joints:Int,externals:Int):Array<LatticeCandidate> {
    try {
      if(count==0){batch.close();return [];}
      var lengths=CartesianCandidateSampler.compactLengths(count,joints,externals);
      var result=MotionKitNative.mk_read_candidate_batch(batch.borrow(),lengths.joints,lengths.coordinates);
      if(result.status!=TrajectoryCoreConstants.MK_OK || result.out_count!=count)
        throw 'Native candidate batch read failed: ${result.status}';
      var decoded=CartesianCandidateSampler.decodeCompact(result.out_joints,result.out_wraps,result.out_coordinates,
        count,joints,externals);
      batch.close();
      return decoded;
    } catch(error:Dynamic){batch.close();throw error;}
  }
}
