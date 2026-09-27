package motionkit.robot;

import MotionKitNative;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.Pose3;

/** Chooses one continuous joint configuration per Cartesian path sample. */
class PathConfigurationSelector {
  public final solver:KinematicsSolver;
  public final lower:Array<Float>;
  public final upper:Array<Float>;
  public final maxJump:Array<Float>;
  public final velocity:Array<Float>;
  public final preferred:Null<Array<Float>>;
  public final threads:Int;
  public final maxCandidates:Int;

  public function new(solver:KinematicsSolver, lower:Array<Float>, upper:Array<Float>,
      maxJump:Array<Float>, velocity:Array<Float>, ?preferred:Array<Float>,
      ?threads:Int = 1, ?maxCandidates:Int = 32) {
    if (solver == null || threads < 1 || maxCandidates < 1)
      throw "Configuration selector needs a solver, threads, and candidates";
    var count = solver.jointCount();
    for (values in [lower, upper, maxJump, velocity])
      if (values == null || values.length != count)
        throw "Configuration selector limit counts must match joints";
    if (preferred != null && preferred.length != count)
      throw "Configuration selector posture count must match joints";
    for (joint in 0...count) {
      if (!Math.isFinite(lower[joint]) || !Math.isFinite(upper[joint]) ||
          lower[joint] > upper[joint] || !Math.isFinite(maxJump[joint]) ||
          maxJump[joint] <= 0.0 || !Math.isFinite(velocity[joint]) ||
          velocity[joint] <= 0.0 ||
          preferred != null && !Math.isFinite(preferred[joint]))
        throw 'Configuration selector has invalid limits at joint $joint';
    }
    this.solver = solver;
    this.lower = lower.copy();
    this.upper = upper.copy();
    this.maxJump = maxJump.copy();
    this.velocity = velocity.copy();
    this.preferred = preferred == null ? null : preferred.copy();
    this.threads = threads;
    this.maxCandidates = maxCandidates;
  }

  /** Samples all poses, then crosses the native ABI once for the whole path. */
  public function selectPoses(distances:Array<Float>, poses:Array<Pose3>,
      startQ:Array<Float>, tolerance:IkTolerance):Array<Array<Float>> {
    if (distances == null || poses == null || distances.length != poses.length ||
        distances.length == 0 || startQ == null || startQ.length != solver.jointCount() ||
        tolerance == null)
      throw "Configuration selector needs aligned path samples and start joints";
    if (Std.isOfType(solver, OpwKinematics)) {
      var opw:OpwKinematics = cast solver;
      var nativeSamples:Array<mk_opw_path_sample> = [];
      for (index in 0...poses.length)
        nativeSamples.push(opw.nativePathSample(distances[index], poses[index]));
      var selected = MotionKitNative.mk_select_opw_configurations(
        nativeRequest(), opw.nativeParameters(), nativeSamples, startQ);
      return readResult(selected.status, selected.out_sequence);
    }
    var candidates:Array<Array<Array<Float>>> = [];
    for (index in 0...poses.length)
      candidates.push(index == 0 ? [startQ.copy()] :
        solver.sampleCandidates(poses[index], maxCandidates, tolerance));
    return select(distances, candidates);
  }

  /** Candidate sets are grouped by sample and passed to Descartes in bulk. */
  public function select(distances:Array<Float>, sets:Array<Array<Array<Float>>>):Array<Array<Float>> {
    if (distances == null || sets == null || distances.length != sets.length ||
        distances.length == 0) throw "Configuration selection needs aligned samples";
    var request = nativeRequest();
    var nativeSamples:Array<mk_configuration_sample> = [];
    var nativeCandidates:Array<mk_configuration_candidate> = [];
    var previous = Math.NEGATIVE_INFINITY;
    for (index in 0...sets.length) {
      var distance = distances[index];
      if (!Math.isFinite(distance) || distance <= previous || sets[index] == null)
        throw 'Configuration selection has an invalid sample at index $index';
      var sample = new mk_configuration_sample();
      sample.set_struct_size(mk_configuration_sample.size());
      sample.set_distance(distance);
      sample.set_first_candidate(nativeCandidates.length);
      sample.set_candidate_count(sets[index].length);
      nativeSamples.push(sample);
      for (candidate in sets[index]) {
        if (candidate == null || candidate.length != solver.jointCount())
          throw 'Configuration selection has an invalid candidate at sample $index';
        var native = new mk_configuration_candidate();
        native.set_struct_size(mk_configuration_candidate.size());
        for (joint in 0...solver.jointCount())
          native.set_joints(joint, candidate[joint]);
        nativeCandidates.push(native);
      }
      previous = distance;
    }
    var result = MotionKitNative.mk_select_configurations(request,
      nativeSamples, nativeCandidates);
    return readResult(result.status, result.out_sequence);
  }

  function nativeRequest():mk_configuration_request {
    var request = new mk_configuration_request();
    request.set_struct_size(mk_configuration_request.size());
    request.set_joint_count(solver.jointCount());
    request.set_threads(threads);
    request.set_has_preferred(preferred == null ? 0 : 1);
    for (joint in 0...solver.jointCount()) {
      request.set_lower(joint, lower[joint]);
      request.set_upper(joint, upper[joint]);
      request.set_max_jump(joint, maxJump[joint]);
      request.set_weights(joint, 1.0 / velocity[joint]);
      request.set_preferred(joint, preferred == null ? 0.0 : preferred[joint]);
    }
    return request;
  }

  function readResult(status:Int,
      sequence:Array<mk_configuration_solution>):Array<Array<Float>> {
    if (status != MotionKitNativeConstants.MK_OK) {
      var message = "";
      if (sequence != null && sequence.length > 0) {
        var bytes = sequence[0];
        var buffer = new StringBuf();
        for (index in 0...256) {
          var character = bytes.get_diagnostic(index);
          if (character == 0) break;
          buffer.addChar(character);
        }
        message = buffer.toString();
      }
      throw message.length > 0 ? message :
        'Configuration selection failed with MotionKit status $status';
    }
    return [for (value in sequence)
      [for (joint in 0...solver.jointCount()) value.get_joints(joint)]];
  }
}
