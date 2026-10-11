package processkit.screwdriving;

import robotkit.model.RobotModel;
import robotkit.model.LinkId;
import robotkit.model.FrameId;
import robotkit.model.JointId;
import robotkit.model.Frame;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.KinematicGroup;
import robotkit.kinematics.RobotKinematics;
import kinematicskit.KinematicState;
import kinematicskit.KinematicSnapshot;

/** The arm plans the free bit position without commanding the process-owned slide or spindle. */
class ScrewdriverArmKinematics {
  public static function create(model:RobotModel, root:LinkId, tcp:FrameId, feedId:JointId, spindleId:JointId):Manipulator {
    var feed:Null<Joint> = null, spindle:Null<Joint> = null, tip:Null<Frame> = null;
    for (joint in model.joints) {
      if (joint.id == feedId) feed = joint;
      if (joint.id == spindleId) spindle = joint;
    }
    for (frame in model.frames) if (frame.id == tcp) tip = frame;
    if (feed == null || spindle == null || tip == null) throw "Screwdriver arm needs its declared feed, spindle and TCP";
    if (feed.type != JointType.Prismatic || spindle.type != JointType.Continuous)
      throw "Screwdriver arm needs a prismatic feed and continuous spindle";
    var path = KinematicGroup.walk(model,feed.parent.id,tip.link.id);
    var movable = [for (joint in path) if (joint.type != JointType.Fixed) joint];
    if (movable.length != 2 || movable[0] != feed || movable[1] != spindle)
      throw "Screwdriver TCP chain must contain only its declared feed then spindle";
    var id:FrameId = Std.string(tcp)+":uncompressed";
    for (frame in model.frames) if (frame.id == id) throw 'Screwdriver planning frame "$id" already exists';
    var compiled = RobotKinematics.compile(model);
    var zero = KinematicSnapshot.of(new KinematicState(compiled));
    var parent = RobotKinematics.toTransform3(zero.bodyPose(compiled.bodyIndex(feed.parent.id)));
    var tipPose = RobotKinematics.toTransform3(zero.framePose(compiled.frameIndex(tcp)));
    var mount = parent.inverse().compose(tipPose);
    var fixedTip = new Frame(Std.string(id),feed.parent,id);
    fixedTip.position = mount.translation.toArray();
    fixedTip.rotation = [mount.rotation.x,mount.rotation.y,mount.rotation.z,mount.rotation.w];
    model.addFrame(fixedTip);
    return new Manipulator(model,root,fixedTip.id);
  }
  /** Rebuild the free-bit planning frame at the measured shaft phase, without commanding either process coordinate. */
  public static function atSpindle(model:RobotModel, root:LinkId, tcp:FrameId, feedId:JointId, spindleId:JointId, shaft:Float):Manipulator {
    if (!Math.isFinite(shaft)) throw "Screwdriver planning needs a finite measured spindle position";
    var feeds = [for (joint in model.joints) if (joint.id == feedId) joint];
    var frames = [for (frame in model.frames) if (frame.id == Std.string(tcp)+":uncompressed") frame];
    if (feeds.length != 1 || frames.length != 1) throw "Screwdriver planning lost its declared feed or planning frame";
    var compiled = RobotKinematics.compile(model), state = new KinematicState(compiled);
    state.q[compiled.dofIndex(Std.string(feedId))] = 0;
    state.q[compiled.dofIndex(Std.string(spindleId))] = shaft;
    var snapshot = KinematicSnapshot.of(state);
    var parent = RobotKinematics.toTransform3(snapshot.bodyPose(compiled.bodyIndex(feeds[0].parent.id)));
    var tip = RobotKinematics.toTransform3(snapshot.framePose(compiled.frameIndex(tcp)));
    var mount = parent.inverse().compose(tip), fixedTip = frames[0];
    fixedTip.position = mount.translation.toArray();
    fixedTip.rotation = [mount.rotation.x,mount.rotation.y,mount.rotation.z,mount.rotation.w];
    return new Manipulator(model,root,fixedTip.id);
  }

}
