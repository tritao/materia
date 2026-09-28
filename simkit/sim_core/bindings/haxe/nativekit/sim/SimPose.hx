package nativekit.sim;

import NativeKitSim;

/** A rigid pose: position in metres and a unit x, y, z, w quaternion. */
class SimPose {
    public final x:Float;
    public final y:Float;
    public final z:Float;
    public final qx:Float;
    public final qy:Float;
    public final qz:Float;
    public final qw:Float;

    public function new(x:Float, y:Float, z:Float, qx:Float = 0.0, qy:Float = 0.0,
            qz:Float = 0.0, qw:Float = 1.0) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.qx = qx;
        this.qy = qy;
        this.qz = qz;
        this.qw = qw;
    }

    public function toNative():nksim_pose {
        var value = new nksim_pose();
        value.set_struct_size(nksim_pose.size());
        value.set_position(0, x);
        value.set_position(1, y);
        value.set_position(2, z);
        value.set_rotation(0, qx);
        value.set_rotation(1, qy);
        value.set_rotation(2, qz);
        value.set_rotation(3, qw);
        return value;
    }

    public static function fromNative(value:nksim_pose):SimPose
        return new SimPose(value.get_position(0), value.get_position(1), value.get_position(2),
            value.get_rotation(0), value.get_rotation(1), value.get_rotation(2),
            value.get_rotation(3));
}
