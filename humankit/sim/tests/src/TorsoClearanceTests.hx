import humankit.rig.HumanBone;

/**
 * The worker stands at a rack and a table to reach the part; the front of the belly, below the
 * surface's top, must stay outside its footprint, leaning the upper body over it to make up the
 * reach instead of standing in the edge. Measured in the plane, against the footprint, while the
 * hand is out on a reach.
 */
class TorsoClearanceTests {
    static inline var BELLY_FRONT = 0.12;

    /** Signed distance from a point to a box's footprint: negative inside, by how far in. */
    public static function outside(point:Array<Float>, box:humankit.HumanTargetBox):Float {
        var c = Math.cos(box.yaw), s = Math.sin(box.yaw);
        var dx = point[0] - box.center[0], dy = point[1] - box.center[1];
        var lx = Math.abs(c * dx + s * dy) - box.halfExtents[0], ly = Math.abs(-s * dx + c * dy) - box.halfExtents[1];
        if (lx <= 0.0 && ly <= 0.0) return Math.max(lx, ly);
        return Math.sqrt(Math.max(lx, 0.0) * Math.max(lx, 0.0) + Math.max(ly, 0.0) * Math.max(ly, 0.0));
    }

    /** The shallowest the belly front came while reaching, and the deepest lean, over a whole job. */
    static function measure(restingOn:Bool):{clearance:Float, lean:Float} {
        var scenario = RackScenario.build(restingOn);
        var body = scenario.worker.body;
        var clearance = Math.POSITIVE_INFINITY, lean = 0.0, ticks = 0;
        while (!scenario.worker.currentJobDone() && ticks++ < 900) {
            scenario.tick();
            lean = Math.max(lean, body.character.spineLean());
            // Only while reaching: for the part at the rack, or holding it to place it on the table.
            // Walking back past a surface after the hand lets go is not standing at it.
            if (body.reachWeight(humankit.HumanLimb.ArmR) <= 0.0 || !(scenario.worker.currentStep() == 0 || body.grip))
                continue;
            var belly = body.character.pose.bonePosition(HumanBone.Spine);
            var front = body.toWorld([belly[0] + BELLY_FRONT, belly[1], belly[2]]);
            var now = Math.min(outside(front, scenario.rack), outside(front, scenario.table));
            if (false) Sys.println('TC tick=$ticks step=' + scenario.worker.currentStep() + ' clearance=' + now + ' lean=' + body.character.spineLean() + ' root=' + body.rootTransform()[12] + ' bellyModel=' + belly + ' front=' + front);
            clearance = Math.min(clearance, now);
        }
        if (!scenario.worker.currentJobDone()) throw "The clearance job did not finish";
        return {clearance: clearance, lean: lean};
    }

    public static function run():Void {
        var without = measure(false);
        var planned = measure(true);
        Sys.println('belly clearance: ${Math.round(planned.clearance * 1000) / 1000} m with the surface known (lean ${Math.round(planned.lean * 100) / 100} rad), ${Math.round(without.clearance * 1000) / 1000} m without');
        if (planned.clearance < JobGate.CLEARANCE)
            throw 'The belly front stood ${-planned.clearance} m inside the surface it reached over';
        if (planned.lean < 0.03)
            throw "The worker did not lean to make up the reach";
        // Knowing the surface must never put the chest nearer it. (It used to put it further: with the palm's depth
        // in the stand-off the worker is far enough back either way at this height.)
        if (planned.clearance < without.clearance - 0.001)
            throw 'Knowing the surface put the chest nearer it: ${planned.clearance} vs ${without.clearance}';
    }
}
