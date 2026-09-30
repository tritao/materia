import humankit.HumanBone;

/**
 * The worker stands at a rack and a table to reach the part; the front of the belly, below the
 * surface's top, must stay outside its footprint, leaning the upper body over it to make up the
 * reach instead of standing in the edge. Measured in the plane, against the footprint, while the
 * hand is out on a reach.
 */
class TorsoClearanceTests {
    static inline var BELLY_FRONT = 0.12;

    /** Signed distance from a point to a rectangle's footprint: negative inside, by how far in. */
    static function outside(point:Array<Float>, center:Array<Float>, half:Array<Float>):Float {
        var dx = Math.abs(point[0] - center[0]) - half[0], dy = Math.abs(point[1] - center[1]) - half[1];
        if (dx <= 0.0 && dy <= 0.0) return Math.max(dx, dy);
        return Math.sqrt(Math.max(dx, 0.0) * Math.max(dx, 0.0) + Math.max(dy, 0.0) * Math.max(dy, 0.0));
    }

    /** The shallowest the belly front came while reaching, and the deepest lean, over a whole job. */
    static function measure(restingOn:Bool):{clearance:Float, lean:Float} {
        var scenario = RackScenario.build(restingOn);
        var body = scenario.worker.body;
        var rack = {center: [scenario.partStart[0], scenario.partStart[1]], half: [0.2, 0.2]};
        var table = {center: [scenario.partStart[0] + 1.0, scenario.partStart[1]], half: [0.2, 0.2]};
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
            var now = Math.min(outside(front, rack.center, rack.half), outside(front, table.center, table.half));
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
        if (planned.clearance < 0.0)
            throw 'The belly front stood ${-planned.clearance} m inside the surface it reached over';
        if (planned.lean < 0.03)
            throw "The worker did not lean to make up the reach";
        if (!(planned.clearance > without.clearance + 0.02))
            throw 'Knowing the surface did not keep the chest any further from it: ${planned.clearance} vs ${without.clearance}';
    }
}
