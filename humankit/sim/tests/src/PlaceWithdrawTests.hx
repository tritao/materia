import humankit.rig.HumanBone;

/**
 * After setting a part down, the worker withdraws the hand and walks away. The hand must stay in
 * front of the chest, where an arm can go, and the elbow below the shoulder: a wrist dragged
 * behind the chest plane folds the arm into a raised elbow.
 */
class PlaceWithdrawTests {
    static inline var MIN_AHEAD = 0.05;

    public static function run():Void {
        var scenario = RackScenario.build();
        var pose = scenario.worker.body.character.pose;
        var closest = Math.POSITIVE_INFINITY, highest = Math.NEGATIVE_INFINITY, ticks = 0, reached = 0;
        while (!scenario.worker.currentJobDone() && ticks++ < 900) {
            scenario.tick();
            // Only in the place step, and only while the hand is held fully out by a reach: as it is
            // released it blends into the gait, which carries it behind the chest.
            if (scenario.worker.currentStep() != 1 ||
                scenario.worker.body.reachWeight(humankit.HumanLimb.ArmR) < 1.0) continue;
            reached++;
            var chest = pose.bonePosition(HumanBone.Chest), hand = pose.bonePosition(HumanBone.HandR);
            var shoulder = pose.bonePosition(HumanBone.UpperArmR), elbow = pose.bonePosition(HumanBone.ForearmR);
            closest = Math.min(closest, hand[0] - chest[0]);
            highest = Math.max(highest, elbow[2] - shoulder[2]);
        }
        if (!scenario.worker.currentJobDone()) throw "The withdrawal job did not finish";
        if (reached < 60) throw 'The hand was held out by a reach for only $reached ticks';
        if (closest < MIN_AHEAD)
            throw 'The hand came ${closest} m in front of the chest; it should stay at least $MIN_AHEAD m ahead';
        if (highest > 0.0)
            throw 'The elbow rose ${highest} m above the shoulder';
        Sys.println('place withdrawal: hand stayed at least ${Math.round(closest * 1000) / 1000} m ahead of the chest, elbow below the shoulder');
    }
}
