import humankit.HumanHand;

/**
 * The worker closes its fingers to the size of the part it picks up: the same rack job with parts of
 * different sizes must finish, and the grasp must follow the part. A thin part is pinched, and a
 * thicker one closes the fingers further. The sizes are kept within what the fingers can reach: the
 * depth of a cube along a tilted palm is up to 1.7 times its side, and the fingertips only get about
 * 7 cm deep, so a part much larger than 4 cm closes the hand as far as it goes and no further.
 */
class GraspShapeTests {
    static var halves:Array<Float> = [0.008, 0.015, 0.02];

    public static function run():Void {
        var index:Array<Float> = [], pinky:Array<Float> = [], thumb:Array<Float> = [];
        for (half in halves) {
            var scenario = RackScenario.build(true, {hand: "right", surface: 1.06, yaw: 0.0, partHalf: half});
            var worker = scenario.worker, body = worker.body, limb = scenario.limb();
            var grasp:Null<Array<Float>> = null, ticks = 0, carried = 0;
            while (!worker.currentJobDone() && ticks++ < 900) {
                scenario.tick();
                if (grasp == null && body.isCarrying(limb)) carried++;
                if (grasp == null && carried > 40) grasp = body.grasp(limb);
            }
            var label = 'a part ${Math.round(half * 200) / 100} cm thick';
            if (!worker.currentJobDone() || worker.currentJobFailure() != null)
                throw '$label: the job did not finish: ${worker.currentJobFailure()}';
            if (grasp == null) throw '$label: the fingers were given no shape';
            index.push(grasp[HumanHand.INDEX]);
            pinky.push(grasp[HumanHand.PINKY]);
            thumb.push(grasp[HumanHand.THUMB]);
            if (half < 0.0095 && (grasp[HumanHand.MIDDLE] > body.posture.relaxedCurl + 1e-6 || grasp[HumanHand.PINKY] > body.posture.relaxedCurl + 1e-6))
                throw '$label was not pinched: $grasp';
        }
        if (!(index[0] < index[1] && index[1] < index[2]))
            throw 'The index finger did not close further on thicker parts: $index';
        Sys.println('grasp shape: index ${[for (v in index) Math.round(v * 100) / 100]}, pinky ${[for (v in pinky) Math.round(v * 100) / 100]}, thumb ${[for (v in thumb) Math.round(v * 100) / 100]} for parts 1.6, 3 and 4 cm on a side');
    }
}
