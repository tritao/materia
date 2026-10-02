import humankit.rig.HumanBone;
import humankit.rig.Mat4;

/**
 * Diagnostic: one rack-to-table layout with a hinge allowed together with a crouch or kneel, printing the arm's bend
 * plane around the moment it turns fastest. HUMANKIT_PROBE=hand,surface,half,yaw.
 */
class ArmFlipProbe {
    public static function run(spec:String):Void {
        var parts = spec.split(",");
        var layout:RackLayout = {hand: parts[0], surface: Std.parseFloat(parts[1]), yaw: Std.parseFloat(parts[3]),
            surfaceHalf: Std.parseFloat(parts[2]), character: "quaternius-ual/ual-work.glb"};
        var scenario = RackScenario.build(true, layout);
        var worker = scenario.worker;
        worker.body.posture.hingeWithCrouch = true;
        var gate = new JobGate(worker, scenario.session, scenario.limbs(), [scenario.rack, scenario.table]);
        var rows:Array<String> = [], ticks = 0;
        var tracked = Sys.getEnv("PROBE_SIDE") != null ? Sys.getEnv("PROBE_SIDE") : parts[0];
        var side = tracked == "left" ? "L" : "R";
        var before:Null<Array<Float>> = null;
        var lastRoot:Null<Array<Float>> = null;
        var lastHand:Null<Array<Float>> = null, lastShoulder:Null<Array<Float>> = null;
        var worstRate = 0.0, worstAt = -1;
        var rates:Array<Float> = [];
        while (!worker.currentJobDone() && ticks++ < 1500) {
            Sys.stderr().writeString('TICK ${ticks}\n'); Sys.stderr().flush();
            scenario.tick();
            gate.sample();
            var pose = worker.body.character.pose;
            var shoulder = pose.bonePosition(side == "L" ? HumanBone.UpperArmL : HumanBone.UpperArmR);
            var elbow = pose.bonePosition(side == "L" ? HumanBone.ForearmL : HumanBone.ForearmR);
            var hand = pose.bonePosition(side == "L" ? HumanBone.HandL : HumanBone.HandR);
            var upper = Mat4.subtract(shoulder, elbow), lower = Mat4.subtract(hand, elbow);
            var angle = Math.acos(Math.max(-1.0, Math.min(1.0, Mat4.dot(upper, lower) / Math.max(1e-9, len(upper) * len(lower))))) * 180.0 / Math.PI;
            var across = Mat4.cross(Mat4.subtract(elbow, shoulder), lower);
            var normal = angle < 175.0 && len(across) > 1e-9 ? Mat4.normalize(across) : null;
            var rate = 0.0;
            if (normal != null && before != null) rate = Math.acos(Math.max(-1.0, Math.min(1.0, Mat4.dot(normal, before)))) / 0.01;
            before = normal;
            rates.push(rate);
            if (rate > worstRate) { worstRate = rate; worstAt = rows.length; }
            var handSpeed = 0.0, shoulderSpeed = 0.0;
            if (lastHand != null) { handSpeed = len(Mat4.subtract(hand, lastHand)) / 0.01; shoulderSpeed = len(Mat4.subtract(shoulder, lastShoulder)) / 0.01; }
            lastHand = hand; lastShoulder = shoulder;
            var travel = worker.body.rootTransform();
            var travelDirection = lastRoot == null ? 0.0 : Math.atan2(travel[13] - lastRoot[1], travel[12] - lastRoot[0]);
            var facingNow = Math.atan2(travel[1], travel[0]);
            var crab = lastRoot == null || Math.abs(travel[13] - lastRoot[1]) + Math.abs(travel[12] - lastRoot[0]) < 1e-5 ? 0.0 : Math.atan2(Math.sin(travelDirection - facingNow), Math.cos(travelDirection - facingNow)) * 180.0 / Math.PI;
            lastRoot = [travel[12], travel[13]];
            var feetInfo = "";
            if (Sys.getEnv("PROBE_FEET") != null) {
                var footL = worker.body.toWorld(pose.bonePosition(HumanBone.FootL)), footR = worker.body.toWorld(pose.bonePosition(HumanBone.FootR));
                feetInfo = ' footL=${footL.map(f).join(",")} footR=${footR.map(f).join(",")} lock=${f(worker.body.footHold())} crab=${f(crab)}deg walking=${worker.body.walker.isWalking()} turning=${worker.body.walker.isTurning()} share=${f(worker.body.walker.stanceShare())}';
            }
            rows.push('$ticks ${worker.currentActionLabel()} crouch=${f(worker.body.crouchAmount())} kneel=${f(worker.body.kneelAmount())} hinge=${f(worker.body.character.spineHinge())} lean=${f(worker.body.character.spineLean())} elbow=${f(angle)}deg reach=${f(len(Mat4.subtract(hand, shoulder)))} handSpeed=${f(handSpeed)} shoulderSpeed=${f(shoulderSpeed)} clearance=${f(gate.clearance)} root=${f(worker.body.rootTransform()[12])},${f(worker.body.rootTransform()[13])} spineX=${f(pose.bonePosition(HumanBone.Spine)[0])} pelvisX=${f(pose.bonePosition(HumanBone.Pelvis)[0])} reachW=${f(worker.body.reachWeight(humankit.HumanLimb.ArmL))}$feetInfo rate=${f(rate)} normal=${normal == null ? "-" : normal.map(f).join(",")} shoulder=${shoulder.map(f).join(",")} elbowPos=${elbow.map(f).join(",")} hand=${hand.map(f).join(",")}');
        }
        var failures:Array<String> = [];
        gate.check("probe", failures);
        Sys.println('GATE ${spec}: worst plane turn ${f(gate.worst().turn)} rad/s; ' + (failures.length == 0 ? "no failures" : failures.join(" | ")));
        Sys.println('PROBE ${spec}: job done=${worker.currentJobDone()} failure=${worker.currentJobFailure()} worst plane turn ${f(worstRate)} rad/s at row $worstAt of ${rows.length}');
        var centre = parts.length > 4 ? Std.parseInt(parts[4]) - 1 : worstAt;
        for (row in (Sys.getEnv("PROBE_ALL") != null ? 0 : Std.int(Math.max(0, centre - 60)))...(Sys.getEnv("PROBE_ALL") != null ? rows.length : Std.int(Math.min(rows.length, centre + 8)))) Sys.println(rows[row]);
    }

    static function len(v:Array<Float>):Float return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    static function f(v:Float):Float return Math.round(v * 1000) / 1000;
}
