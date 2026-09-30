import humankit.HumanBodyProxy;
import humankit.HumanCharacter;
import humankit.HumanDescription;
import humankit.HumanLimb;
import humankit.HumanTargetBox;
import humankit.HumanoidRig;
import humankit.facility.FacilityJobs;
import humankit.sim.HumanWorker;
import animkit.AnimationAsset;
import materia.automation.facility.Facility;
import materia.automation.facility.Lane;
import materia.automation.facility.Rack;
import materia.automation.facility.RackSlot;
import materia.automation.facility.RackSlotPose;
import materia.automation.facility.Station;
import materia.automation.facility.Surface;
import materia.automation.facility.Zone;
import nativekit.scene.Scene;
import nativekit.sim.MotionType;
import nativekit.sim.MujocoSimWorld;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import nativekit.sim.SimShape;
import robotkit.mobile.Footprint;
import robotkit.mobile.Pose2;
import robotkit.navigation.Path;

/** Where the station stands and the lane between rack and station, and how high the surfaces are. */
typedef FacilityLayout = {
    /** The table's floor position; the rack stands at (0.9, -0.2). The station is where the worker stands at it. */
    var station:Array<Float>;
    /** Floor points the lane passes through between rack and station, in order. */
    var via:Array<Array<Float>>;
    /** Height of the rack's and station's tops, in metres. */
    var surface:Float;
}

/** A facility fetch-and-deliver job built from a Facility, run on a bare session. */
class FacilityScenario {
    public final session:SimSession;
    public final worker:HumanWorker;
    public final part:SimObject;
    public final placePoint:Array<Float>;
    public final surfaces:Array<HumanTargetBox>;
    final world:MujocoSimWorld;

    function new(session:SimSession, world:MujocoSimWorld, worker:HumanWorker, part:SimObject, placePoint:Array<Float>,
            surfaces:Array<HumanTargetBox>) {
        this.session = session;
        this.world = world;
        this.worker = worker;
        this.part = part;
        this.placePoint = placePoint;
        this.surfaces = surfaces;
    }

    public function limb():HumanLimb return ArmR;

    /**
     * How the job learns what the worker stands at and picks up: "model", the facility describes its own
     * surfaces and slot items; "explicit", the caller hands them to the job; "none", nothing is described.
     */
    public static function build(layout:FacilityLayout, describe:String = "model"):FacilityScenario {
        var asset = AnimationAsset.load("../../../animkit/assets/quaternius/worker.glb");
        var scene = Scene.create();
        var world = MujocoSimWorld.create(scene, {timestep: 1.0 / 60.0, physicsSubsteps: 4, gravity: [0.0, 0.0, -9.81]});
        var session = new SimSession(scene, world.nativeHandle());
        var human = new HumanCharacter(scene, asset, HumanoidRig.detect(asset), null, "facility worker");
        human.advance(0.0);
        var proxy = HumanBodyProxy.standard(human.pose, HumanDescription.measure(human.pose, human.height()));
        var worker = new HumanWorker(session, human, proxy, new SimPose(0, 0, 0));
        var surface = layout.surface, station = layout.station;
        var slotPoint = [0.9, -0.2, surface + 0.09], placePoint = [station[0], station[1], surface + 0.09];
        // A facility station is where a worker stands at the table, not the table itself: short of it along
        // the lane's last leg, where the worker steps back to once the part is down.
        var last = layout.via.length == 0 ? [0.9, -0.2] : layout.via[layout.via.length - 1];
        var leg = [station[0] - last[0], station[1] - last[1]], legLength = Math.sqrt(leg[0] * leg[0] + leg[1] * leg[1]);
        var stand = [station[0] - 0.65 * leg[0] / legLength, station[1] - 0.65 * leg[1] / legLength];
        var modelled = describe == "model";
        var facility = new Facility("sweep", "Sweep");
        facility.addZone(new Zone("floor", "Floor", "map", Footprint.rectangle(12, 12)));
        var rack = new Rack("rack", "Rack", "floor", "map", new Pose2(0.9, -0.2),
            [new RackSlot("part", new RackSlotPose(0, 0, slotPoint[2]), modelled ? [0.04, 0.04, 0.04] : null)],
            modelled ? new Surface(0.2, 0.2, surface) : null);
        // The table lies 0.65 m ahead of where the worker stands, along the lane's last leg.
        var facing = Math.atan2(leg[1], leg[0]);
        var table = new Station("table", "Table", "floor", "map", new Pose2(stand[0], stand[1], facing),
            modelled ? new Surface(0.2, 0.2, surface, 0.65) : null);
        facility.addRack(rack);
        facility.addStation(table);
        var lane = [rack.pose];
        for (point in layout.via) lane.push(new Pose2(point[0], point[1]));
        lane.push(table.pose);
        facility.addLane(new Lane("carry", "rack", "table", new Path(lane, "map"), 1, 1));
        var part = session.createObject(MotionType.Dynamic, SimShape.box(0.04, 0.04, 0.04), new SimPose(0.9, -0.2, surface + 0.04), 0.1);
        session.createObject(MotionType.Static, SimShape.box(0.2, 0.2, 0.05), new SimPose(0.9, -0.2, surface - 0.05));
        session.createObject(MotionType.Static, SimShape.box(0.2, 0.2, 0.05), new SimPose(station[0], station[1], surface - 0.05));
        session.createObject(MotionType.Static, SimShape.box(6, 6, 0.1), new SimPose(0, 0, -0.05));
        var surfaces:Array<HumanTargetBox> = [
            {center: [0.9, -0.2, surface - 0.05], halfExtents: [0.2, 0.2, 0.05], yaw: 0.0},
            {center: [station[0], station[1], surface - 0.05], halfExtents: [0.2, 0.2, 0.05], yaw: 0.0}
        ];
        var partBox:HumanTargetBox = {center: slotPoint, halfExtents: [0.04, 0.04, 0.04], yaw: 0.0};
        var fetch = FacilityJobs.fetch(facility, "rack", "part");
        var job = describe == "explicit" ? fetch.deliver("table", placePoint, surfaces[0], surfaces[1], partBox) :
            fetch.deliver("table", placePoint);
        var actions = job.orderedActions();
        worker.bindPick(cast actions[1], part, slotPoint);
        worker.bindPlace(cast actions[4], part, slotPoint);
        worker.run(job);
        return new FacilityScenario(session, world, worker, part, placePoint, surfaces);
    }

    /** Feeds the worker and runs one tick; returns where the part is afterwards. */
    public function tick():Array<Float> {
        worker.advance();
        session.step();
        var captured = session.capture();
        var pose = captured.objectPose(part);
        captured.dispose();
        return [pose.x, pose.y, pose.z];
    }
}
