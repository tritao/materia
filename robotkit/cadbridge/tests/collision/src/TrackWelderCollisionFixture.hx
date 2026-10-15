import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import cadbridge.CellCollisionSnapshot;
import cadkit.modeling.AssemblyState;
import collisionkit.native.NativeCollisionWorld;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import robotkit.manipulation.Manipulator;
import robotkit.model.Frame;

private typedef TrackInterfaceFixture = {
  var first:String;
  var second:String;
  var distance:Float;
  var margin:Float;
  var clear:Bool;
}

/** The shipped track-welder geometry and typed weld interfaces, rather than surrogate boxes. */
class TrackWelderCollisionFixture {
  static final interfaces:Array<TrackInterfaceFixture> = [
    {first: "rackSupport", second: "motorPlate", distance: 0.004, margin: processkit.WeldPathPlanner.AIR_MARGIN, clear: true}
  ];
  public static function run(check:(Bool, String)->Void):Void {
    var artifact = SceneArtifact.decode(TrackWelderPreview.cell());
    var definition = artifact.assemblyDefinition, mission = artifact.missions[0], tools = artifact.robotTools;
    if (definition == null || mission == null || tools == null || tools.length != 1)
      throw "Track-welder corpus needs its CAD definition, weld mission and physical tool";
    var weld = mission.steps[0].weld;
    if (weld == null) throw "Track-welder corpus lost its typed CAD weld faces";
    var physical = AssemblyPhysicalPartView.fromSceneArtifact(artifact);
    var converted = AssemblySimulationBridge.toRobotModel(definition, physical, artifact.assemblyState);
    var tool = tools[0], carrier = converted.partLinks.get(tool.contact.occurrence);
    if (carrier == null) throw "Track-welder corpus lost its tool carrier";
    var flat = AssemblyDefinitionFlattener.flatten(definition);
    var componentIds = [for (occurrence in flat.occurrences) if (occurrence.id == tool.contact.occurrence) occurrence.definition];
    if (componentIds.length != 1) throw "Track-welder corpus needs one tool occurrence";
    var connectors = [for (component in flat.definitions) if (component.id == componentIds[0])
      for (connector in component.connectors) if (connector.name == tool.contact.connector) connector.frame];
    if (connectors.length != 1) throw "Track-welder corpus needs one contact connector";
    var source = connectors[0], scale = artifact.metresPerUnit;
    var tcp = AssemblyFrames.compose(carrier.offset, {x: source.x * scale, y: source.y * scale, z: source.z * scale,
      qx: source.qx, qy: source.qy, qz: source.qz, qw: source.qw});
    var frame = converted.model.addFrame(new Frame("corpus-contact", converted.model.links[carrier.link]));
    frame.position = [tcp.x, tcp.y, tcp.z]; frame.rotation = [tcp.qx, tcp.qy, tcp.qz, tcp.qw];
    var arm = new Manipulator(converted.model, converted.model.links[0].id, frame.id);
    var reference = [for (_ in 0...arm.group.count()) 0.0];
    var state = new AssemblyState(definition, artifact.assemblyState);
    var captured = CellCollisionSnapshot.fromAssembly("track-welder", definition, physical, state, converted, [weld.metal], tool.members);
    var scene = captured.describe(arm, reference, 0.005);
    var members:Array<String> = [];
    for (segment in weld.path) for (face in segment.faces) if (members.indexOf(face.member) < 0) members.push(face.member);
    processkit.WeldCollisionPolicy.apply(scene, scene.memberObjects(members), processkit.WeldPathPlanner.AIR_MARGIN, robotkit.collision.CollisionClearance.CONTACT_MARGIN);
    var build = scene.build(new NativeCollisionWorld());
    try {
      for (fixture in interfaces) {
        var first = scene.memberObjects([fixture.first]), second = scene.memberObjects([fixture.second]);
        if (first.length == 0 || second.length == 0) throw "Track-welder corpus lost a physical interface member";
        var pairs:Array<collisionkit.CollisionPair> = [];
        for (a in first) for (b in second) pairs.push(new collisionkit.CollisionPair(build.objects[a], build.objects[b],
          build.bodies[scene.description.objects[a].body], build.bodies[scene.description.objects[b].body]));
        var nearest = Math.POSITIVE_INFINITY;
        for (distance in build.world.pairDistances(pairs)) nearest = Math.min(nearest, distance.distance);
        check(Math.abs(nearest - fixture.distance) < 1e-7, "the real track welder retains its 4 mm mechanical interface geometry");
        for (a in first) for (b in second) check(scene.description.marginBetweenObjects(a, b) == fixture.margin,
          "the real installed interface retains the welding policy's 3 mm margin");
        Sys.println('Track-welder corpus interface: ${fixture.first}/${fixture.second}, $nearest m');
      }
      var clearance = scene.clearance(() -> new NativeCollisionWorld());
      var hit = clearance.violation(reference);
      clearance.world.dispose();
      check((hit == null) == interfaces[0].clear, 'the shipped track-welder ready pose clears under the shared policy: $hit');
    } catch (error:Dynamic) { build.world.dispose(); throw error; }
    build.world.dispose();
  }
}
