package app;

import app.AssemblyRobot.AssemblyPart;
import haxe.io.Bytes;
import materia.project.SceneArtifact.SceneArtifactWeld;
import nativekit.scene.GeometryData;
import robotkit.runtime.SimulatedWelder;
import robotkit.runtime.Simulation;
import robotkit.tool.WeldBead;

/** The weld metal laid along one seam of the mission, and where it is shown. */
class SeamBead {
  public final step:Int;
  public final weld:SceneArtifactWeld;
  public final bead:WeldBead;
  /** First part index of the bead's chunks among its host's parts, and how many chunks the bead is cut into. */
  public final firstPart:Int;
  public final chunks:Int;
  /** Stations changed since the bead was last shown. */
  public var dirtyFrom:Int;
  public var dirtyTo:Int = -1;

  public function new(step:Int, weld:SceneArtifactWeld, bead:WeldBead, firstPart:Int, chunks:Int) {
    this.step = step;
    this.weld = weld;
    this.bead = bead;
    this.firstPart = firstPart;
    this.chunks = chunks;
    dirtyFrom = bead.count;
  }
}

/** The part that carries the weld metal of some seams: the beads are shown as parts of its scene object. */
private class BeadHost {
  public final part:AssemblyPart;
  public final beads:Array<SeamBead> = [];
  public var parts:Int = 0;
  /** The object shows beads now. */
  public var shown:Bool = false;

  public function new(part:AssemblyPart) this.part = part;
}

/**
 * The weld metal of a mission's seams, laid as the welder deposits it. Each tick of a weld step it tells the bead of
 * that seam (`WeldBead`) whether the arc is established, how fast the wire feeds and where the wire tip is; the bead
 * is what the torch actually did, so its leg is a result, compared with the leg the seam asks for. The metal shows as
 * runtime geometry on the part that carries the weld metal (the `metal` occurrence of the step), cut into chunks of
 * `CHUNK` stations so only the stretch that grew is meshed again, and no more often than `REFRESH` seconds of
 * simulation time. Like the CNC router's stock, it is view state: not saved, and gone with a reset.
 *
 * Beads are in the world frame the seams were given in (the assembly as designed), and the part that carries them
 * must be fixed to the workpiece for the bead to follow it; the mesh is expressed in that part's frame as it is now.
 */
class WeldBeads implements SessionMember {
  /** Stations in one mesh chunk. */
  public static inline var CHUNK:Int = 20;
  /** Simulation seconds between meshings of what grew. */
  public static inline var REFRESH:Float = 0.1;
  /** Grey of weld metal, as RGBA. */
  static inline var METAL = 0x9A968FFF;

  public final beads:Array<SeamBead> = [];

  final mission:MissionPlayer;
  final welder:SimulatedWelder;
  final simulation:Simulation;
  final scene:EditorScene;
  final timestep:Float;
  final hosts:Array<BeadHost> = [];
  var elapsed:Float = 0.0;
  var shownAt:Float = Math.NEGATIVE_INFINITY;

  /**
   * `parts` are the simulated parts, from which each weld's `metal` occurrence is taken; `wireDiameterMm` is the
   * torch's wire.
   */
  public function new(mission:MissionPlayer, welder:SimulatedWelder, simulation:Simulation, parts:Array<AssemblyPart>,
      scene:EditorScene, timestep:Float, wireDiameterMm:Float) {
    this.mission = mission;
    this.welder = welder;
    this.simulation = simulation;
    this.scene = scene;
    this.timestep = timestep;
    for (index in 0...mission.mission.steps.length) {
      var step = mission.mission.steps[index];
      var weld = step.weld;
      if (step.kind != "weld" || weld == null) continue;
      var id = "project:" + weld.metal;
      var host:Null<BeadHost> = null;
      for (candidate in hosts) if (candidate.part.id == id) host = candidate;
      if (host == null) {
        var found = [for (part in parts) if (part.id == id) part];
        if (found.length != 1) throw 'The weld metal part "${weld.metal}" is not part of the simulated assembly';
        host = new BeadHost(found[0]);
        hosts.push(host);
      }
      var bead = new WeldBead(weld.start.position, weld.stop.position, weld.normals[0], weld.normals[1], wireDiameterMm);
      var chunks = Std.int(Math.ceil(bead.count / CHUNK));
      var seam = new SeamBead(index, weld, bead, host.parts, chunks);
      host.parts += chunks;
      host.beads.push(seam);
      beads.push(seam);
    }
  }

  /** The bead of weld step `step`. */
  public function beadOf(step:Int):WeldBead {
    for (seam in beads) if (seam.step == step) return seam.bead;
    throw 'Mission step $step does not weld';
  }

  /** Deposits what the welder melted in the tick that just ran, into the seam being welded. */
  public function feed():Void {
    elapsed += timestep;
    var index = mission.weldingStep();
    if (index < 0) return;
    var reading = welder.reading();
    for (seam in beads) if (seam.step == index) {
      seam.bead.step(timestep, reading.arc, welder.wireSpeed(), welder.tip());
      var changed = seam.bead.takeChanges();
      if (changed != null) {
        seam.dirtyFrom = Std.int(Math.min(seam.dirtyFrom, changed.first));
        seam.dirtyTo = Std.int(Math.max(seam.dirtyTo, changed.last));
      }
    }
  }

  /** A reset forgets the metal: the workpiece is bare again. */
  public function reset():Void {
    for (seam in beads) {
      seam.bead.reset();
      seam.bead.takeChanges();
      seam.dirtyFrom = seam.bead.count;
      seam.dirtyTo = -1;
    }
    for (host in hosts) if (host.shown) {
      scene.clearRuntimeGeometry(host.part.id);
      host.shown = false;
    }
    elapsed = 0.0;
    shownAt = Math.NEGATIVE_INFINITY;
  }

  /** Shows the metal that grew, at most every `REFRESH` seconds. */
  public function present():Void {
    if (elapsed - shownAt < REFRESH) return;
    shownAt = elapsed;
    for (host in hosts) {
      var changed:Map<Int, GeometryData> = new Map();
      var pose = AssemblyRobot.partPose(simulation, host.part);
      for (seam in host.beads) if (seam.dirtyTo >= seam.dirtyFrom) {
        // A station's mesh depends on its neighbours' legs, so the chunks beside the changed stretch change too.
        var first = Std.int(Math.max(0, seam.dirtyFrom - 1) / CHUNK);
        var last = Std.int(Math.min(seam.bead.count - 1, seam.dirtyTo + 1) / CHUNK);
        for (chunk in first...last + 1)
          changed.set(seam.firstPart + chunk, chunkGeometry(seam.bead, chunk, pose, host.part.center));
        seam.dirtyFrom = seam.bead.count;
        seam.dirtyTo = -1;
      }
      var count = 0;
      for (_ in changed.keys()) count++;
      if (count == 0) continue;
      scene.setRuntimeGeometryParts(host.part.id, host.parts, changed);
      host.shown = true;
    }
  }

  /** Releases what the beads show. */
  public function dispose():Void {
    for (host in hosts) if (host.shown) scene.clearRuntimeGeometry(host.part.id);
  }

  /**
   * The mesh of stations `chunk * CHUNK` on, in the frame of the part that carries the metal (`pose` is that part's world
   * pose, `center` its preview centre, in metres). Each station is the hypotenuse of its triangular section between the two
   * boundary sections, whose legs are the mean of the stations beside them, with an end cap where the metal stops.
   */
  static function chunkGeometry(bead:WeldBead, chunk:Int, pose:{position:Array<Float>, rotation:Array<Float>},
      center:Array<Float>):GeometryData {
    var first = chunk * CHUNK, end = Std.int(Math.min(bead.count, first + CHUNK));
    var inverse = [-pose.rotation[0], -pose.rotation[1], -pose.rotation[2], pose.rotation[3]];
    function local(point:Array<Float>):Array<Float> {
      var turned = AssemblyRobot.rotate(inverse, [for (axis in 0...3) point[axis] - pose.position[axis]]);
      return [for (axis in 0...3) turned[axis] - center[axis]];
    }
    function turn(direction:Array<Float>):Array<Float> return AssemblyRobot.rotate(inverse, direction);
    function leg(station:Int):Float return station < 0 || station >= bead.count ? 0.0 : bead.leg(station);
    // The leg of the section at the boundary before station `j`.
    function ring(j:Int):Float {
      var a = leg(j - 1), b = leg(j);
      return a > 0 && b > 0 ? (a + b) / 2 : a > 0 ? a : b;
    }
    var positions:Array<Float> = [], normals:Array<Float> = [], indices:Array<Int> = [];
    function triangle(p:Array<Array<Float>>, normal:Array<Float>):Void {
      var cross = [(p[1][1] - p[0][1]) * (p[2][2] - p[0][2]) - (p[1][2] - p[0][2]) * (p[2][1] - p[0][1]),
        (p[1][2] - p[0][2]) * (p[2][0] - p[0][0]) - (p[1][0] - p[0][0]) * (p[2][2] - p[0][2]),
        (p[1][0] - p[0][0]) * (p[2][1] - p[0][1]) - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0])];
      var flip = cross[0] * normal[0] + cross[1] * normal[1] + cross[2] * normal[2] < 0;
      var base = Std.int(positions.length / 3);
      for (point in p) {
        for (axis in 0...3) positions.push(point[axis]);
        for (axis in 0...3) normals.push(normal[axis]);
      }
      indices.push(base);
      indices.push(flip ? base + 2 : base + 1);
      indices.push(flip ? base + 1 : base + 2);
    }
    for (station in first...end) {
      if (!(leg(station) > 0)) continue;
      var near = ring(station), far = ring(station + 1);
      var s0 = station * WeldBead.BIN, s1 = Math.min(bead.length, (station + 1) * WeldBead.BIN);
      var p0 = bead.pointAt(s0), p1 = bead.pointAt(s1);
      function corner(base:Array<Float>, direction:Array<Float>, size:Float):Array<Float>
        return local([for (axis in 0...3) base[axis] + direction[axis] * size]);
      var a0 = corner(p0, bead.legA, near), b0 = corner(p0, bead.legB, near);
      var a1 = corner(p1, bead.legA, far), b1 = corner(p1, bead.legB, far);
      // The face of the fillet looks out of the corner, along the sum of the faces' normals.
      var open = turn([for (axis in 0...3) bead.normalA[axis] + bead.normalB[axis]]);
      var length = Math.sqrt(open[0] * open[0] + open[1] * open[1] + open[2] * open[2]);
      var normal = [for (axis in 0...3) open[axis] / length];
      triangle([a0, b0, b1], normal);
      triangle([a0, b1, a1], normal);
      var along = turn(bead.tangent);
      if (!(leg(station - 1) > 0))
        triangle([local(p0), a0, b0], [-along[0], -along[1], -along[2]]);
      if (!(leg(station + 1) > 0))
        triangle([local(p1), a1, b1], along);
    }
    var vertices = Std.int(positions.length / 3);
    var data = new GeometryData();
    var positionBytes = Bytes.alloc(vertices * 12), normalBytes = Bytes.alloc(vertices * 12), colours = Bytes.alloc(vertices * 4);
    var indexBytes = Bytes.alloc(indices.length * 4);
    for (i in 0...vertices * 3) {
      positionBytes.setFloat(i * 4, positions[i]);
      normalBytes.setFloat(i * 4, normals[i]);
    }
    for (vertex in 0...vertices) {
      colours.set(vertex * 4, (METAL >>> 24) & 0xFF);
      colours.set(vertex * 4 + 1, (METAL >>> 16) & 0xFF);
      colours.set(vertex * 4 + 2, (METAL >>> 8) & 0xFF);
      colours.set(vertex * 4 + 3, METAL & 0xFF);
    }
    for (i in 0...indices.length) indexBytes.setInt32(i * 4, indices[i]);
    data.addStream(1, 2, positionBytes, vertices, 12);
    data.addStream(2, 2, normalBytes, vertices, 12);
    data.addStream(6, 4, colours, vertices, 4);
    data.setIndexBuffer(indexBytes, indices.length);
    if (vertices > 0) {
      var low = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
      var high = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
      for (vertex in 0...vertices) for (axis in 0...3) {
        var value = positions[vertex * 3 + axis];
        if (value < low[axis]) low[axis] = value;
        if (value > high[axis]) high[axis] = value;
      }
      data.setBounds(low[0], low[1], low[2], high[0], high[1], high[2]);
    }
    return data;
  }
}
