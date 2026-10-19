package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Plane;
import cadkit.modeling.Location;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Hollow, tapered bridge between parallel arm axes. A flat service land has
 * a separate cover; the uninterrupted lower shell carries the load. Cable ports
 * enter the bridge duct without claiming a through-bore in the drive modules.
 * Wall, boss and fastening dimensions are assumed construction.
 */
class CobotArmLink extends MachineComponent {
  public static inline var WALL:Float = 2.5;
  public static inline var CAP:Float = 5;
  public static inline var PORT_RADIUS:Float = 4.5;
  public final seat:AssemblyFrame;
  public final serviceFrame:AssemblyFrame;
  public final serviceLength:Float;
  public final serviceWidth:Float;
  public final inputRadius:Float;
  public final outputRadius:Float;
  public final bridgeStart:Vector;
  public final bridgeEnd:Vector;
  public final bridgeAxis:Vector;
  public final bridgeLength:Float;
  public final faceY:Float;
  public final input:CobotJoint;
  public final output:CobotJoint;
  static final collisionCache:Map<String,Array<Array<Float>>> = [];
  final inputPad:Float;
  final outputPad:Float;
  final end:Vector;
  final span:Float;

  public function new(a:Float, d:Float, previous:CobotJoint, next:CobotJoint) {
    var nextRadius = next.diameter / 2;
    if (!Math.isFinite(a) || !Math.isFinite(d) || a >= 0 || nextRadius <= 0)
      throw "Arm bridge needs finite parallel-axis dimensions and a positive next housing radius";
    super('COBOT-ARM-SHELL-$a-$d-${previous.size}-$nextRadius',
      "Tapered hollow arm link with removable service cover", "aluminium 6061", true);
    span = -a;
    input = previous; output = next;
    // The forearm must also clear the folded-back wrist on the short arm.
    // Retain the original proximal radius there, tapering toward the wrist;
    // the upper arm has room for the broader section.
    inputRadius = previous.diameter * (nextRadius < previous.diameter * 0.4 ? 0.23 : 0.30);
    outputRadius = Math.min(inputRadius, nextRadius * 0.60);
    inputPad = CobotWristMount.hubRadius(previous); outputPad = CobotWristMount.hubRadius(next);
    seat = AssemblyFrames.translation(a, 0, d - previous.length);
    end = new Vector(a, 0, seat.z - CAP - outputRadius);
    bridgeStart = new Vector(-previous.diameter / 2 - inputRadius, 0, CAP + inputRadius);
    bridgeEnd = new Vector(a + nextRadius + outputRadius, 0, end.z);
    var delta = bridgeEnd.subtract(bridgeStart);
    bridgeLength = delta.length(); bridgeAxis = delta.normalized();
    if (bridgeEnd.x >= bridgeStart.x || bridgeLength < 60 || Math.min(inputRadius, outputRadius) < 18)
      throw "Arm bridge has insufficient room for its shell, duct and service cover";
    var minimum = Math.min(inputRadius, outputRadius);
    faceY = minimum * 0.72;
    serviceLength = Math.max(52, bridgeLength * 0.45);
    serviceWidth = Math.max(31.5, minimum * 1.10);
    var origin = bridgeStart.add(delta.scale(0.5)).add(new Vector(0, faceY, 0));
    var plane = new Plane(origin, bridgeAxis, Vector.Y());
    serviceFrame = AssemblyFrames.fromRotationMatrix(origin.x, origin.y, origin.z,
      [plane.xDirection.x, plane.yDirection.x, plane.normal.x,
       plane.xDirection.y, plane.yDirection.y, plane.normal.y,
       plane.xDirection.z, plane.yDirection.z, plane.normal.z]);
    addConnector("start", Mount, Solids.axial(0, 0, 0));
    addConnector("end", Mount, AssemblyFrames.compose(seat, Solids.axial(0, 0, 0)));
    addConnector("service", Mount, AssemblyFrames.compose(serviceFrame, Solids.axial(0, 0, 0)));
    var index = 1;
    for (p in CobotWristMount.points(previous))
      addConnector('inputBolt${index++}', Mount, Solids.axial(p.x, p.y, CAP));
    index = 1;
    for (p in CobotWristMount.points(next)) addConnector('outputBolt${index++}', Mount,
      AssemblyFrames.compose(seat, AssemblyFrames.alongY(p.x, p.y, -CAP, 0, 0, -1)));
    for (i in 0...2) {
      var t = i == 0 ? 0.15 : 0.85;
      var p = bridgeStart.add(delta.scale(t)); p = p.add(new Vector(0, -radiusAt(t), 0));
      addConnector(i == 0 ? "cableIn" : "cableOut", Mount,
        AssemblyFrames.alongY(p.x, p.y, p.z, 0, -1, 0));
    }
    // Preserve the routed exterior rather than filling the bridge's empty elbow.
    var key=haxe.Json.stringify([a,d,previous.diameter,previous.length,previous.pocketDiameter,
      next.diameter,next.length,next.pocketDiameter]);
    var hulls=collisionCache.get(key);
    if(hulls==null) {hulls=collisionPieces();collisionCache.set(key,hulls);}
    addFacet(new machinekit.component.CollisionHullFacet(hulls));
  }

  public function radiusAt(t:Float):Float
    return inputRadius - 0.5 + (outputRadius - inputRadius) * t;

  /** Consume a solid in the service-cover frame. */
  public function atService(part:Part):Part {
    var m = AssemblyFrames.toRotationMatrix(serviceFrame);
    try {
      var out = part.placed(new Location(new Plane(
        new Vector(serviceFrame.x, serviceFrame.y, serviceFrame.z),
        new Vector(m[0], m[3], m[6]), new Vector(m[2], m[5], m[8]))));
      part.close(); return out;
    } catch (error:Dynamic) { part.close(); throw error; }
  }

  function flattenTop(part:Part, y:Float):Part {
    var clip = Part.box(span * 4, span * 4, span * 4);
    var placed = clip.placed(new Location(new Plane(new Vector(0, y, 0), Vector.X(), Vector.Y())));
    clip.close(); return Solids.cut(part, [placed]);
  }

  function bridge(inset:Float):Part {
    // Analytic conical faces keep both shell walls well conditioned where
    // the bridge meets the spherical end transitions and flat service land.
    // Overlap the end transitions axially instead of ending the cone exactly
    // at their equators, where coincident circular edges make fragile booleans.
    var overlap = 1.0, slope = (outputRadius - inputRadius) / bridgeLength;
    var taper = Part.revolve([{r:0.0,z:0.0}, {r:inputRadius - 0.5 - inset - slope * overlap,z:0.0},
      {r:outputRadius - 0.5 - inset + slope * overlap,z:bridgeLength + 2 * overlap},
      {r:0.0,z:bridgeLength + 2 * overlap}]);
    try {
      var result = taper.placed(new Location(new Plane(bridgeStart.subtract(bridgeAxis.scale(overlap)), Vector.Y(), bridgeAxis)));
      taper.close();
      return flattenTop(result, faceY - inset);
    } catch (error:Dynamic) { taper.close(); throw error; }
  }

  function endPieces(inset:Float):Array<Part> {
    var r0 = inputRadius - inset, r1 = outputRadius - inset;
    var parts = [Part.cylinderSpan(r0, inset == 0 ? 0 : CAP,
      CAP + inputRadius), Part.cylinderSpan(r1, end.z, inset == 0 ? seat.z : seat.z - CAP, end.x)];
    parts.push(Part.cylinderAlong(r0, new Vector(0, 0, CAP + inputRadius),
      Vector.X().scale(-1), -bridgeStart.x));
    parts.push(Part.cylinderAlong(r1, end, Vector.X(), bridgeEnd.x - end.x));
    // Each turn needs both an outer rounding and its matching inner cavity.
    // Intersecting two hollow cylinders alone leaves the outer quadrant open
    // even though their remaining walls form one connected solid.
    for (pair in [{p:bridgeStart, r:r0}, {p:bridgeEnd, r:r1},
        {p:new Vector(0, 0, CAP + inputRadius), r:r0}, {p:end, r:r1}]) {
      var ball = Part.sphere(pair.r); parts.push(ball.translated(pair.p)); ball.close();
    }
    return parts;
  }

  /** Preserve the end-transition construction order. Balanced unions of the
   * tangent end pieces can leave overlapping solids in OCCT even when the
   * compound reports valid topology.
   */
  static function joined(parts:Array<Part>):Part {
    var result:Part = null;
    try {
      result = parts[0];
      for (i in 1...parts.length) {
        var next = result.combine(parts[i]); result.close(); parts[i].close(); result = next;
      }
      return result;
    } catch (error:Dynamic) {
      if (result != null && !result.shape.isClosed()) result.close();
      for (part in parts) if (!part.shape.isClosed()) part.close();
      throw error;
    }
  }

  function routed(inset:Float):Part {
    var ends = endPieces(inset);
    ends.push(bridge(inset));
    return joined(ends);
  }

  public static function roundedPlate(length:Float, width:Float, height:Float, radius:Float, z:Float):Part {
    var parts = [Part.box(length - 2 * radius, width, height), Part.box(length, width - 2 * radius, height)];
    for (x in [-1, 1]) for (y in [-1, 1])
      parts.push(Part.cylinderSpan(radius, 0, height, x * (length / 2 - radius), y * (width / 2 - radius)));
    var plate = joined(parts);
    try { var placed = plate.translated(new Vector(0, 0, z)); plate.close(); return placed; }
    catch (error:Dynamic) { plate.close(); throw error; }
  }

  /** Convex construction pieces cover every added feature before holes are cut. */
  function outerPieces():Array<Part> {
    var pieces=endPieces(0);pieces.push(bridge(0));
    pieces.push(Part.cylinderSpan(inputPad, 0, CAP));
    pieces.push(Part.cylinderSpan(outputPad, seat.z - CAP, seat.z, seat.x));
    var pilot = CobotWristMount.pilot(input);
    pieces.push(pilot.translated(new Vector(0, 0, -CobotWristMount.PILOT_DEPTH))); pilot.close();
    pilot = CobotWristMount.pilot(output);
    pieces.push(pilot.translated(new Vector(seat.x, 0, seat.z))); pilot.close();
    return pieces;
  }

  function collisionPieces():Array<Array<Float>> {
    var pieces=outerPieces(),hulls:Array<Array<Float>> = [];
    // These bosses are added after hollowing in geometry(), so include them too.
    for(p in CobotLinkCover.boltPoints(serviceLength,serviceWidth))
      pieces.push(atService(Part.cylinderSpan(3.5,-9,0,p.x,p.y)));
    try {
      for(index in 0...pieces.length) {
        var mesh=pieces[index].shape.tessellate(0.03),vertices=mesh.vertices;
        // A world-axis DOP around a long oblique taper can fill tens of mm
        // outside the shell. Construct that piece in its longitudinal frame.
        if(index==8) {
          vertices=haxe.io.Bytes.alloc(mesh.vertexCount*24);
          var inverse=AssemblyFrames.inverse(serviceFrame);
          for(i in 0...mesh.vertexCount) {
            var v=AssemblyFrames.transformPoint(inverse,mesh.vertices.getDouble(i*24),mesh.vertices.getDouble(i*24+8),mesh.vertices.getDouble(i*24+16));
            vertices.setDouble(i*24,v.x);vertices.setDouble(i*24+8,v.y);vertices.setDouble(i*24+16,v.z);
          }
        }
        var hull=cadkit.ConvexHullVertices.enclosingFromMesh(vertices,mesh.vertexCount,0.001,0.05).vertices;
        if(index==8)for(i in 0...Std.int(hull.length/3)) {
          var v=AssemblyFrames.transformPoint(serviceFrame,hull[i*3],hull[i*3+1],hull[i*3+2]);
          hull[i*3]=v.x;hull[i*3+1]=v.y;hull[i*3+2]=v.z;
        }
        hulls.push(hull);
      }
    } catch(error:Dynamic) {for(piece in pieces)piece.close();throw error;}
    for(piece in pieces)piece.close();return hulls;
  }

  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var pieces=outerPieces();
    var body = joined(pieces);
    if (detail == Envelope) return body;
    // Fuse overlapping duct tools first: a compound of tangent taper/cylinder
    // tools can leave ambiguous internal faces in OCCT's multi-tool subtraction.
    body = Solids.cut(body, [routed(WALL)]);
    var bosses:Array<Part> = [body];
    for (p in CobotLinkCover.boltPoints(serviceLength, serviceWidth))
      bosses.push(atService(Part.cylinderSpan(3.5, -9, 0, p.x, p.y)));
    body = joined(bosses);
    var cuts = [atService(roundedPlate(serviceLength - 16, serviceWidth - 16, WALL + 0.2, 3, -WALL - 0.1))];
    for (p in CobotWristMount.points(input)) {
      cuts.push(Part.cylinderSpan(CobotWristMount.HOLE_RADIUS, -0.1, CAP + 0.1, p.x, p.y));
      cuts.push(Part.cylinderSpan(4, CAP, CAP + 2 * inputRadius + 1, p.x, p.y));
    }
    for (p in CobotWristMount.points(output)) {
      cuts.push(Part.cylinderSpan(CobotWristMount.HOLE_RADIUS, seat.z - CAP - 0.1, seat.z + 0.1, seat.x + p.x, p.y));
      cuts.push(Part.cylinderSpan(4, seat.z - CAP - 2 * outputRadius - 1, seat.z - CAP, seat.x + p.x, p.y));
    }
    for (p in CobotLinkCover.boltPoints(serviceLength, serviceWidth))
      cuts.push(atService(Part.cylinderSpan(1.6, -7.5, 0.1, p.x, p.y)));
    for (t in [0.15, 0.85]) {
      var p = bridgeStart.add(bridgeEnd.subtract(bridgeStart).scale(t));
      cuts.push(Part.cylinderAlong(PORT_RADIUS, p.add(new Vector(0, -inputRadius - 1, 0)),
        Vector.Y(), inputRadius + 1));
    }
    return Solids.cut(body, cuts);
  }
}
