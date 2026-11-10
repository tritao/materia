import cadkit.Mesh;
import cadkit.MeshFeatureEdges;
import cadkit.modeling.Part;
import cadkit.modeling.Selection;
import cadkit.modeling.Vector;

/** Step heights of feature edges: corners draw at any size, steps fade by their height (app/plans/EDGE_SIGNIFICANCE.md). */
class EdgeStepSmoke {
  static var passed = 0;

  static function check(ok:Bool, message:String):Void {
    if (!ok) throw "edge steps: " + message;
    passed++;
  }

  static function steps(part:Part):Array<Float> {
    var mesh:Mesh = part.shape.tessellate(0.1, 0.5);
    var faces = MeshFeatureEdges.triangleFaces(mesh.indexCount, [for (r in mesh.faceRanges) r.faceIndex],
      [for (r in mesh.faceRanges) r.firstIndex], [for (r in mesh.faceRanges) r.indexCount]);
    return MeshFeatureEdges.steps(mesh.vertices, mesh.indices, mesh.vertexCount, mesh.indexCount, faces, mesh.edgeSegments,
      mesh.edgeIds);
  }

  static function near(value:Float, expected:Float):Bool return Math.abs(value - expected) < 1e-6;

  public static function main():Void {
    var box = Part.box(10, 10, 10);
    var plain = steps(box);
    check(plain.length > 0 && plain.filter(h -> h != 0.0).length == 0, "every box edge is a corner");

    var edges = Selection.from(box.shape, Edge);
    var chamfered = box.chamfer(edges, 1.0);
    edges.close();
    check(steps(chamfered).filter(h -> h != 0.0).length == 0, "chamfer edges are corners");

    // A 0.05 mm band raised on a plate: its eight edges are steps of 0.05. The plate's top outline is a ledge out to
    // the band (5 and 7 mm), its bottom outline a corner.
    var plate = Part.box(20, 20, 2), band = Part.box(10, 6, 0.05).translated(new Vector(0, 0, 2));
    var raised = Part.fuseAll([plate, band]).refined();
    var bandSteps = steps(raised);
    check(bandSteps.filter(h -> near(h, 0.05)).length >= 8, 'band edges are 0.05 steps, got $bandSteps');
    check(bandSteps.filter(h -> h != 0.0 && !near(h, 0.05) && !near(h, 5.0) && !near(h, 7.0)).length == 0,
      'only band steps and ledges, got $bandSteps');
    check(bandSteps.filter(h -> h == 0.0).length >= 4, "the plate's bottom outline stays corners");

    // A way-cover skin: a tube with a 0.05 mm proud collar at one end. The collar's end is one ring-shaped face, so
    // its step has to be found from the ring's neighbours, not from the ring's extent.
    var tube = Part.box(20, 20, 30).subtract(Part.box(19.6, 19.6, 32).translated(new Vector(0, 0, -1)));
    var collar = Part.box(20.1, 20.1, 6).subtract(Part.box(19.9, 19.9, 8).translated(new Vector(0, 0, -1)));
    var skin = Part.fuseAll([tube, collar]).refined();
    var skinSteps = steps(skin);
    check(skinSteps.filter(h -> near(h, 0.05)).length >= 8, 'collar edges are 0.05 steps, got $skinSteps');
    // From the collar's step, the skin's outer face runs 24 mm to the tube's end: a 24 mm ledge. Nothing reaches across
    // the tube to the far wall.
    check(skinSteps.filter(h -> h != 0.0 && !near(h, 0.05) && !near(h, 24.0)).length == 0, 'a tube end is no step across the tube, got $skinSteps');

    // Two nested open channels (a U-section way cover): the outer channel's end steps down 0.25 to the inner one; no
    // edge sees a step across the channel's width from one leg's outer face to the other leg's inner face.
    function channel(half:Float, length:Float, z:Float):Part
      return Part.box(2 * half + 0.4, 10.2, length).subtract(Part.box(2 * half, 10, length + 2).translated(new Vector(0, -0.2, -1)))
        .translated(new Vector(0, 0, z));
    var nested = Part.fuseAll([channel(10.25, 20, 0), channel(10, 40, 0)]).refined();
    var nestedSteps = steps(nested);
    // Real steps only: 0.25 between the channels, each leg's 9.9 inner wall (foot to ceiling), the inner channel's 20 mm
    // beyond the outer one. Never the 20.2-20.7 mm from one leg's outer face across to the other leg's inner face.
    check(nestedSteps.filter(h -> h != 0.0 && !near(h, 0.25) && !near(h, 9.9) && !near(h, 20.0)).length == 0,
      'no step across a channel, got $nestedSteps');
    check(nestedSteps.filter(h -> near(h, 0.25)).length > 0, 'the outer channel steps 0.25 down to the inner, got $nestedSteps');

    // A 10 mm step: the step's two edges carry 10.
    var base = Part.box(20, 20, 10), upper = Part.box(20, 10, 10).translated(new Vector(0, 5, 10));
    var block = Part.fuseAll([base, upper]).refined();
    var blockSteps = steps(block).filter(h -> h != 0.0);
    check(blockSteps.length > 0 && blockSteps.filter(h -> !near(h, 10.0)).length == 0, 'block step is 10, got $blockSteps');
    Sys.println('edge steps: $passed checks passed');
  }
}
