package machinekit.motion;

import cadkit.modeling.Curve;
import cadkit.modeling.Sketch;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** HGR15R running section, in the rail-top datum. Analytic lines/arcs measured
 * from the pinned Matis-hosted HIWIN reference (see linear-rail-reference/README).
 * This geometry does not qualify the generic catalog's load or material ratings. */
class Hgr15RailGeometry {
    public static function build(length:Float, holes:Array<Float>):Part {
        if (!Math.isFinite(length) || length <= 0) throw "HGR15 rail needs a finite positive length";
        for (z in holes) if (!Math.isFinite(z) || z < 3.75 || z > length - 3.75)
            throw "HGR15 counterbore must fit within the cut length";
        var edges:Array<Curve> = [];
        var wire:Null<Curve> = null, face:Null<Sketch> = null, result:Null<Part> = null;
        var tools:Array<Part> = [];
        try {
            var current = new Vector(5.048994949669, 0);
            for (segment in section()) {
                var end = new Vector(segment[0], segment[1]);
                edges.push(segment.length == 2 ? Curve.line(current, end)
                    : Curve.arc(current, new Vector(segment[2], segment[3]), end));
                current = end;
            }
            wire = Curve.wire(edges);
            face = Sketch.face(wire);
            result = face.extrude(length);
            // HGR15R: 4.5 mm through bore, 7.5 mm counterbore, 5.3 mm deep.
            for (z in holes) {
                tools.push(Part.cylinderAlongY(2.25, -16, 1, 0, z));
                tools.push(Part.cylinderAlongY(3.75, -5.3, 1, 0, z));
            }
            if (tools.length > 0) {
                var drilled = result.subtractAll(tools);
                result.close(); result = drilled;
            }
            for (tool in tools) tool.close();
            face.close(); wire.close(); for (edge in edges) edge.close();
            return result;
        } catch (error:Dynamic) {
            for (tool in tools) tool.close();
            if (result != null) result.close();
            if (face != null) face.close();
            if (wire != null) wire.close();
            for (edge in edges) edge.close();
            throw error;
        }
    }

    // Each row gives an endpoint, followed by an optional circular-arc midpoint.
    static function section():Array<Array<Float>> return [
            [-5.048994949668, 0.000000000000],
            [-5.104881891275, -0.055886941609],
            [-5.218323295937, -0.284333896494, -5.180299301473, -0.160826072046],
            [-6.730359224779, -1.681364275152, -5.723905870254, -1.253900677434],
            [-7.500000000000, -2.451005050392],
            [-7.500000000000, -3.988994949012],
            [-6.741857403607, -4.747137545403],
            [-5.216972519766, -6.154862699427, -5.724401567524, -5.174762911767],
            [-5.103197590218, -6.385797358788, -5.179171401238, -6.279733328308],
            [-4.887157287523, -6.601837661482],
            [-4.769999999999, -6.884680373954, -4.800448186994, -6.731607001010],
            [-4.769999999999, -8.446740981527],
            [-4.887157287517, -8.729583693994, -4.800448186993, -8.599814354468],
            [-7.207106781203, -11.049533187790],
            [-7.500000000000, -11.756639968960, -7.423879532515, -11.373956536606],
            [-7.500000000000, -14.299999999999],
            [-6.799999999999, -15.000000000000],
            [6.800000000001, -15.000000000000],
            [7.500000000001, -14.300000000000],
            [7.500000000001, -11.756639968960],
            [7.207106781206, -11.049533187790, 7.423879532517, -11.373956536606],
            [4.887157287519, -8.729583693995],
            [4.770000000000, -8.446740981526, 4.800448186994, -8.599814354468],
            [4.770000000000, -6.884680373957],
            [4.887157287526, -6.601837661481, 4.800448186996, -6.731607001010],
            [5.103197590223, -6.385797358785],
            [5.216972519769, -6.154862699427, 5.179171401309, -6.279733328340],
            [6.741857403609, -4.747137545403, 5.724401567578, -5.174762911823],
            [7.500000000001, -3.988994949012],
            [7.500000000001, -2.451005050392],
            [6.730359224780, -1.681364275152],
            [5.218323295940, -0.284333896494, 5.723905870205, -1.253900677488],
            [5.104881891277, -0.055886941610, 5.180299301409, -0.160826072080],
            [5.048994949669, -0.000000000000],
    ];
}
