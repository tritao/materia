import cadkit.modeling.Part;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.process.FillAnalysis;
import materia.tooling.process.OrthogonalSections;
import materia.tooling.process.OrthogonalSections.SectionAxis;
import machinekit.tooling.ToolingBlock;
import machinekit.component.Solids;

/** Independent BREP checks for the neutral exact event-plane fill calculation. */
class ProcessNativeTests {
	static var assertions = 0;

	static function check(ok:Bool, message:String):Void {
		assertions++;
		if (!ok)
			throw message;
	}

	static function box(x:Float, z:Float, width:Float, height:Float):MoldBox
		return {
			x: x,
			y: 0,
			z: z,
			width: width,
			depth: 2,
			height: height
		};

	static function verify(vented:Bool):Void {
		var cells = [box(0, 0, 2, 10), box(2, 0, 2, 2), box(4, 0, 4, 6)];
		if (vented)
			cells.push(box(6, 6, 2, 4));
		var report = FillAnalysis.analyze("fixture", "source", "process", cells, [{id: "ceiling", box: box(4, 6, 4, 4)}], [], 10, 2400, 9.80665, null, null,
			[1, 3, 5, 9]);
		var owned:Array<Part> = [];
		try {
			for (b in cells)
				owned.push(ToolingBlock.placedBox(b));
			var source = Part.fuseAll(owned);
			owned.push(source);
			check(Math.abs(source.volume() - report.geometricResinVolumeMm3) < 1e-7, "fill region agrees with fused BREP volume");
			var trappedParts:Array<Part> = [];
			for (pocket in report.trappedPockets) {
				var boxes=switch pocket.geometry {
					case Rectilinear(boxes,_):boxes;
					case CadAirAbove(_,_):throw "This independent control requires the box fast path";
				};
				for (box in boxes) {
					var part = ToolingBlock.placedBox(box);
					owned.push(part);
					trappedParts.push(part);
				}
			}
			var fluid = trappedParts.length == 0 ? source : source.subtractAll(trappedParts);
			if (fluid != source)
				owned.push(fluid);
			check(Math.abs(fluid.volume() - report.totalMixVolumeMm3) < 1e-7, "actual liquid quantity excludes BREP trapped-air region");
			for (level in report.levels) {
				if (level.heightMm == 0)
					continue;
				var mask = ToolingBlock.placedBox({
					x: -1,
					y: -1,
					z: -1,
					width: 10,
					depth: 4,
					height: level.heightMm + 1
				});
				owned.push(mask);
				var below = fluid.intersect(mask);
				owned.push(below);
				check(Math.abs(below.volume() - level.mixVolumeMm3) < 1e-7, "fill curve agrees with independent BREP intersection at " + level.heightMm);
			}
			var sections = OrthogonalSections.analyze("fixture", "source", cells, []);
			for (axis in [SectionAxis.X, SectionAxis.Y, SectionAxis.Z]) {
				var pieces:Array<Part> = [];
				for (span in sections.spans)
					if (span.axis == axis)
						for (strip in span.strips) {
							var part = ToolingBlock.placedBox(strip);
							owned.push(part);
							pieces.push(part);
						}
				var rebuilt = Part.fuseAll(pieces);
				owned.push(rebuilt);
				var missing = source.subtract(rebuilt),
					extra = rebuilt.subtract(source);
				owned.push(missing);
				owned.push(extra);
				check(missing.volume() + extra.volume() < 1e-7, "opposing-face sections reconstruct BREP by symmetric difference on axis " + axis);
			}
			var mask = ToolingBlock.placedBox({
				x: -1,
				y: -1,
				z: 2,
				width: 10,
				depth: 4,
				height: 9
			});
			owned.push(mask);
			var air = source.intersect(mask);
			owned.push(air);
			var count = air.shape.subshapeCount(CadKit.ShapeKind.Solid);
			check(count == 2, "BREP confirms topology at neck closure plane: vented=" + vented + ", count=" + count);
			check(report.trappedPockets.length == (vented ? 0 : 1), "neutral trapped component count matches BREP topology");
			{
				var trappedVolume = 0.0;
				for (i in 0...count) {
					var solid = air.shape.subshape(CadKit.ShapeKind.Solid, i);
					try {
						if (solid.bounds().get_max().get_z() < 10 - 1e-7)
							trappedVolume += solid.volume();
					} catch (e:Dynamic) {
						solid.close();
						throw e;
					}
					solid.close();
				}
				check(Math.abs(trappedVolume - (vented ? 0 : report.trappedPockets[0].volumeMm3)) < 1e-7,
					"trapped-air limiting volume agrees with BREP component");
			}
		} catch (e:Dynamic) {
			for (part in owned)
				if (!part.shape.isClosed())
					part.close();
			throw e;
		}
		for (part in owned)
			if (!part.shape.isClosed())
				part.close();
	}

	static function verifyJunction():Void {
		var boxes:Array<MoldBox> = [
			{
				x: -5,
				y: -1,
				z: -4,
				width: 10,
				depth: 2,
				height: 8
			},
			{
				x: -1,
				y: -5,
				z: -4,
				width: 2,
				depth: 10,
				height: 8
			}
		];
		var report = materia.tooling.process.JunctionThickness.analyze("junction", "source", boxes, [], {diameterToleranceMm: 0.01, maximumCells: 50000});
		var owned:Array<Part> = [];
		try {
			var blocks:Array<Part> = [];
			for (box in boxes) {
				var part = ToolingBlock.placedBox(box);
				owned.push(part);
				blocks.push(part);
			}
			var source = Part.fuseAll(blocks);
			owned.push(source);
			var area = 0.0;
			for (face in report.faces) {
				var b = face.region;
				area += face.axis == SectionAxis.X ? b.depth * b.height : face.axis == SectionAxis.Y ? b.width * b.height : b.width * b.depth;
			}
			check(Math.abs(source.shape.area() - area) < 1e-7, "exposed rectangles agree with independent BREP surface area");
			for (extra in [-1e-6, 0.01]) {
				var ball = new Part(cadkit.Shape.sphere(report.lowerDiameterMm / 2 + extra));
				owned.push(ball);
				var placed = ball.translated(new cadkit.modeling.Vector(report.witness.x, report.witness.y, report.witness.z));
				owned.push(placed);
				var outside = placed.subtract(source);
				owned.push(outside);
				check(extra < 0?outside.volume()<1e-8:outside.volume()>1e-7,
					extra < 0 ? "witness sphere is inside native union" : "enlarging witness sphere crosses native boundary");
			}
			check(report.lowerDiameterMm <= 2 * Math.sqrt(2) + 1e-7 && report.upperDiameterMm >= 2 * Math.sqrt(2) - 1e-7,
				"native cross fixture agrees with analytic global interval");
		} catch (error:Dynamic) {
			for (part in owned)
				if (!part.shape.isClosed())
					part.close();
			throw error;
		}
		for (part in owned)
			if (!part.shape.isClosed())
				part.close();
	}

	public static function main():Void
		run();

	public static function run():Void {
		verify(false);
		verify(true);
		verifyJunction();
		Sys.println('ProcessNativeTests: $assertions assertions passed');
	}
}
