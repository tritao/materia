import cadkit.modeling.Part;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.process.FillAnalysis;
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
			check(Math.abs(source.volume() - report.totalMixVolumeMm3) < 1e-7, "fill region agrees with fused BREP volume");
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
				var below = source.intersect(mask);
				owned.push(below);
				check(Math.abs(below.volume() - level.mixVolumeMm3) < 1e-7, "fill curve agrees with independent BREP intersection at " + level.heightMm);
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

	public static function main():Void
		run();

	public static function run():Void {
		verify(false);
		verify(true);
		Sys.println('ProcessNativeTests: $assertions assertions passed');
	}
}
