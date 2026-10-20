import machinekit.structural.MineralCasting;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import machinekit.assembly.PosedParts;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import materia.casting.CastingCavities;
import materia.casting.CastingPatterns;

/** Independent frozen pre-migration CAD plus exact neutral/CAD intersection checks. */
class FoundationDesignTests {
	static var assertions = 0;

	static function same(a:Part, b:Part, id:String):Void {
		var added = a.subtract(b),
			removed = b.subtract(a),
			difference = added.volume() + removed.volume();
		added.close();
		removed.close();
		assertions++;
		if (difference > 1e-3)
			throw id + " symmetric difference: " + difference;
		Sys.println(id + " symmetric difference: " + difference);
	}

	static function prism(p:materia.casting.CastingPrism):Part {
		var local = Part.prism([for (v in p.profile) new Vector(v.x, v.y)], 0, p.height);
		try {
			var out = PosedParts.moved(local, p.pose);
			local.close();
			return out;
		} catch (e:Dynamic) {
			local.close();
			throw e;
		}
	}

	static function slopedFoot():Void {
		var values = new ComponentValues().setNumber("width", 100)
			.setNumber("depth", 80)
			.setNumber("height", 120)
			.setNumber("footWidth", 160)
			.setNumber("footHeight", 60)
			.setNumber("footShoulder", 20);
		var source = new MineralCasting(values), body = source.geometry();
		var feature = CastingPatterns.box("foot-crossing", materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening, [], 60, 30, 30, 65, 0, 25);
		feature.openings = CastingCavities.reachedOpenings(source.castingEnvelope(), feature.geometry);
		var cavity = CastingCavities.derive(source.castingEnvelope(), [feature])[0];
		var localTool = Part.box(60, 30, 30),
			tool = localTool.translated(new Vector(65, 0, 25));
		localTool.close();
		var reference = tool.intersect(body);
		var actual = Solids.union([for (p in cavity.pieces) prism(p)]);
		same(reference, actual, "foot-crossing cut/body intersection");
		reference.close();
		actual.close();
		tool.close();
		body.close();
		var message = "";
		try
			CastingCavities.boxes(cavity)
		catch (e:Dynamic)
			message = Std.string(e);
		assertions++;
		if (message.indexOf("Unsupported non-box finished cavity: foot-crossing") < 0)
			throw "Box consumer lost feature ID";
	}

	public static function main():Void {
		var cwd = Sys.getCwd(),
			root = (sys.FileSystem.exists(cwd + "/machinekit") ? cwd + "/machinekit/tests/tooling" : cwd) + "/fixtures/foundation-design";
		var frame = MillMoldSetup.frame();
		for (row in [{id: "mill-base", part: frame.base}, {id: "mill-column", part: frame.column}]) {
			var process = sys.io.Process.run("python3", [
				"-c",
				"import sys,zlib;sys.stdout.buffer.write(zlib.decompress(open(sys.argv[1],'rb').read()))",
				root + "/" + row.id + ".step.zlib"
			]);
			var text = process.readStdout(), status = process.exitCode();
			process.close();
			if (status != 0)
				throw "Cannot decode frozen migration CAD fixture";
			var previous = new Part(cadkit.Shape.importStepText(text)),
				actual = row.part.body.geometry();
			same(previous, actual, row.id);
			previous.close();
			actual.close();
		}
		slopedFoot();
		Sys.println("FoundationDesignTests: " + assertions + " assertions passed");
	}
}
