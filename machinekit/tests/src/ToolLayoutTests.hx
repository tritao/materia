import cadkit.ElementNames;
import cadkit.modeling.Part;
import machinekit.pressbrake.ToolClamp;
import machinekit.pressbrake.ToolClampMode;
import machinekit.pressbrake.ToolProfiles;
import machinekit.pressbrake.PunchProfile;
import machinekit.pressbrake.PunchKind;
import machinekit.pressbrake.ToolHorn;
import machinekit.pressbrake.ToolHornEnd;
import machinekit.pressbrake.ToolProfile;
import machinekit.pressbrake.ToolSegment;
import machinekit.pressbrake.ToolRow;
import machinekit.pressbrake.ToolStation;
import machinekit.pressbrake.ToolLayout;
import machinekit.pressbrake.ToolingCatalogue;

/** Segment sums and gap intervals remain distinct throughout paired tooling layouts. */
class ToolLayoutTests {
	public static function main():Void run();

	public static function run():Void {
		var punch = Punch(ToolingCatalogue.straight()), die = SingleV(ToolingCatalogue.dies()[1]);
		for (length in ToolSegment.standardLengths()) {
			var segment = new ToolSegment(punch, length);
			var solid = segment.geometry();
			if (CadKit.shapeValidChecked(solid.shape.borrowHandle()) == 0) throw "Standard segment native validity";
			solid.close();
		}
		var row = new ToolRow([new ToolSegment(punch, 100), new ToolSegment(punch, 50), new ToolSegment(punch, 20)], [10, 15]);
		if (row.length != 170 || row.span != 195) throw "Segment sum and gap span";
		var positions = row.placements(200);
		if (positions[0].start != 200 || positions[0].end != 300 || positions[1].start != 310
			|| positions[1].end != 360 || positions[2].start != 375 || positions[2].end != 395)
			throw "Gaps determine segment bed intervals";
		var dieRow = new ToolRow([new ToolSegment(die, 200)], []);
		var first = new ToolStation("first", row, dieRow, 200);
		if (first.length != 170 || first.span != 200 || first.end != 400) throw "Paired station reserves both rows";
		var second = new ToolStation("second", row, dieRow, 400);
		var layout = new ToolLayout(1600, [second, first]);
		if (layout.stations().length != 2) throw "Touching stations fit the bed";
		var process = first.processTooling();
		if (process.vWidth != 12 || process.punchRadius != 0.6 || process.shoulderRadius != 1)
			throw "Process contact dimensions derive from installed profiles";
		reject(function() { var invalid = new ToolLayout(1600, [first, new ToolStation("overlap", row, dieRow, 399)]); }, "overlap");
		reject(function() { var invalid = new ToolLayout(1600, [new ToolStation("outside", row, dieRow, 1401)]); }, "past bed");
		reject(function() { var invalid = new ToolSegment(punch, 33); }, "nonstandard segment");
		reject(function() { var invalid = new ToolRow(row.segments(), [10]); }, "missing gap");
		reject(function() { var invalid = new ToolRow(row.segments(), [-1, 0]); }, "negative gap");
		reject(function() { var invalid = new ToolStation("wrong", dieRow, row, 0); }, "reversed rows");
		for (profile in [Punch(ToolingCatalogue.straight()), Punch(ToolingCatalogue.gooseneck())]) {
			var plain = new ToolSegment(profile, 100).geometry();
			var plainVolume = plain.volume();
			plain.close();
			var hornVolume = -1.0;
			for (end in [Left, Right]) {
				var segment = new ToolSegment(profile, 100, new ToolHorn(end, 20, 25));
				var part = segment.geometry();
				if (CadKit.shapeValidChecked(part.shape.borrowHandle()) == 0) throw "Horn native geometry";
				if (!(part.volume() > 0 && part.volume() < plainVolume)) throw "Horn removes upper end material";
				if (hornVolume > 0 && Math.abs(hornVolume - part.volume()) > 1e-5) throw "Horn ends remove equal volume";
				hornVolume = part.volume();
				var coverVolume = 0.0;
				for (prism in segment.collisionPrisms(0.01)) {
					var atHorn = end == Left ? prism.start < 20 : prism.end > 80;
					if (atHorn) for (point in prism.section.points())
						if (point.z > 25 + 1e-9) throw "Horn collision prism fills relieved upper end";
					coverVolume += prism.section.area * (prism.end - prism.start);
				}
				if (coverVolume < part.volume() - 1e-5) throw "Horn collision prisms lose material";
				part.close();
			}
		}
		reject(function() { var invalid = new ToolSegment(die, 100, new ToolHorn(Left, 20, 25)); }, "die horn");
		reject(function() { var invalid = new ToolSegment(punch, 20, new ToolHorn(Left, 20, 25)); }, "horn consumes mounting length");
		for (profile in ToolProfiles.catalogueProfiles()) {
			var segment = new ToolSegment(profile, 100);
			var recipe = segment.componentType();
			if (recipe == null) throw "Catalogue tooling recipe missing";
			var rebuilt = recipe.create(segment.values());
			if (recipe.key(rebuilt.values()) != recipe.key(segment.values())) throw "Tool segment recipe inputs changed";
			var original = segment.geometry(), copy = rebuilt.geometry();
			checkNames(original); checkNames(copy);
			if (Math.abs(original.volume() - copy.volume()) > 1e-5) throw "Tool segment recipe geometry changed";
			original.close(); copy.close();
		}
		for (end in [Left, Right]) {
			var segment = new ToolSegment(punch, 100, new ToolHorn(end, 20, 25));
			var recipe = segment.componentType();
			if (recipe == null) throw "Horn segment recipe missing";
			var rebuilt = recipe.create(segment.values());
			var original = segment.geometry(), copy = rebuilt.geometry();
			checkNames(original); checkNames(copy);
			if (Math.abs(original.volume() - copy.volume()) > 1e-5) throw "Horn recipe lost end relief";
			original.close(); copy.close();
		}
		var reference = ToolingCatalogue.straight();
		var spoof = new PunchProfile(reference.id, Gooseneck, reference.tipRadius, reference.includedAngle,
			reference.height, reference.upperSection);
		reject(function() { var invalid = new ToolRow([new ToolSegment(punch, 100), new ToolSegment(Punch(spoof), 100)], [0]); },
			"different profile specifications sharing an identity");
		if (new ToolSegment(Punch(spoof), 100).componentType() != null)
			throw "Custom profile reused catalogue identity without matching its specification";
		for (mode in [Manual, Powered]) {
			var clamp = new ToolClamp(mode);
			switch clamp.control {
				case Manual:
					if (mode != ToolClampMode.Manual || clamp.ports().length != 0) throw "Manual clamp interface";
				case Powered(lock, unlock, locked, air):
					if (mode != ToolClampMode.Powered || clamp.ports().length != 4) throw "Powered clamp interface";
					if (clamp.port(lock).kind != Signal || clamp.port(unlock).kind != Signal
						|| clamp.port(locked).kind != Signal || clamp.port(air).kind != Pneumatic)
						throw "Powered clamp signal and air port kinds";
					if (clamp.port(lock).role != Consumer || clamp.port(unlock).role != Consumer
						|| clamp.port(locked).role != Supply) throw "Powered clamp command and feedback roles";
			}
			var recipe = clamp.componentType();
			if (recipe == null) throw "Tool clamp recipe missing";
			var rebuilt = recipe.create(clamp.values());
			if (recipe.key(rebuilt.values()) != recipe.key(clamp.values())) throw "Clamp recipe inputs changed";
			var part = rebuilt.geometry();
			if (CadKit.shapeValidChecked(part.shape.borrowHandle()) == 0 || Math.abs(part.volume() - 120000) > 1e-5)
				throw "Clamp recipe geometry";
			part.close();
		}
		trace("Tool layout tests passed");
	}

	static function checkNames(part:Part):Void {
		var seen:Map<String, Bool> = [];
		for (name in part.shape.elementNames(CadKit.ShapeKind.Face)) {
			if (!ElementNames.isStrong(name) || seen.exists(name)) throw 'Tooling face name is weak or repeated: $name';
			seen.set(name, true);
		}
	}

	static function reject(action:Void->Void, label:String):Void {
		var rejected = false;
		try { action(); } catch (_:Dynamic) rejected = true;
		if (!rejected) throw 'Tool layout accepted $label';
	}
}
