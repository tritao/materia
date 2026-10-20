import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.structural.MineralCasting;
import machinekit.structural.MineralFoundation;
import machinekit.structural.CastInsert;
import materia.casting.CastingPatterns;
import machinekit.structural.FoundationIntegrity;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;
import materia.project.MaterialLibrary;

/** Exercises persistence, physical partitioning and layout validation through public APIs. */
class CastingTests {
	static var assertions = 0;

	static function check(ok:Bool, label:String):Void {
		assertions++;
		if (!ok)
			throw label;
	}

	static function near(a:Float, b:Float, tolerance:Float, label:String):Void
		check(Math.abs(a - b) <= tolerance, '$label: $a vs $b');

	static function rejects(fn:Void->Void, label:String):Void {
		var failed = false;
		try
			fn()
		catch (_:Dynamic)
			failed = true;
		check(failed, label);
	}

	public static function main():Void
		run();

	public static function run():Void {
		var input = P.resolve(InputProbe.parameters(), new ComponentValues().setNumber("length", 25));
		var probe = new InputProbe(input);
		input.setNumber("length", 999);
		near(probe.parameterNumber("length"), 25, 0, "constructor owns its inputs");
		var snapshot = probe.values();
		snapshot.setNumber("length", 888);
		near(probe.parameterNumber("length"), 25, 0, "values are snapshots");
		probe.setMaterial("aluminium 6061");
		check(probe.values().token("material") == "aluminium 6061", "current material is serialized");
		check(probe.parameterCount("count") == 2 && probe.parameterToken("label") == "reference", "typed inputs");
		rejects(() -> P.resolve(InputProbe.parameters(), new ComponentValues().setNumber("typo", 1)), "unknown constructor input");
		rejects(() -> P.resolve(InputProbe.parameters(), new ComponentValues().setNumber("length", Math.NaN)), "nonfinite input");
		rejects(() -> P.resolve(InputProbe.parameters(), new ComponentValues().setToken("length", "x")), "wrong input type");
		rejects(() -> P.resolve([P.length("x", 1), P.length("x", 2)]), "duplicate schema input");
		var type = InputProbe.recipeType();
		check(type.key(type.resolve(new ComponentValues().setNumber("length", 25).setToken("material", "aluminium 6061"))) == type.key(probe.values()),
			"constructor and recipe use the same resolver");
		rejects(() -> type.resolve(new ComponentValues().setNumber("typo", 1)), "unknown recipe input");

		var core = CastingPatterns.box("core",materia.casting.CastingFeature.CastingFeatureKind.Lightening,[], 120, 100, 60, 0, 0, 20);
		var v = new ComponentValues().setNumber("width", 200)
			.setNumber("depth", 180)
			.setNumber("height", 100)
			.setToken("features", CastingPatterns.encode([core]));
		var casting = new MineralCasting(v), shape = casting.geometry();
		near(shape.volume(), 200 * 180 * 100 - 120 * 100 * 60, 1e-5, "core removes only its own volume");
		near(casting.castingByWeight().kg, shape.volume() * 1e-9 * 2400, 1e-9, "casting mass uses shared density");
		shape.close();
		var registry = MachineKitComponents.standard();
		var parts:Array<MachineComponent> = [
			casting,
			new CastInsert(new ComponentValues().setToken("role", "generic-mount")),
			new machinekit.milling.MillSupportTray(new ComponentValues().setNumber("width", 1200)
				.setNumber("depth", 900)
				.setNumber("millX", -80)
				.setNumber("millY", 30))
		];
		for (part in parts) {
			var recipe = part.componentType();
			if (recipe == null)
				throw "Missing casting recipe";
			var rebuilt = registry.byId(recipe.id).create(part.values());
			check(recipe.key(part.values()) == recipe.key(rebuilt.values()), "recipe roundtrip");
			near(part.massProperties().mass, rebuilt.massProperties().mass, 1e-9, "roundtrip mass");
			var solid = rebuilt.geometry();
			check(solid.valid() && solid.solidCount() == 1, "registered casting/insert/tray is one valid solid");
			solid.close();
		}
		rejects(() -> new MineralCasting(new ComponentValues().setToken("material", "steel")), "casting material lock");
		rejects(() -> new CastInsert(new ComponentValues().setToken("role", "generic-mount").setNumber("positionTolerance", 0)), "insert tolerance positive");
		rejects(() -> new MineralCasting(new ComponentValues().setNumber("cornerChamfer", 400)), "oversized chamfer");
		rejects(() -> CastingPatterns.decode('[{"id":"missing-dimensions"}]'), "malformed feature");
		rejects(() -> CastingPatterns.encode([core, core]), "duplicate feature ids");
		rejects(() -> CastingPatterns.grid(100, 100, 100, 60, 20, 20, 25), "impossible core dimensions");
		var cores = CastingPatterns.grid(650, 550, 120, 30, 20, 20, 25);
		check(cores.length == 4, "cross-ribbed layout has four cores");

		var insert = new CastInsert(new ComponentValues().setNumber("width", 40)
			.setNumber("depth", 30)
			.setNumber("height", 20)
			.setToken("role", "generic-mount")
			.setToken("label", "test mount"));
		var placements:Array<CastInsertPlacement> = [{id: "seat", part: insert,
			openings: [CastingPatterns.face("body.z-max"), CastingPatterns.face("body.x-max")], pose: F.translation(80, 0, 90)}];
		var breakout = "";
		try new MineralFoundation(v, [{id: "seat", part: insert,
			openings: [CastingPatterns.face("body.z-max")], pose: F.translation(80, 0, 90)}])
		catch (error:Dynamic) breakout = Std.string(error);
		check(breakout.indexOf("Undeclared casting breakout: insert-seat body.x-max") >= 0,
			"source insert recess openings are declared by the caller");
		var foundation = new MineralFoundation(v, placements);
		var poses = foundation.solvedPoses(),
			body = foundation.body.geometry();
		var localSteel = insert.geometry(),
			steel = PosedParts.moved(localSteel, poses.get("seat"));
		localSteel.close();
		var common = body.intersect(steel);
		near(common.volume(), 0, 1e-5, "insert has a matching recess, no double-counted volume");
		near(body.volume(), 200 * 180 * 100 - 120 * 100 * 60 - 40 * 30 * 10, 1e-5, "only embedded steel displaces casting");
		common.close();
		steel.close();
		body.close();
		check(!FoundationIntegrity.check(foundation).hasErrors(), "nominal insert planes");
		var movedSeat = poses.get("seat");
		if (movedSeat == null)
			throw "Missing seat pose";
		movedSeat.z += 0.02;
		check(FoundationIntegrity.check(foundation, poses).hasErrors(), "insert plane displacement is detected");
		check(FoundationIntegrity.checkMounts(foundation, [{name: "bad", parent: "casting"}]).hasErrors(), "precision mount requires steel");
		check(!FoundationIntegrity.checkMounts(foundation, [{name: "good", parent: "seat"}]).hasErrors(), "steel precision mount");
		rejects(() -> new MineralFoundation(v, [placements[0], placements[0]]), "duplicate insert ids");
		rejects(() -> new MineralFoundation(v, placements, "missing"), "unknown root insert");
		var restored = MachineAssembly.decode(foundation.encode(), registry);
		check(restored.billOfMaterials().lines().length == 2, "casting and steel are separate BOM items");
		for (member in restored.components()) {
			var original:MachineComponent = foundation.body;
			if (member.id != "casting")
				original = insert;
			near(member.component.massProperties().mass, original.massProperties().mass, 1e-9, "assembly roundtrip mass");
		}
		var references:Array<MineralFoundation> = [
			new machinekit.hobbing.MineralCastBed(),
			new machinekit.hobbing.MineralCastColumn()
		];
		for (reference in references) {
			check(!FoundationIntegrity.check(reference).hasErrors(), "hobber insert alignment");
			var decoded = MachineAssembly.decode(reference.encode(), registry);
			check(decoded.components().length == reference.components().length, "hobber reference layout roundtrip");
			var referenceBody = reference.body.geometry();
			check(referenceBody.volume() > 0 && referenceBody.valid() && referenceBody.solidCount() == 1, "hobber casting is one valid solid");
			referenceBody.close();
		}
		var material = MaterialLibrary.require("mineral-cast").physical;
		check(material.propertiesAssumed == true && material.youngsModulusPa == 35e9, "bulk properties retain assumed provenance");
		var bored = new CastInsert(new ComponentValues().setToken("role", "generic-mount")
			.setNumber("width", 40)
			.setNumber("depth", 40)
			.setNumber("height", 20)
			.setToken("holes",machinekit.structural.InsertHoles.through(18,20)));
		var boredShape = bored.geometry();
		check(boredShape.valid() && boredShape.solidCount() == 1, "bored foot seat remains one solid");
		near(boredShape.volume(), 40 * 40 * 20 - Math.PI * 9 * 9 * 20, 1e-6, "foot bore removes steel volume");
		boredShape.close();
		rejects(() -> new CastInsert(new ComponentValues().setToken("role", "generic-mount").setToken("holes",machinekit.structural.InsertHoles.through(40,20))),
			"reject insert bore consuming seat");
		Sys.println('CastingTests: $assertions assertions passed');
		if (Sys.args().indexOf("--recipes") >= 0) {
			RecipeContractTests.run();
			Sys.println("CastingTests: registered recipe contracts passed");
		}
	}
}

/** No geometry or engineering assumptions are imposed by stored parameter support. */
class InputProbe extends MachineComponent {
	static var recipe:Null<ComponentType>;

	public static function parameters():Array<ComponentParameter>
		return [P.length("length", 10), P.count("count", 2), P.text("label", "reference")];

	public static function recipeType():ComponentType {
		if (recipe == null)
			recipe = new ComponentType("test.input-probe", parameters(), v -> new InputProbe(v));
		return recipe;
	}

	public function new(?input:ComponentValues) {
		var v = P.resolve(parameters(), input);
		super("INPUT-PROBE", "Input probe", v.get("material") == null ? "steel" : v.token("material"), false, v);
	}

	override public function componentType():Null<ComponentType>
		return recipeType();
}
