package machinekit.structural;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.tooling.ToolingEnums.InsertRole;

/** Cast-in steel design envelope; local +Z is the finished mounting-plane normal. */
class CastInsert extends MachineComponent {
	static var recipe:Null<ComponentType>;

	public static function parameters():Array<ComponentParameter>
		return [
			P.length("boreDiameter", 0),
			P.length("width", 35),
			P.length("depth", 400),
			P.length("height", 20),
			P.length("positionTolerance", 0.01),
			P.choice("role", [
				InsertRole.TableMount,
				InsertRole.LiftingMount,
				InsertRole.ThreadedAnchor,
				InsertRole.GenericMount,
				InsertRole.GuideSeat,
				InsertRole.BearingSeat,
				InsertRole.ColumnMount,
				InsertRole.LevelingSeat,
				InsertRole.HatchMount
			], InsertRole.GuideSeat),
			P.text("label", "rail seat"),
			P.choice("finish", [
				"ground after casting",
				"machined after casting",
				"replicated after casting",
				"as cast"
			], "ground after casting")
		];

	public static function recipeType():ComponentType {
		if (recipe == null)
			recipe = new ComponentType("machinekit.structural.cast-in-steel-insert", parameters(), v -> new CastInsert(v), false, false, null, false);
		return recipe;
	}

	public function new(?overrides:ComponentValues) {
		var v = P.resolve(parameters(), overrides);
		for (name in ["width", "depth", "height", "positionTolerance"])
			if (v.number(name) <= 0)
				throw "Insert dimensions and tolerance must be positive";
		if (v.number("boreDiameter") >= Math.min(v.number("width"), v.number("depth")))
			throw "Insert bore consumes its seat";
		if (StringTools.trim(v.token("label")).length == 0)
			throw "Insert needs a label";
		if (v.get("material") != null && v.token("material") != "steel")
			throw "Cast insert requires steel";
		super("CAST-IN-INSERT", "Cast-in steel " + v.token("label") + ", " + v.token("finish"), "steel", false, v);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("top", Mount, Solids.axial(0, 0, parameterNumber("height")));
	}

	override public function componentType():Null<ComponentType>
		return recipeType();

	override public function hasGeometry():Bool
		return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.named(Part.box(parameterNumber("width"), parameterNumber("depth"), parameterNumber("height")), "insert");
		return parameterNumber("boreDiameter") == 0 ? body : Solids.cut(body, [
			Part.cylinderSpan(parameterNumber("boreDiameter") / 2, -1, parameterNumber("height") + 1)
		]);
	}

	override public function setMaterial(spec:String):Void {
		if (spec != "steel")
			throw "Cast insert requires steel";
		super.setMaterial(spec);
	}
}
