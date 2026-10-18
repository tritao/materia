package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.structural.CastingFeatures.CastingBox;

/** A mineral casting design body. Layouts supply cores and recesses; no process adequacy implied. */
class MineralCasting extends MachineComponent {
	final voids:Array<CastingBox>;
	static var recipe:Null<ComponentType>;

	public static function parameters():Array<ComponentParameter> return [
		P.length("width", 650), P.length("depth", 550), P.length("height", 120),
		P.length("cornerChamfer", 0), P.text("voids", "[]")];

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.structural.mineral-casting", parameters(),
			v -> new MineralCasting(v), false, false, null, false);
		return recipe;
	}

	public function new(?overrides:ComponentValues) {
		var v = P.resolve(parameters(), overrides);
		var cuts = CastingFeatures.decode(v.token("voids"));
		v.setToken("voids", CastingFeatures.encode(cuts));
		if (v.get("material") != null && v.token("material") != "mineral cast") throw "Casting requires mineral cast";
		for (name in ["width", "depth", "height"]) if (v.number(name) <= 0) throw "Casting dimensions must be positive";
		if (2 * v.number("cornerChamfer") >= Math.min(v.number("width"), v.number("depth")))
			throw "Casting chamfer consumes the body";
		super("MINERAL-CASTING", "Mineral casting (purchase by casting mass, kg)", "mineral cast", false, v);
		voids = cuts;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("top", Mount, Solids.axial(0, 0, parameterNumber("height")));
	}

	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function setMaterial(spec:String):Void {
		if (spec != "mineral cast") throw "Casting requires mineral cast";
		super.setMaterial(spec);
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var x = parameterNumber("width") / 2, y = parameterNumber("depth") / 2;
		var h = parameterNumber("height"), c = parameterNumber("cornerChamfer");
		var body = Solids.named(c == 0 ? Part.box(2*x, 2*y, h) : Part.prism([
			new Vector(-x+c,-y), new Vector(x-c,-y), new Vector(x,-y+c), new Vector(x,y-c),
			new Vector(x-c,y), new Vector(-x+c,y), new Vector(-x,y-c), new Vector(-x,-y+c)], 0, h), "casting");
		return Solids.building([body], parts -> {
			for (feature in voids) {
				var tool = Part.box(feature.width, feature.depth, feature.height);
				try {
					parts.push(Solids.named(PosedParts.moved(tool, feature.pose), feature.id));
					tool.close();
				} catch (error:Dynamic) { tool.close(); throw error; }
			}
			return Solids.cut(parts[0], parts.slice(1));
		});
	}

	public function castingByWeight():{material:String, kg:Float}
		return {material: materialSpec(), kg: massProperties().mass};
}
