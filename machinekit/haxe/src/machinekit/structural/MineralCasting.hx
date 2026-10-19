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
		P.length("footWidth", 0), P.length("footHeight", 0), P.length("footShoulder", 0),
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
		if (!Math.isFinite(v.number("footWidth")+v.number("footHeight")+v.number("footShoulder")) ||
			(v.number("footWidth")==0 && (v.number("footHeight")!=0 || v.number("footShoulder")!=0))) throw "Incomplete casting foot";
		if (v.number("footWidth") != 0 && (v.number("footWidth") <= v.number("width") ||
			v.number("footShoulder") <= 0 || v.number("footHeight") <= v.number("footShoulder") ||
			v.number("footHeight") >= v.number("height"))) throw "Casting foot needs a wider sole and a tapered shoulder";
		super("MINERAL-CASTING", "Mineral casting (purchase by casting mass, kg)", "mineral cast", false, v);
		voids = cuts;
    var orthogonal=true;for(cut in cuts)if(!CastingCells.orthogonal(cut))orthogonal=false;
    if(orthogonal&&v.number("cornerChamfer")==0&&v.number("footWidth")==0)
      addFacet(CollisionHullFacet.fromBoxes(CastingCells.compact(CastingCells.material(v.number("width"),v.number("depth"),v.number("height"),cuts))));
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
		if (parameterNumber("footWidth") > 0) {
			var f = parameterNumber("footWidth")/2, top = parameterNumber("footHeight"), shoulder = parameterNumber("footShoulder");
			var profile = Part.prism([new Vector(-f,0),new Vector(f,0),new Vector(f,shoulder),
				new Vector(x,top),new Vector(-x,top),new Vector(-f,shoulder)],0,2*y);
			var foot = PosedParts.moved(profile, materia.assembly.AssemblyFrames.fromRotationMatrix(0,y,0,[1,0,0, 0,0,-1, 0,1,0]));
			profile.close();
			body = Solids.union([body,foot]);
		}
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
