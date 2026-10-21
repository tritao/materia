package machinekit.structural;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.casting.InsertSpec.InsertRole;
import materia.casting.CastingSolid;
import materia.casting.InsertSpec.InsertBody;
import materia.casting.InsertSpec.InsertBodyCodec;

/** Cast-in steel design envelope; local +Z is the finished mounting-plane normal. */
class CastInsert extends MachineComponent {
	static var recipe:Null<ComponentType>;

	public static function parameters():Array<ComponentParameter>
		return [
			P.choice("stockMaterial", materia.casting.InsertMaterial.InsertMaterials.tokens(), materia.casting.InsertMaterial.C45Bright),
			P.text("holes", InsertHoles.fromNeutral([])),
			P.text("interfaces", "[]"),
			P.text("body", InsertBodyCodec.encode(Primitive(Box(35, 400, 20)))),
			P.text("origin", haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(Fabricated))),
			P.length("positionTolerance", 0.01),
			P.requiredChoice("role", materia.casting.InsertSpec.InsertRoles.tokens()),
			P.text("label", "rail seat"),
			P.choice("finish", materia.casting.InsertSpec.InsertFinishes.tokens(), materia.casting.InsertSpec.InsertFinish.GroundAfterCasting)
		];

	public static function recipeType():ComponentType {
		if (recipe == null)
			recipe = new ComponentType("machinekit.structural.cast-in-steel-insert", parameters(), v -> new CastInsert(v), false, false, null, false,
				new ComponentValues().setToken("role", InsertRole.GenericMount));
		return recipe;
	}

	public function new(?overrides:ComponentValues) {
		var v = P.resolve(parameters(), overrides);
		var body = InsertBodyCodec.decode(v.token("body"));
		var holes = InsertHoles.records(v.token("holes"));
		v.setToken("body", InsertBodyCodec.encode(body));
		v.setToken("holes", InsertHoles.fromNeutral(holes));
		var precision = materia.casting.InterfaceRequirement.InterfaceRequirements.snapshot(haxe.Json.parse(v.token("interfaces")));
		v.setToken("interfaces", haxe.Json.stringify(materia.casting.InterfaceRequirement.InterfaceRequirements.records(precision)));
		var origin = materia.casting.InsertOrigin.InsertOriginCodec.snapshot(haxe.Json.parse(v.token("origin")), body, holes);
		v.setToken("origin", haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(origin)));
		if (v.number("positionTolerance") <= 0)
			throw "Insert tolerance must be positive";
		if (StringTools.trim(v.token("label")).length == 0)
			throw "Insert needs a label";
		var materialId = stockMaterialId(cast v.token("stockMaterial"));
		if (v.get("material") != null && materia.project.MaterialLibrary.fromSpec(v.token("material")) != materialId)
			throw "Cast insert material differs from its declared stock grade";
		super("CAST-IN-INSERT", "Cast-in steel " + v.token("label") + ", " + v.token("finish"),
			materia.project.MaterialLibrary.require(materialId).physical.spec, false, v);
		var stock = switch materia.casting.InsertSpec.InsertBodyCodec.mouth(body) {
			case Box(width, depth, height): {width: width, depth: depth, height: height};
			case Cylinder(diameter, height): {width: diameter, depth: diameter, height: materia.casting.InsertSpec.InsertBodyCodec.height(body)};
		};
		var sourceHoles = InsertHoles.facet(holes, stock.width, stock.depth, stock.height);
		addFacet(new machinekit.design.DesignInterfaces(sourceHoles.holes, sourceHoles.datums, sourceHoles.material, precision));
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("top", Mount, Solids.axial(0, 0, stock.height));
	}

	public static function boxValues(width:Float, depth:Float, height:Float):ComponentValues
		return new ComponentValues().setToken("body", InsertBodyCodec.encode(Primitive(Box(width, depth, height))));

	public function bodySpec():InsertBody
		return InsertBodyCodec.decode(parameterToken("body"));

	public function boxDimensions(id:String):{width:Float, depth:Float, height:Float}
		return InsertBodyCodec.box(bodySpec(), id);

	public function insertMaterial():materia.casting.InsertMaterial.InsertMaterialSpec
		return materia.casting.InsertMaterial.InsertMaterials.specification(cast parameterToken("stockMaterial"));

	public function insertSpec():materia.casting.InsertSpec {
		var body = bodySpec(), holes = InsertHoles.copy(machinekit.design.DesignInterfaces.of(this));
		return {
			body: body,
			holes: holes,
			origin: materia.casting.InsertOrigin.InsertOriginCodec.snapshot(haxe.Json.parse(parameterToken("origin")), body, holes)
		};
	}

	static function stockMaterialId(material:materia.casting.InsertMaterial):String {
		return switch material {
			case C45Bright, C45Ground: "steel-c45";
			case Stainless14571: "stainless-sheet";
			default: throw "Unsupported insert stock grade";
		};
	}

	override public function componentType():Null<ComponentType>
		return recipeType();

	override public function hasGeometry():Bool
		return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.named(InsertBodies.part(insertSpec().body), "insert");
		return Solids.building([body], owned -> {
			var tools = machinekit.design.DesignInterfaces.of(this).drillingTools();
			for (tool in tools)
				owned.push(tool);
			return tools.length == 0 ? body : Solids.cut(body, tools);
		});
	}

	override public function setMaterial(spec:String):Void {
		if (materia.project.MaterialLibrary.fromSpec(spec) != stockMaterialId(cast parameterToken("stockMaterial")))
			throw "Cast insert material differs from its declared stock grade";
		super.setMaterial(spec);
	}
}
