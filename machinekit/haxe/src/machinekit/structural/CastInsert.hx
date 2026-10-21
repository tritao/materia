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
	final sourceDesign:machinekit.design.DesignInterfaces;

	public static function parameters():Array<ComponentParameter>
		return [
			P.choice("buildStage",materia.casting.InsertStages.tokens(),materia.casting.InsertStages.InsertStage.Service),
			P.choice("stockMaterial", materia.casting.InsertMaterial.InsertMaterials.tokens(), materia.casting.InsertMaterial.C45Bright),
			P.text("holes", InsertHoles.fromNeutral([])),
			P.text("interfaces", "[]"),
			P.text("anchorage", haxe.Json.stringify(materia.casting.InsertAnchorage.InsertAnchorageCodec.record(Undeclared))),
			P.text("body", InsertBodyCodec.encode(Primitive(Box(35, 400, 20)))),
			P.text("origin", haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(Fabricated(
				materia.casting.InsertManufacturing.InsertManufacturingCodec.draft(
					materia.casting.InsertMaterial.InsertMaterials.specification(materia.casting.InsertMaterial.C45Bright)))))),
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
		v.setToken("interfaces", haxe.Json.stringify(materia.casting.InterfaceRequirement.InterfaceRequirements.records(
			materia.casting.InterfaceRequirement.InterfaceRequirements.snapshot(haxe.Json.parse(v.token("interfaces"))))));
		var origin = materia.casting.InsertOrigin.InsertOriginCodec.snapshot(haxe.Json.parse(v.token("origin")), body, holes);
		switch origin {
			case Fabricated(manufacturing):
				if (manufacturing.material.grade!=v.token("stockMaterial")) throw "Insert manufacturing grade differs from source stock";
				materia.casting.InsertManufacturing.InsertManufacturingCodec.references(manufacturing,holes,
					materia.casting.InterfaceRequirement.InterfaceRequirements.snapshot(haxe.Json.parse(v.token("interfaces"))));
			case Catalogue(_):
		}
		v.setToken("origin", haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(origin)));
		var anchorage=materia.casting.InsertAnchorage.InsertAnchorageCodec.snapshot(haxe.Json.parse(v.token("anchorage")));
		materia.casting.InsertAnchorage.InsertAnchorageCodec.references(anchorage,body,holes,origin,
			materia.casting.InterfaceRequirement.InterfaceRequirements.snapshot(haxe.Json.parse(v.token("interfaces"))));
		v.setToken("anchorage",haxe.Json.stringify(materia.casting.InsertAnchorage.InsertAnchorageCodec.record(anchorage)));
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
		var interfaces=materia.casting.InterfaceRequirement.InterfaceRequirements.snapshot(haxe.Json.parse(v.token("interfaces")));
		var finished=switch materia.casting.InsertFinishing.assess(body,interfaces) {
			case Resolved(steel):steel;
			case Invalid(_),Unsupported(_):null;
		};
		var serviceStock=finished==null ? null : materia.casting.InsertFinishing.box(finished);
		if (serviceStock!=null && finished!=null && finished.removedSteel.length>0) for (hole in holes)
			materia.casting.InsertStages.validateServiceHole(hole,serviceStock);
		var holeInterfaces = serviceStock==null ? InsertHoles.facet(holes,stock.width,stock.depth,stock.height)
			: InsertHoles.facet(holes,serviceStock.width,serviceStock.depth,serviceStock.height);
		sourceDesign=new machinekit.design.DesignInterfaces(holeInterfaces.holes,holeInterfaces.datums,holeInterfaces.material,
			materia.casting.InterfaceRequirement.InterfaceRequirements.snapshot(haxe.Json.parse(v.token("interfaces"))));
		if (buildStage()==materia.casting.InsertStages.InsertStage.Service) addFacet(sourceDesign);
		else {
			sourceDesign.check(this);
			var selected=materia.casting.InsertStages.holes(
				{body:body,holes:holes,origin:origin,anchorage:anchorage,interfaces:sourceDesign.precisionRequirements()},buildStage());
			var physical=InsertHoles.facet(selected,stock.width,stock.depth,stock.height);
			addFacet(physical);
		}
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		if (buildStage()==materia.casting.InsertStages.InsertStage.Service && serviceStock!=null)
			addConnector("top",Mount,Solids.axial(serviceStock.x+serviceStock.width/2,
				serviceStock.y+serviceStock.depth/2,serviceStock.z+serviceStock.height));
		else addConnector("top", Mount, Solids.axial(0, 0, stock.height));
	}

	public static function boxValues(width:Float, depth:Float, height:Float):ComponentValues
		return manufacturedValues(materia.casting.InsertMaterial.C45Bright)
			.setToken("body", InsertBodyCodec.encode(Primitive(Box(width, depth, height))));

	/** Grade and the attached manufacturing choices are authored together, so a
	 * recipe rebuild never depends on whether defaults were resolved beforehand.
	 */
	public static function manufacturedValues(material:materia.casting.InsertMaterial):ComponentValues
		return new ComponentValues().setToken("stockMaterial",material).setToken("origin",
			haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(Fabricated(
				materia.casting.InsertManufacturing.InsertManufacturingCodec.draft(
					materia.casting.InsertMaterial.InsertMaterials.specification(material))))));

	/** Explicit source authoring for holes which must exist during the pour.
	 * Bought-part mounting holes use their separate connector/clearance-budget route.
	 */
	public static function precastHoles(values:ComponentValues,encodedHoles:String):ComponentValues {
		var result=values.copy(),holes=InsertHoles.records(encodedHoles);
		var origin=materia.casting.InsertOrigin.InsertOriginCodec.snapshot(haxe.Json.parse(result.token("origin")),
			InsertBodyCodec.decode(result.token("body")),holes);
		switch origin {
			case Fabricated(manufacturing):
				manufacturing.holeRoutes=[for (hole in holes) {holeId:hole.id,
					route:materia.casting.InsertManufacturing.InsertHoleRoute.BeforeCasting}];
			case Catalogue(_):throw "Catalogue holes are supplied by the product, not a made-insert hole route";
		}
		return result.setToken("holes",InsertHoles.fromNeutral(holes)).setToken("origin",
			haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(origin)));
	}

	public function bodySpec():InsertBody
		return InsertBodyCodec.decode(parameterToken("body"));

	public function boxDimensions(id:String):{width:Float, depth:Float, height:Float}
		return InsertBodyCodec.box(bodySpec(), id);

	public function insertMaterial():materia.casting.InsertMaterial.InsertMaterialSpec
		return materia.casting.InsertMaterial.InsertMaterials.specification(cast parameterToken("stockMaterial"));

	public function buildStage():materia.casting.InsertStages.InsertStage
		return cast parameterToken("buildStage");

	/** Complete source design; the published physical facet follows buildStage. */
	public function insertSpec():materia.casting.InsertSpec {
		var body = bodySpec(), holes = InsertHoles.copy(sourceDesign);
		return {
			body: body,
			anchorage:materia.casting.InsertAnchorage.InsertAnchorageCodec.snapshot(haxe.Json.parse(parameterToken("anchorage"))),
			holes: holes,
			origin: materia.casting.InsertOrigin.InsertOriginCodec.snapshot(haxe.Json.parse(parameterToken("origin")), body, holes),
			interfaces:sourceDesign.precisionRequirements()
		};
	}

	override function get_bom():BomItem {
		var item=super.get_bom();
		switch insertSpec().origin {
			case Fabricated(_):item.manufacturingProcess=materia.casting.InsertManufacturing.InsertMakeProcess.MachinedSteel;
			case Catalogue(_):
		}
		return item;
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

	override public function geometry(detail:ComponentDetail = Preview):Part
		return InsertGeometry.part(insertSpec(),buildStage());

	override public function setMaterial(spec:String):Void {
		if (materia.project.MaterialLibrary.fromSpec(spec) != stockMaterialId(cast parameterToken("stockMaterial")))
			throw "Cast insert material differs from its declared stock grade";
		super.setMaterial(spec);
	}
}
