package machinekit.structural;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.casting.InsertHole;
import materia.casting.InsertMountHoleRequirement;
import materia.casting.InsertMountHoleRequirement.InsertMountHoleRequirements;

/** A fabricated C45 mounting member, not a cast-in insert. Shared rail/rack/
 * scale receiver and adjustment steel rebuilds from its source drilling.
 * Welded host pads remain separate parts; this member is bolted to them.
 */
class LinearMountSteel extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public final width:Float;
	public final length:Float;
	public final thickness:Float;
	final drilling:Array<InsertHole>;
	final lineage:Array<InsertMountHoleRequirement>;

	static function inputs():Array<ComponentParameter> return [P.length("width",28),P.length("length",500),
		P.length("thickness",20),P.choice("role",["receiver","adjustment-bar","carrier"],"receiver"),
		P.text("holes",InsertHoles.fromNeutral([])),P.text("mountingRequirements","[]"),P.text("compliance","null")];

	public static function recipeType():ComponentType {
		if(recipe==null)recipe=new ComponentType("machinekit.structural.linear-mount-steel",inputs(),
			v->new LinearMountSteel(v),false,false,null,false);
		return recipe;
	}

	public function new(?overrides:ComponentValues) {
		var v=P.resolve(inputs(),overrides,false);
		width=v.number("width");length=v.number("length");thickness=v.number("thickness");
		for(value in [width,length,thickness])if(!Math.isFinite(value) || value<=0)throw "Invalid linear-mount steel dimensions";
		drilling=InsertHoles.records(v.token("holes"));
		lineage=InsertMountHoleRequirements.snapshot(haxe.Json.parse(v.token("mountingRequirements")));
		InsertMountHoleRequirements.references(lineage,drilling);
		var box={x:-width/2,y:-length/2,z:0.0,width:width,depth:length,height:thickness};
		for(hole in drilling)materia.casting.InsertStages.validateServiceHole(hole,box);
		v.setToken("holes",InsertHoles.fromNeutral(drilling));
		v.setToken("mountingRequirements",haxe.Json.stringify(lineage));
		super("LINEAR-MOUNT-STEEL","C45 linear-mount "+v.token("role"),"steel C45",false,v);
		lockManufacturingMaterial("steel C45");
		addFacet(InsertHoles.facet(drilling,width,length,thickness));
		var compliance=LinearMountCompliance.decode(v.token("compliance"));
		if(compliance!=null)addFacet(compliance);
		addConnector("mount",Mount,Solids.axial(0,0,0));
		addConnector("top",Mount,Solids.axial(0,0,thickness));
	}

	public static function receiver(width:Float,length:Float,thickness:Float,
		derived:MountHoleDerivation.DerivedMountHoles):LinearMountSteel
		return new LinearMountSteel(new ComponentValues().setNumber("width",width).setNumber("length",length)
			.setNumber("thickness",thickness).setToken("holes",InsertHoles.fromNeutral(derived.holes))
			.setToken("mountingRequirements",haxe.Json.stringify(derived.requirements)));

	public function mountingRequirements():Array<InsertMountHoleRequirement>
		return InsertMountHoleRequirements.snapshot(lineage);

	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail=Preview):Part {
		// Drilling is structural at every detail; no solid blind hole in Envelope.
		return Solids.cut(Solids.named(Part.box(width,length,thickness),"steel"),
			InsertHoles.facet(drilling,width,length,thickness).drillingTools());
	}
}
