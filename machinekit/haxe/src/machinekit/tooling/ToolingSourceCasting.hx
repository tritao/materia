package machinekit.tooling;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingToolingInput;
import materia.casting.CastingCavities;
import machinekit.structural.CastingCavityCad;

/** A detached casting source, with one strict neutral encoding and exact cavities. */
class ToolingSourceCasting extends MachineComponent {
	static var recipe:Null<ComponentType>;

	static function parameters():Array<ComponentParameter>
		return [new ComponentParameter("source",Text,Unset)];

	public static function recipeType():ComponentType {
		if (recipe == null)
			recipe = new ComponentType("machinekit.tooling.source-casting",parameters(),v -> new ToolingSourceCasting(v),false,false,null,false,
				new ComponentValues().setToken("source",ToolingCodec.encodeInput({
					id:"recipe-unit-sample",revision:1,lengthUnit:materia.tooling.ToolingEnums.ToolingLengthUnit.Millimetres,
					datumScheme:null, interfaces:[], interfaceContext:null,
					envelope:[{id:"body",height:1,pose:F.identity(),profile:[{x:-0.5,y:-0.5},{x:0.5,y:-0.5},{x:0.5,y:0.5},{x:-0.5,y:0.5}]}],
					features:[],inserts:[]
				})));
		return recipe;
	}

	public static function fromInput(input:CastingToolingInput):ToolingSourceCasting
		return new ToolingSourceCasting(new ComponentValues().setToken("source",ToolingCodec.encodeInput(input)));

	public function new(values:ComponentValues) {
		var v = P.resolve(parameters(),values);
		var input = ToolingCodec.decodeInput(v.token("source"));
		v.setToken("source",ToolingCodec.encodeInput(input));
		super("CASTING-"+ToolingCodec.inputFingerprint(input).substr(0,16),"Casting from neutral source geometry","mineral cast",false,v);
		addConnector("mount",Mount,F.identity());
	}

	public function input():CastingToolingInput
		return ToolingCodec.decodeInput(parameterToken("source"));

	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function setMaterial(spec:String):Void {
		if (spec != "mineral cast") throw "Source casting requires mineral cast";
		super.setMaterial(spec);
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var source = input();
		return Solids.building([], owned -> {
			for (prism in source.envelope) owned.push(CastingCavityCad.prism(prism));
			var body = Solids.union(owned);
			return Solids.building([body], shapes -> {
				for (cavity in CastingCavities.derive(source.envelope,source.features))
					shapes.push(CastingCavityCad.part(cavity));
				return Solids.cut(shapes[0],shapes.slice(1));
			});
		});
	}
}
