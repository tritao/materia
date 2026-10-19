package machinekit.tooling;

import cadkit.modeling.Part;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Nominal plain locating pin envelope; no supplier fit or standard is implied. */
class ToolingPin extends MachineComponent {
    static var recipe:Null<ComponentType>;
    static function parameters():Array<machinekit.component.ComponentParameter> return [P.length("diameter",6),P.length("length",40)];
    public static function recipeType():ComponentType {
        if(recipe==null)recipe=new ComponentType("machinekit.tooling.plain-pin",parameters(),v->new ToolingPin(v),false,false,null,false);
        return recipe;
    }
    public function new(?input:ComponentValues) {
        var v=P.resolve(parameters(),input);
        if(v.number("diameter")<=0 || v.number("length")<=0)throw "Pin dimensions must be positive";
        if(v.get("material")!=null && v.token("material")!="steel")throw "Tooling pin requires steel";
        super("TOOLING-PIN-"+v.number("diameter")+"x"+v.number("length"),"Nominal plain locating pin","steel",false,v);
        addConnector("mount",Mount,F.identity());
    }
    override public function componentType():Null<ComponentType> return recipeType();
    override public function hasGeometry():Bool return true;
    override public function setMaterial(spec:String):Void {if(spec!="steel")throw "Tooling pin requires steel";super.setMaterial(spec);}
    override public function geometry(detail:ComponentDetail=Preview):Part return Part.cylinder(parameterNumber("diameter")/2,parameterNumber("length"));
}
