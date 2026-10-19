package machinekit.tooling;

import cadkit.modeling.Part;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Rebuildable block in stock coordinates [0,w] x [0,d] x [0,h].
 * Visual material is explicit; supplier/process evidence remains in CorePlan.
 */
class ToolingBlock extends MachineComponent {
    static var recipe:Null<ComponentType>;
    static function parameters():Array<machinekit.component.ComponentParameter> return [P.length("width",100),P.length("depth",80),P.length("height",50)];
    public static function recipeType():ComponentType {
        if(recipe==null)recipe=new ComponentType("machinekit.tooling.block",parameters(),v->new ToolingBlock(v));return recipe;
    }
    public function new(?input:ComponentValues) {
        var v=P.resolve(parameters(),input);
        for(k in ["width","depth","height"])if(v.number(k)<=0)throw "Block dimensions must be positive";
        super("TOOLING-BLOCK","Tooling stock block",v.get("material")==null?"unspecified":v.token("material"),false,v);
        addConnector("mount",Mount,F.identity());
    }
    override public function componentType():Null<ComponentType> return recipeType();
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part return Part.box(parameterNumber("width"),parameterNumber("depth"),parameterNumber("height"),Min,Min,Min);
}
