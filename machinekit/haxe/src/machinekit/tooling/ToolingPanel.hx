package machinekit.tooling;

import cadkit.modeling.Part;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.panel.PanelPlan;
import materia.tooling.panel.PanelPlan.PanelOperationKind;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelJoints;
import materia.casting.CastingProfileGeometry;

/** Rebuildable exact panel; material is an explicit CAD/BOM material selection. */
class ToolingPanel extends MachineComponent {
    static var recipe:Null<ComponentType>;
    final record:PanelPlan;
    static function parameters():Array<machinekit.component.ComponentParameter> return [
        P.text("plan",PanelCodec.encode(PanelJoints.blank("panel",100,100,PanelJoints.stock())))];
    public static function recipeType():ComponentType {
        if(recipe==null)recipe=new ComponentType("machinekit.tooling.panel",parameters(),v->new ToolingPanel(v));
        return recipe;
    }
    public static function fromPlan(plan:PanelPlan,material:String="birch plywood"):ToolingPanel
        return new ToolingPanel(new ComponentValues().setToken("plan",PanelCodec.encode(plan)).setToken("material",material));
    public function new(?input:ComponentValues) {
        var v=P.resolve(parameters(),input),p=PanelCodec.decode(v.token("plan"));
        v.setToken("plan",PanelCodec.encode(p));
        super("PANEL-"+p.id+"-"+PanelCodec.fingerprint(p).substr(0,12),"Tooling panel "+p.id,
            v.get("material")==null?"birch plywood":v.token("material"),false,v);
        record=p;
        addConnector("mount",Mount,F.identity());
        addConnector("top",Mount,Solids.axial(0,0,p.stock.measuredThickness));
        for(op in p.operations)if(op.kind!=OutsideProfile) {
            var b=CastingProfileGeometry.bounds(op.contour),positive=op.side==MachiningSide.PositiveZ;
            addConnector("cut-"+op.id,Mount,F.alongY(b.x+b.width/2,b.y+b.height/2,positive?p.stock.measuredThickness:0,0,0,positive?1:-1));
        }
    }
    public function plan():PanelPlan return PanelCodec.snapshot(record);
    override public function componentType():Null<ComponentType> return recipeType();
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part {
        var t=record.stock.measuredThickness,outer=record.operations.filter(o->o.kind==OutsideProfile)[0];
        var body=Solids.named(PanelCad.prism(outer,0,t),"panel");
        return Solids.building([body],parts->{
            for(op in record.operations)if(op.kind!=OutsideProfile) {
                var z0=op.through?-1e-5:(op.side==MachiningSide.PositiveZ?t-op.depth:-1e-5);
                var z1=op.through?t+1e-5:(op.side==MachiningSide.PositiveZ?t+1e-5:op.depth);
                parts.push(Solids.named(PanelCad.prism(op,z0,z1),op.id));
            }
            return Solids.cut(parts[0],parts.slice(1));
        });
    }
}
