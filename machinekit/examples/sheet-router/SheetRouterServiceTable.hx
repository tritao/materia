import cadkit.modeling.Part;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentDetail;

/** Same plenum envelope; bottom service ports offset from the supporting crossmembers. */
class SheetRouterServiceTable extends machinekit.routing.VacuumTable {
    public static inline var portOffsetY:Float=120;
    final nominal:machinekit.routing.VacuumTable;
    static var serviceRecipe:Null<machinekit.component.ComponentType>;
    public function new(values:ComponentValues) {super(values);nominal=new machinekit.routing.VacuumTable(values);}
    override public function componentType():Null<machinekit.component.ComponentType> {
        if(serviceRecipe==null)serviceRecipe=new machinekit.component.ComponentType("sheet-router-service-table",machinekit.routing.VacuumTable.recipeType().parameters(),v->new SheetRouterServiceTable(v));
        return serviceRecipe;
    }
    override public function geometry(detail:ComponentDetail=Preview):Part {
        var original=nominal.geometry(detail),plugs:Array<Part> = [],tools:Array<Part> = [];
        for(i in 0...zoneCount()) {
            var p=zoneFrame(i);
            plugs.push(Part.cylinderSpan(n("zonePortDiameter")/2,0,n("wall"),p.x,p.y));
            tools.push(Part.cylinderSpan(n("zonePortDiameter")/2,-1,n("wall")+1,p.x,p.y+portOffsetY));
        }
        var filled=Part.fuseAll([original].concat(plugs));original.close();for(p in plugs)p.close();
        var result=filled.subtractAll(tools);filled.close();for(p in tools)p.close();return result;
    }
}
