import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.motion.VallderSfu1605Nut;
import materia.assembly.AssemblyFrames as F;

/** Fabricated parts for the worked 200 mm axis. X across, Y up, Z travel. */
class RbkAxisPart extends MachineComponent {
    public final kind:String;
    static var recipe:Null<ComponentType>;
    public static function kinds():Array<String> return ["base","riserLeft","riserRight","nutBracket","carriage"];
    public function new(kind:String="base") {
        if(kinds().indexOf(kind)<0) throw "Unknown axis fabrication part";
        super("RBK-SFU1605-STUDY-"+kind,"Worked axis "+kind,"aluminium 6061");this.kind=kind;
        addConnector("base",Mount,F.identity());
        switch kind {
            case "base":
                addConnector("rbk",Mount,F.identity());
                addConnector("floating",Mount,F.translation(0,0,433.5));
                addConnector("riserLeft",Mount,F.translation(-67,0,100));
                addConnector("riserRight",Mount,F.translation(67,0,100));
                // Reference CAD spacer end; standard 54 mm screw end remains unchanged.
                addConnector("screwSeat",Axis,Solids.axial(0,25,94.5));
                addConnector("travel",Axis,Solids.axial(0,0,140));
            case "riserLeft","riserRight": addConnector("rail",Mount,F.translation(0,37,0));
            case "nutBracket":
                addConnector("slide",Axis,Solids.axial(0,0,0));
                addConnector("nut",Mount,Solids.axial(0,25,10));
                addConnector("plate",Mount,F.translation(0,65,0));
            case "carriage":
                addConnector("guideLeft",Mount,F.translation(-67,0,0));
                addConnector("guideRight",Mount,F.translation(67,0,0));
            default:
        }
    }
    static function box(w:Float,h:Float,l:Float,x:Float,y:Float,z:Float):Part {
        var raw=Part.box(w,h,l),posed=raw.translated(new Vector(x,y,z));raw.close();return posed;
    }
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part return Solids.building([],owned -> {
        var body:Part=null,tools:Array<Part> = [];
        switch kind {
            case "base":
                body=box(200,12,442,0,-6,13);owned.push(body);
                for(z in [40.0,70,83]) for(x in [-23.0,23]) tools.push(Part.cylinderAlongY(3.35,-12.1,0.1,x,z));
                for(x in [-23.0,23]) tools.push(Part.cylinderAlongY(3.15,-12.1,0.1,x,433.5));
                for(x in [-83.0,83]) for(z in [110.0,390]) tools.push(Part.cylinderAlongY(3.3,-12.1,0.1,x,z));
            case "riserLeft","riserRight":
                body=box(44,37,300,0,18.5,0);owned.push(body);
                var x=kind=="riserLeft" ? -16.0 : 16.0;
                for(z in [10.0,290]) tools.push(Part.cylinderAlongY(3.3,-0.1,37.1,x,z));
                for(z in [20.0,80,140,200,260]) tools.push(Part.cylinderAlongY(1.65,25,37.1,0,z));
            case "nutBracket":
                body=Solids.union([box(84,64,10,0,33,10),box(84,10,40,0,60,-20)]);owned.push(body);
                tools.push(Part.cylinderSpan(14.1,9.9,20.1,0,25));
                for(p in new VallderSfu1605Nut().boltPattern()) tools.push(Part.cylinderSpan(2.75,9.9,20.1,p.x,25+p.y));
                for(x in [-30.0,30]) for(z in [-10.0,10]) tools.push(Part.cylinderAlongY(3.3,54.9,65.1,x,z));
            case "carriage":
                body=box(180,10,80,0,5,-40);owned.push(body);
                for(x in [-30.0,30]) for(z in [-10.0,10]) tools.push(Part.cylinderAlongY(3.3,-0.1,10.1,x,z));
                for(rail in [-67.0,67]) for(x in [-13.0,13]) for(z in [-13.0,13])
                    tools.push(Part.cylinderAlongY(2.25,-0.1,10.1,rail+x,z));
            default:throw "Unknown fabrication part";
        }
        for(tool in tools) owned.push(tool);
        return Solids.cut(body,tools);
    });
    public static function recipeType():ComponentType {
        if(recipe==null) recipe=new ComponentType("example.rbk-axis.fabricated-part",[P.choice("kind",kinds(),"base")],v -> new RbkAxisPart(v.token("kind")),true);
        return recipe;
    }
    override public function componentType():Null<ComponentType> return recipeType();
    override public function values():ComponentValues return new ComponentValues().setToken("kind",kind).setToken("material",materialSpec());
}
