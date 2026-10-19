import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Plane;
import cadkit.modeling.Location;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.motion.VallderSfu1605Nut;
import materia.assembly.AssemblyFrames as F;

/** Fabricated parts for the worked 200 mm axis. X across, Y up, Z travel. */
class RbkAxisPart extends MachineComponent {
    public static inline final RISER_BOLT_OFFSET:Float=14;
    public static function riserBoltSeat():Float return 37-SocketHeadCapScrew.catalog().get("M6").counterboreDepth;
    public final kind:String;
    public final revision:String;
    static var recipe:Null<ComponentType>;
    public static function kinds():Array<String> return ["base","riserLeft","riserRight","nutBracket","carriage"];
    public function new(kind:String="base",revision:String="v2-tapped") {
        if(kinds().indexOf(kind)<0) throw "Unknown axis fabrication part";
        if(revision!="v1-clearance" && revision!="v2-tapped")throw "Unknown fabrication revision";
        super("RBK-SFU1605-STUDY-"+kind+(revision=="v1-clearance" ? "" : "-v2-tapped"),"Worked axis "+kind,"aluminium 6061");this.kind=kind;this.revision=revision;
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
    static function tapped(size:String,depth:Float,x:Float,y:Float,z:Float,alongY:Bool=true):Part {
        var raw=SocketHeadCapScrew.metric(size,depth).tapHole(depth);
        try {
            var placed=raw.placed(new Location(new Plane(new Vector(x,y,z),Vector.X(),alongY ? Vector.Y() : Vector.Z())));
            raw.close();return Solids.named(placed,"tap-"+size+"-"+Dimension.format(x)+"-"+Dimension.format(y)+"-"+Dimension.format(z));
        } catch(e:Dynamic){raw.close();throw e;}
    }
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part return Solids.building([],owned -> {
        var body:Part=null,tools:Array<Part> = [];
        switch kind {
            case "base":
                body=box(200,12,442,0,-6,13);owned.push(body);
                for(z in [40.0,70,83]) for(x in [-23.0,23]) tools.push(revision=="v2-tapped" ? tapped("M6",12,x,0,z) : Part.cylinderAlongY(3.35,-12.1,0.1,x,z));
                for(x in [-23.0,23]) tools.push(revision=="v2-tapped" ? tapped("M6",12,x,0,433.5) : Part.cylinderAlongY(3.15,-12.1,0.1,x,433.5));
                var outer=67+(revision=="v2-tapped" ? RISER_BOLT_OFFSET : 16);
                for(x in [-outer,outer]) for(z in [110.0,390]) tools.push(revision=="v2-tapped" ? tapped("M6",12,x,0,z) : Part.cylinderAlongY(3.3,-12.1,0.1,x,z));
            case "riserLeft","riserRight":
                body=box(44,37,300,0,18.5,0);owned.push(body);
                var offset=revision=="v2-tapped" ? RISER_BOLT_OFFSET : 16.0;
                var x=kind=="riserLeft" ? -offset : offset;
                for(z in [10.0,290]) {
                    tools.push(Part.cylinderAlongY(3.3,-0.1,37.1,x,z));
                    if(revision=="v2-tapped")tools.push(Part.cylinderAlongY(SocketHeadCapScrew.catalog().get("M6").counterboreDiameter/2,riserBoltSeat(),37.1,x,z));
                }
                for(z in [20.0,80,140,200,260]) tools.push(revision=="v2-tapped" ? tapped("M4",12,0,37,z) : Part.cylinderAlongY(1.65,25,37.1,0,z));
            case "nutBracket":
                body=Solids.union([box(84,64,10,0,33,10),box(84,10,40,0,60,-20)]);owned.push(body);
                tools.push(Part.cylinderSpan(14.1,9.9,20.1,0,25));
                for(p in new VallderSfu1605Nut().boltPattern()) tools.push(revision=="v2-tapped" ? tapped("M5",10,p.x,25+p.y,20,false) : Part.cylinderSpan(2.75,9.9,20.1,p.x,25+p.y));
                for(x in [-30.0,30]) for(z in [-10.0,10]) tools.push(revision=="v2-tapped" ? tapped("M6",10,x,65,z) : Part.cylinderAlongY(3.3,54.9,65.1,x,z));
            case "carriage":
                body=box(180,10,80,0,5,-40);owned.push(body);
                for(x in [-30.0,30]) for(z in [-10.0,10]) tools.push(Part.cylinderAlongY(3.3,-0.1,10.1,x,z));
                for(rail in [-67.0,67]) for(x in [-13.0,13]) for(z in [-13.0,13])
                {
                    tools.push(Part.cylinderAlongY(2.25,-0.1,10.1,rail+x,z));
                    if(revision=="v2-tapped")tools.push(Part.cylinderAlongY(3.75,7.5,10.1,rail+x,z));
                }
            default:throw "Unknown fabrication part";
        }
        for(tool in tools) owned.push(tool);
        return Solids.cut(body,tools);
    });
    public static function recipeType():ComponentType {
        if(recipe==null) recipe=new ComponentType("example.rbk-axis.fabricated-part",[P.choice("kind",kinds(),"base"),P.choice("revision",["v1-clearance","v2-tapped"],"v1-clearance")],v -> new RbkAxisPart(v.token("kind"),v.token("revision")),true);
        return recipe;
    }
    override public function componentType():Null<ComponentType> return recipeType();
    override public function values():ComponentValues return new ComponentValues().setToken("kind",kind).setToken("revision",revision).setToken("material",materialSpec());
}
