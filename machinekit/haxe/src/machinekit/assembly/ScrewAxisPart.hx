package machinekit.assembly;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Plane;
import cadkit.modeling.Location;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Saveable fabrication for a checked ScrewAxisLayout. Legacy example recipes remain separate. */
class ScrewAxisPart extends MachineComponent {
    public static inline final RISER_BOLT_OFFSET:Float=14;
    public static function riserBoltSeat():Float return 37-SocketHeadCapScrew.catalog().get("M6").counterboreDepth;
    public final kind:String;
    public final revision:String;
    public final layout:ScrewAxisLayout;
    static var recipe:Null<ComponentType>;
    public static function kinds():Array<String> return ["base","riserLeft","riserRight","nutBracket","carriage"];
    public function new(kind:String="base",revision:String="v2-tapped",?layout:ScrewAxisLayout) {
        if(layout==null)layout=new ScrewAxisLayout();
        if(kinds().indexOf(kind)<0) throw "Unknown axis fabrication part";
        if(revision!="v2-tapped")throw "Unknown fabrication revision";
        super("TWIN-RAIL-SCREW-"+kind+"-v2-tapped"+"-L"+Dimension.format(layout.screwLength)+"-R"+Dimension.format(layout.railLength)+"-S"+Dimension.format(layout.railSpacing),"Twin rail screw axis "+kind,"aluminium 6061");this.kind=kind;this.revision=revision;this.layout=layout;
        addConnector("base",Mount,F.identity());
        switch kind {
            case "base":
                addConnector("rbk",Mount,F.identity());
                addConnector("floating",Mount,F.translation(0,0,layout.floatingZ()));
                addConnector("riserLeft",Mount,F.translation(-layout.halfSpacing(),0,100));
                addConnector("riserRight",Mount,F.translation(layout.halfSpacing(),0,100));
                // Reference CAD spacer end; standard 54 mm screw end remains unchanged.
                addConnector("screwSeat",Axis,Solids.axial(0,25,94.5));
                addConnector("travel",Axis,Solids.axial(0,0,140));
            case "riserLeft","riserRight": addConnector("rail",Mount,F.translation(0,37,0));
            case "nutBracket":
                addConnector("slide",Axis,Solids.axial(0,0,0));
                addConnector("nut",Mount,Solids.axial(0,25,10));
                addConnector("plate",Mount,F.translation(0,65,0));
            case "carriage":
                addConnector("guideLeft",Mount,F.translation(-layout.halfSpacing(),0,0));
                addConnector("guideRight",Mount,F.translation(layout.halfSpacing(),0,0));
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
                body=box(layout.baseWidth(),12,layout.screwLength+42,0,-6,13);owned.push(body);
                for(z in [40.0,70,83]) for(x in [-23.0,23]) tools.push(tapped("M6",12,x,0,z));
                for(x in [-23.0,23]) tools.push(tapped("M6",12,x,0,layout.floatingZ()));
                var outer=layout.halfSpacing()+RISER_BOLT_OFFSET;
                for(x in [-outer,outer]) for(z in layout.riserBolts()) tools.push(tapped("M6",12,x,0,z));
            case "riserLeft","riserRight":
                body=box(44,37,layout.railLength,0,18.5,0);owned.push(body);
                var offset=RISER_BOLT_OFFSET;
                var x=kind=="riserLeft" ? -offset : offset;
                for(z in [10.0,layout.railLength-10]) {
                    tools.push(Part.cylinderAlongY(3.3,-0.1,37.1,x,z));
                    tools.push(Part.cylinderAlongY(SocketHeadCapScrew.catalog().get("M6").counterboreDiameter/2,riserBoltSeat(),37.1,x,z));
                }
                for(z in layout.railHoles) tools.push(tapped("M4",12,0,37,z));
            case "nutBracket":
                body=Solids.union([box(84,64,10,0,33,10),box(84,10,40,0,60,-20)]);owned.push(body);
                tools.push(Part.cylinderSpan(14.1,9.9,20.1,0,25));
                for(p in [for(a in [45,90,135,225,270,315]) {x:19*Math.cos(a*Math.PI/180),y:19*Math.sin(a*Math.PI/180)}]) tools.push(tapped("M5",10,p.x,25+p.y,20,false));
                for(x in [-30.0,30]) for(z in [-10.0,10]) tools.push(tapped("M6",10,x,65,z));
            case "carriage":
                body=box(layout.carriageWidth(),10,80,0,5,-40);owned.push(body);
                for(x in [-30.0,30]) for(z in [-10.0,10]) tools.push(Part.cylinderAlongY(3.3,-0.1,10.1,x,z));
                for(rail in [-layout.halfSpacing(),layout.halfSpacing()]) for(x in [-13.0,13]) for(z in [-13.0,13])
                {
                    tools.push(Part.cylinderAlongY(2.25,-0.1,10.1,rail+x,z));
                    tools.push(Part.cylinderAlongY(3.75,7.5,10.1,rail+x,z));
                }
            default:throw "Unknown fabrication part";
        }
        for(tool in tools) owned.push(tool);
        return Solids.cut(body,tools);
    });
    public static function recipeType():ComponentType {
        if(recipe==null) recipe=new ComponentType("machinekit.assembly.twin-rail-screw-part",[P.choice("kind",kinds(),"base"),P.choice("revision",["v2-tapped"],"v2-tapped"),P.length("screwLength",400),P.length("railLength",300),P.length("railSpacing",134),P.length("travel",200)],v -> new ScrewAxisPart(v.token("kind"),v.token("revision"),new ScrewAxisLayout(v.number("travel"),v.number("screwLength"),v.number("railLength"),v.number("railSpacing"))),true);
        return recipe;
    }
    override public function componentType():Null<ComponentType> return recipeType();
    override public function values():ComponentValues return new ComponentValues().setToken("kind",kind).setToken("revision",revision).setToken("material",materialSpec()).setNumber("screwLength",layout.screwLength).setNumber("railLength",layout.railLength).setNumber("railSpacing",layout.railSpacing).setNumber("travel",layout.travel);
}
