package machinekit.robotics;

import machinekit.component.MachineComponent;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames;

/** Purchased complete adapter: connector bodies and leads are not separate BOM purchases.
    Photo-based layout envelopes, NOT supplier dimensional CAD or verified electrical mating.
    Standard is rigid; Expert includes illustrative straightened leads (not supplied lengths).
    No electrical bridges are asserted until supplier wiring/backfeed behavior is documented. */
class FairinoDualToolAdapter extends MachineComponent {
    public final variant:String;
    public final manufacturerPartNumber:String;
    public final supplierPartNumber:String;
    public final sourceUrl:String;
    public final externalPower:Bool;
    public final geometryVerified:Bool = false;
    public final pinoutVerified:Bool = false;
    public final currentRatingA:Null<Float> = null;
    public final geometryStatus:String = "Photo-based estimated layout; dimensions, mass, keying and lead lengths unverified";

    public function new(variant:String = "Standard") {
        if(variant!="Standard" && variant!="Expert") throw "Unknown Fairino adapter variant";
        var expert=variant=="Expert";
        super(expert ? "ICACX1500GSCEXP" : "ICACX1500GSCSTA",
            'Fairino 2-to-1 end-effector adapter $variant (purchased complete; provisional CAD)',"rubber");
        this.variant=variant;externalPower=expert;manufacturerPartNumber=designation;
        supplierPartNumber=expert ? "FA2-860952" : "FA2-606082";
        sourceUrl="https://www.welectron.com/Fairino-End-Effector-2-to-1-Adapter-"+variant;
        // Frame origin is estimated splitter centre, NOT a mounting hole or vendor datum.
        addConnector("layout",Mount,AssemblyFrames.identity());
        addConnector("robot",Mount,AssemblyFrames.alongY(0,expert ? -110 : -24,expert ? 34 : 10,0,expert ? 0 : -1,expert ? 1 : 0));
        for(i in 1...3) addConnector('tool$i',Mount,AssemblyFrames.alongY(i==1 ? -12 : 12,expert ? 140 : 26,10,0,1,0));
        if(expert) addConnector("externalPower",Mount,AssemblyFrames.alongY(24,180,10,0,1,0));
        // Unqualified keys deliberately cannot silently mate with an OEM-qualified port.
        addPort({name:"robotSignal",kind:Signal,role:Consumer,iface:Plug("fairino-adapter-unverified-robot",5),required:true,connector:"robot"});
        addPort({name:"powerIn",kind:ElectricalPower,role:Consumer,iface:Unspecified,required:true,
            connector:expert ? "externalPower" : "robot"});
        for(i in 1...3) {
            addPort({name:'signal$i',kind:Signal,role:Supply,iface:Unspecified,required:false,connector:'tool$i'});
            addPort({name:'power$i',kind:ElectricalPower,role:Supply,iface:Unspecified,required:false,connector:'tool$i'});
        }
    }
    static var recipe:Null<ComponentType>;
    public static function recipeType():ComponentType {
        if(recipe==null) recipe=new ComponentType("machinekit.robotics.fairino-dual-tool-adapter",
            [ComponentRecipeSupport.choice("variant",["Standard","Expert"],"Standard")],
            v->new FairinoDualToolAdapter(v.token("variant")),true,false,null,false);
        return recipe;
    }
    override public function componentType():Null<ComponentType> return recipeType();
    override public function values():ComponentValues return new ComponentValues().setToken("variant",variant);
    override public function hasGeometry():Bool return true;
    static function tube(r:Float,x:Float,y:Float,z:Float,dx:Float,dy:Float,dz:Float,length:Float):Part
        return Part.cylinderAlong(r,new Vector(x,y,z),new Vector(dx,dy,dz),length);
    override public function geometry(detail:ComponentDetail = Preview):Part {
        // Every number below is a planning estimate; no connector pins/threads/key are invented.
        var pieces:Array<Part> = [],bores:Array<Part> = [];
        if(!externalPower) {
            pieces.push(Part.box(42,24,20));
            pieces.push(tube(7,0,-24,10,0,1,0,14));
            for(x in [-12.0,12.0]) {
                pieces.push(tube(8,x,10,10,0,1,0,16));
                bores.push(tube(5.5,x,20,10,0,1,0,7));
            }
        } else {
            pieces.push(Part.box(36,44,20));
            // Short illustrative routes display the harness topology, not its full coiled stock length.
            pieces.push(tube(3,0,-100,10,0,1,0,80));
            pieces.push(tube(6,0,-110,10,0,1,0,16));
            pieces.push(tube(7,0,-110,10,0,0,1,24));
            for(x in [-12.0,12.0]) {
                pieces.push(tube(3,x,20,10,0,1,0,105));
                pieces.push(tube(8,x,122,10,0,1,0,18));
                bores.push(tube(5.5,x,134,10,0,1,0,7));
            }
            pieces.push(tube(3,12,0,10,1,0,0,12));
            pieces.push(tube(3,24,0,10,0,1,0,170));
            for(x in [22.0,26.0]) pieces.push(tube(0.8,x,169,10,0,1,0,11));
        }
        return Solids.cut(Solids.union(pieces),bores);
    }
}
