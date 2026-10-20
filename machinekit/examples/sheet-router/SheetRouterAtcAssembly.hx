import SheetRouterAtc.RouterTool;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.routing.ToolHolder;
import machinekit.routing.Collet;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyFrames as F;
import machinekit.assembly.PosedParts;
import machinekit.assembly.PosedParts.PartBox;

/** Two-way tool ownership is represented by independent holder roots, including
 * every parked holder. The spindle itself never acquires a second XYZ joint.
 */
class SheetRouterAtcAssembly {
    public final machine:SheetRouter;
    public final definition:AssemblyDefinition;
    public final roots:Map<Int,String> = new Map();
    public final interfaces:Array<{a:String,b:String}> = [];
    final tools:Array<RouterTool>;
    final specialParts:Map<String,Part> = new Map();
    final specialBounds:Map<String,PartBox> = new Map();
    public function new(machine:SheetRouter,tools:Array<RouterTool>) {
        this.machine=machine;this.tools=tools;
        interfaces.push({a:"shoeLiftCylinder",b:"shoeLiftRod"});
        interfaces.push({a:"rackCoverLiftCylinder",b:"rackCoverLiftRod"});
        for(t in tools) {
            if(t.number==1&&(t.diameterMm>12||t.nominalGaugeTipMm>machine.routerSpec.longestTool))throw "T1 exceeds the original router reference envelope";
            var root=t.number==1?"holder":'atcHolder${t.number}';roots.set(t.number,root);
            var cutter=t.number==1?"longestTool":'atcTool${t.number}';
            if(t.number!=1) {
                var pose=pocket(t.pocket),holder=new ToolHolder();
                machine.attach(root,holder,pose,"toolRack");
                var nose=F.compose(pose,F.translation(0,0,-holder.n("gaugeLength")));
                machine.attach('atcCollet${t.number}',new Collet(new ComponentValues().setNumber("toolDiameter",t.diameterMm)),nose,root);
                machine.attach(cutter,new RouterAtcCutter(t.diameterMm,t.nominalGaugeTipMm-holder.n("gaugeLength")),nose,root);
            }
            interfaces.push({a:root,b:"spindle"});
            interfaces.push({a:root,b:'fork${t.pocket}'});
            interfaces.push({a:cutter,b:"setter"});
        }
        if(!roots.exists(1))throw "The native ATC fixture requires the router's reference T1";
        definition=materia.assembly.AssemblyDefinitionCodec.decode(materia.assembly.AssemblyDefinitionCodec.encode(machine.definition()));
        var rootNames=[for(id in roots)id];
        definition.joints=[for(j in definition.joints)if(rootNames.indexOf(j.child)<0)j];
        if(definition.mates!=null)definition.mates=[for(m in definition.mates)if(rootNames.indexOf(m.second)<0)m];
        for(o in definition.occurrences)if(rootNames.indexOf(o.id)>=0)o.grounded=false;
    }
    public function pocket(number:Int):materia.assembly.AssemblyRecord.AssemblyFrame {
        var fork=machine.component('fork$number'),p=F.compose(machine.zeroPose('fork$number'),fork.connector("pocket").frame);
        return F.translation(p.x,p.y,p.z+8);
    }
    public function place(state:AssemblyState,atc:SheetRouterAtc):Void {
        state.forwardKinematics();
        var connector=state.worldConnector("spindle","gauge"),gauge=F.translation(connector.x,connector.y,connector.z);
        for(t in tools)state.setRootPose(roots.get(t.number),atc.loaded==t.number?gauge:pocket(t.pocket));
        state.forwardKinematics();
    }
    function special(id:String):Part {
        if(!specialParts.exists(id)) {
            var part=machine.component(id).geometry();specialParts.set(id,part);specialBounds.set(id,PosedParts.boxOf(part));
        }
        return specialParts.get(id);
    }
    function forbidden(state:AssemblyState,a:String,b:String):Void {
        var first=special(a),second=special(b);
        var x=PosedParts.carried(specialBounds.get(a),state.worldPose(a)),y=PosedParts.carried(specialBounds.get(b),state.worldPose(b));
        if(Math.min(x.maxX,y.maxX)-Math.max(x.minX,y.minX)<=1e-7||Math.min(x.maxY,y.maxY)-Math.max(x.minY,y.minY)<=1e-7||Math.min(x.maxZ,y.maxZ)-Math.max(x.minZ,y.minZ)<=1e-7)return;
        var placedA=PosedParts.moved(first,state.worldPose(a)),placedB=PosedParts.moved(second,state.worldPose(b));
        var common=placedA.intersect(placedB),volume=common.volume();common.close();placedA.close();placedB.close();
        if(volume>0.001)throw 'Unseated ATC contact $a/$b: $volume mm3';
    }
    /** Re-check every blanket interface under its actual seating conditions;
     * the generic gantry checker remains opaque and needs no core changes.
     */
    public function verifySpecial(state:AssemblyState,atc:SheetRouterAtc):Void {
        var spindle=state.worldConnector("spindle","gauge"),setter=state.worldConnector("setter","trigger");
        for(t in tools) {
            var root=roots.get(t.number),holder=state.worldPose(root),seat=pocket(t.pocket);
            var entry=F.compose(machine.zeroPose('fork${t.pocket}'),machine.component('fork${t.pocket}').connector("approach").frame);
            var aligned=Math.abs(spindle.x-seat.x)<1e-5&&Math.abs(spindle.y-seat.y)<1e-5&&spindle.z>=seat.z-1e-5&&spindle.z<=seat.z+55+1e-5;
            if(atc.loaded!=t.number&&!aligned)forbidden(state,root,"spindle");
            var seated=Math.abs(holder.x-seat.x)<1e-5&&Math.abs(holder.z-seat.z)<1e-5&&holder.y>=entry.y-1e-5&&holder.y<=seat.y+1e-5;
            if(!seated)forbidden(state,root,'fork${t.pocket}');
            var tip=holder.z-t.nominalGaugeTipMm;
            var probing=atc.loaded==t.number&&Math.abs(holder.x-setter.x)<1e-5&&Math.abs(holder.y-setter.y)<1e-5&&tip<=setter.z+2.5+1e-5&&tip>=setter.z-1.5-1e-5;
            if(!probing)forbidden(state,t.number==1?"longestTool":'atcTool${t.number}',"setter");
        }
    }
    public function close():Void {for(part in specialParts)part.close();specialParts.clear();specialBounds.clear();}
}

class RouterAtcCutter extends MachineComponent {
    final diameter:Float;
    final length:Float;
    public function new(diameter:Float,length:Float) {
        super('SR7-CUTTER-$diameter-$length',"Router ATC cutter envelope","steel",true);
        this.diameter=diameter;this.length=length;addConnector("tip",Axis,F.translation(0,0,-length));
    }
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part return Part.cylinderSpan(diameter/2,-length,0);
}
