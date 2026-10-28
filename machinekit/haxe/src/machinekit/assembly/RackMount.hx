package machinekit.assembly;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;
import machinekit.component.MachineComponent;

typedef RackContactLink = {var first:String;var second:String;}

/** Required physical support. The datum is in host coordinates: +Y from rack
 * back towards teeth, +Z along travel. Contact links name actual load-path
 * members; "$rack" denotes the element being placed. */
class RackMount {
    public final host:String;
    public var datum(get,never):AssemblyFrame;
    final datumSnapshot:AssemblyFrame;
    function get_datum():AssemblyFrame return F.compose(F.identity(),datumSnapshot);
    public final attachment:String;
    public final phaseOffsetMm:Float;
    public var contacts(get,never):Array<RackContactLink>;
    final contactSnapshot:Array<RackContactLink>;
    function get_contacts():Array<RackContactLink> return [for(c in contactSnapshot){first:c.first,second:c.second}];
    public function new(host:String,datum:AssemblyFrame,attachment:String,contacts:Array<RackContactLink>,phaseOffsetMm:Float=0) {
        if(host==null || StringTools.trim(host)=="" || host=="$rack" || attachment==null || StringTools.trim(attachment)=="" || attachment=="$rack" || contacts==null || contacts.length==0)
            throw "Rack mount requires a host and physical contact load path";
        machinekit.design.DesignGeometry.validateFrame(datum);
        if(!Math.isFinite(phaseOffsetMm))throw "Rack phase offset must be finite";
        this.phaseOffsetMm=phaseOffsetMm;this.host=host;this.attachment=attachment;this.datumSnapshot=F.compose(F.identity(),datum);
        this.contactSnapshot=[for(c in contacts){first:c.first,second:c.second}];
        var reached:Map<String,Bool>=["$rack"=>true],changed=true;
        while(changed){changed=false;for(c in contacts){
            if(c.first==null || c.second==null || c.first==c.second)throw "Invalid rack support link";
            if(reached.exists(c.first) && !reached.exists(c.second)){reached.set(c.second,true);changed=true;}
            if(reached.exists(c.second) && !reached.exists(c.first)){reached.set(c.first,true);changed=true;}
        }}
        if(contacts.filter(c->c.first=="$rack" && c.second==attachment || c.second=="$rack" && c.first==attachment).length==0)throw "Rack attachment must touch the element directly";
        if(!reached.exists(host))throw "Rack contact graph does not reach its host";
        for(c in contacts)if(!reached.exists(c.first))throw "Disconnected rack support contact";
    }
    public function validate(builder:AxisBuilder,rack:MachineComponent,pose:AssemblyFrame):Void {
        PosedParts.scope(cache->{for(link in contactSnapshot){
            var a=link.first=="$rack"?rack:builder.component(link.first);
            var b=link.second=="$rack"?rack:builder.component(link.second);
            var pa=link.first=="$rack"?pose:builder.zeroPose(link.first);
            var pb=link.second=="$rack"?pose:builder.zeroPose(link.second);
            var first=cache.posed(a,pa),second=cache.posed(b,pb);
            try {
                var gap=first.distance(second);
                if(!Math.isFinite(gap) || gap<0 || gap>AssemblyIntegrity.CONTACT_TOLERANCE)throw 'Missing rack support contact ${link.first} -> ${link.second}: $gap mm';
                first.close();second.close();
            }catch(e:Dynamic){first.close();second.close();throw e;}
        }});
    }
}
