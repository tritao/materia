package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingRecipe;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelPlan;
import materia.tooling.panel.PanelAssemblyPlan.PanelPlacement;
import materia.tooling.casting.CoreSupport;
import materia.tooling.casting.CoreSupport.CoreSupportOptions;
import materia.tooling.casting.CoreSupport.CoreSupportMode;
import materia.tooling.casting.CoreSupport.CoreSupportKind;
import materia.tooling.casting.MoldPlan.CorePlan;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.casting.MoldPlan.PenetrationPlan;

/** Two explicit support templates; neither establishes adhesive or structural qualification. */
class CoreSupports {
    public static function defaults(r:ToolingRecipe):CoreSupportOptions return {mode:Auto,embedDepth:r.panel.measuredThickness/3,fitClearance:r.jointClearance,adhesiveThickness:0.2};
    public static function options(raw:Dynamic):CoreSupportOptions {
        if(raw==null || Reflect.fields(raw).length!=4)throw "Invalid core support options";
        for(k in ["mode","embedDepth","fitClearance","adhesiveThickness"])if(!Reflect.hasField(raw,k))throw "Missing core support option "+k;
        var token:Dynamic=Reflect.field(raw,"mode");
        if(!Std.isOfType(token,String))throw "Invalid core support mode";
        var text:String=cast token;
        var mode:CoreSupportMode=switch text {case "auto":Auto;case "bridge":Bridge;case "bed-socket":BedSocket;default:throw "Invalid core support mode";};
        function number(k:String,zero:Bool):Float {
            var v:Dynamic=Reflect.field(raw,k);
            if((!Std.isOfType(v,Float) && !Std.isOfType(v,Int)) || !Math.isFinite(v) || (zero?v<0:v<=0))throw "Invalid core support dimension "+k;
            return v;
        }
        return {mode:mode,embedDepth:number("embedDepth",false),fitClearance:number("fitClearance",true),adhesiveThickness:number("adhesiveThickness",false)};
    }
    public static function cutting(b:MoldBox):{block:MoldBox,stockToMold:materia.assembly.AssemblyRecord.AssemblyFrame,wireDirection:{x:Float,y:Float,z:Float}}
        return {block:{x:0,y:0,z:0,width:b.width,depth:b.depth,height:b.height},stockToMold:F.translation(b.x,b.y,b.z),wireDirection:{x:0,y:1,z:0}};
    public static function kind(o:CoreSupportOptions,cb:MoldBox,b:MoldBox):CoreSupportKind {
        var bottom=Math.abs(cb.z-b.z)<1e-7,top=Math.abs(cb.z+cb.height-b.z-b.height)<1e-7;
        if(o.mode==BedSocket || (o.mode==Auto && bottom)) {
            if(!bottom)throw "Bed socket requires a cavity opening onto the bed";return BedSocket;
        }
        if(!top)throw "Bridge requires a cavity opening onto the fill face";return Bridge;
    }
    public static function build(c:CorePlan,b:MoldBox,r:ToolingRecipe,o:CoreSupportOptions,bed:PanelPlacement,
            panels:Array<PanelPlacement>,penetrations:Array<PenetrationPlan>):Void {
        var s=c.support,t=r.panel.measuredThickness,l=r.minimumLigament,tool=r.routerDiameter,g=o.adhesiveThickness;
        if(s.kind==BedSocket) {
            if(o.embedDepth>t-l || o.embedDepth<=2*g)throw "Core socket leaves insufficient bed stock or glue space";
            var inset=tool+l,foot:MoldBox={x:c.foam.x+inset,y:c.foam.y+inset,z:b.z-o.embedDepth+g,
                width:c.foam.width-2*inset,depth:c.foam.depth-2*inset,height:o.embedDepth-2*g};
            if(foot.width<=tool || foot.depth<=tool)throw "Core too small for locating foot";
            s.foot={foam:foot,cutting:cutting(foot)};s.embedDepth=o.embedDepth;s.bondAreaMm2=foot.width*foot.depth;
            s.socketOperationId=c.id+"-socket";
            var op=PanelJoints.operation(s.socketOperationId,Pocket,PanelContours.relievedRectangle(foot.x+foot.width/2-bed.pose.x,foot.y+foot.depth/2-bed.pose.y,
                foot.width+o.fitClearance,foot.depth+o.fitClearance,tool/2),o.embedDepth,tool,MachiningSide.PositiveZ,false,tool);
            var bounds=materia.tooling.panel.PanelGeometry.bounds(op.contour),cb=c.cavity;
            if(bounds.x+bed.pose.x<cb.x+l || bounds.y+bed.pose.y<cb.y+l || bounds.x+bounds.width+bed.pose.x>cb.x+cb.width-l || bounds.y+bounds.height+bed.pose.y>cb.y+cb.depth-l)throw "Core socket escapes sealing footprint";
            bed.panel.operations.push(op);bed.panel.contactOpenings.push(op.id);
            penetrations.push({panelId:bed.panel.id,operationId:op.id,kind:CoreSealed,insertId:null,coreId:c.id,sealMargin:l,sealMaterial:r.releaseSpec});
            return;
        }
        // The carrier is bonded to the foam; the bridge is bonded to the carrier.
        // Removable top pads register over the entire wall top in blind sockets.
        // Their locating pins and separate bolts sit outside the casting boundary.
        var edge=2*l+tool,padDepth=3*t,padWidth=b.width+2*t+2*edge,px=b.x-t-edge;
        var cx=c.foam.x+c.foam.width/2,bw=Math.max(c.foam.width,80.0),bridgeX=cx-bw/2;
        var southY=b.y-padDepth,northY=b.y+b.depth-edge,bridgeY=southY,bridgeDepth=b.depth+2*padDepth;
        var carrier=PanelJoints.blank(c.id+"-carrier",c.foam.width,c.foam.depth,r.panel,tool,l);
        var bridge=PanelJoints.blank(c.id+"-bridge",bw,bridgeDepth,r.panel,tool,l);
        function placed(p:PanelPlan,x:Float,y:Float,z:Float):Void {
            panels.push({panel:PanelCodec.snapshot(p),pose:F.translation(x,y,z),withdrawal:{x:0,y:0,z:1}});s.panelIds.push(p.id);
        }
        for(north in [false,true]) {
            var id=c.id+(north?"-north-pad":"-south-pad"),py=north?northY:southY;
            var pad=PanelJoints.blank(id,padWidth,padDepth+edge,r.panel,tool,l),slotY=north?edge+t/2:padDepth-t/2;
            pad.labelY=north?padDepth+edge-8:8;
            pad.operations.push(PanelJoints.operation("wall-top-socket",Pocket,
                PanelContours.relievedRectangle(padWidth/2,slotY,b.width+2*t+o.fitClearance,t+o.fitClearance,tool/2),t/2,tool,MachiningSide.NegativeZ,false,tool));
            var y=north?b.y+b.depth+2*t:b.y-2*t,pinx=cx-bw/4,boltx=cx+bw/4;
            pad.operations.push(PanelJoints.operation("locating-pin",Drill,PanelContours.circle(pinx-px,y-py,3+o.fitClearance/2),t,6+o.fitClearance));
            pad.operations.push(PanelJoints.operation("clamp-bolt",Drill,PanelContours.circle(boltx-px,y-py,3.5),t,7));
            var contour=north?PanelContours.placed(PanelContours.obround(0,0,8+o.fitClearance,6+o.fitClearance),Math.PI/2,pinx-bridgeX,y-bridgeY):PanelContours.circle(pinx-bridgeX,y-bridgeY,3+o.fitClearance/2);
            bridge.operations.push(PanelJoints.operation(north?"secondary-slot":"primary-pin",north?InsideProfile:Drill,contour,t,north?tool:6+o.fitClearance));
            bridge.operations.push(PanelJoints.operation(north?"north-bolt":"south-bolt",Drill,PanelContours.circle(boltx-bridgeX,y-bridgeY,3.5),t,7));
            var prefix=c.id+(north?"-north":"-south");
            s.hardware.push({id:prefix+"-pin",kind:LocatingPin,pose:F.translation(pinx,y,b.z+b.height),diameter:6,length:2*t});
            s.hardware.push({id:prefix+"-bolt",kind:ClampBolt,pose:F.translation(boltx,y,b.z+b.height+2*t),diameter:6,length:2*t});
            placed(pad,px,py,b.z+b.height);
        }
        placed(carrier,c.foam.x,c.foam.y,b.z+b.height);
        placed(bridge,bridgeX,bridgeY,b.z+b.height+t);
        s.bondedPanelIds=[carrier.id,bridge.id];
        s.bondAreaMm2=c.foam.width*c.foam.depth;
    }
    public static function validateFoot(c:CorePlan,r:ToolingRecipe,o:CoreSupportOptions,bed:MoldBox):Void {
        var s=c.support,foot=s.foot;
        if(s.kind!=CoreSupportKind.BedSocket)return;
        if(foot==null)throw "Missing locating foot";
        var f=foot.foam,b=foot.cutting.block,p=foot.cutting.stockToMold,v=foot.cutting.wireDirection;
        for(n in [f.x,f.y,f.z,f.width,f.depth,f.height,b.x,b.y,b.z,b.width,b.depth,b.height,p.x,p.y,p.z,p.qx,p.qy,p.qz,p.qw])if(!Math.isFinite(n))throw "Invalid foot geometry";
        var inset=r.routerDiameter+r.minimumLigament;
        if(f.width<=0 || f.depth<=0 || f.height<=0 || f.width>r.core.width || f.depth>r.core.depth || f.height>r.core.height ||
            Math.abs(f.x-c.foam.x-inset)+Math.abs(f.y-c.foam.y-inset)+Math.abs(f.width-c.foam.width+2*inset)+Math.abs(f.depth-c.foam.depth+2*inset)+
            Math.abs(f.z-bed.z+o.embedDepth-o.adhesiveThickness)+Math.abs(f.height-o.embedDepth+2*o.adhesiveThickness)>1e-7)throw "Locating foot does not match support";
        if(b.x!=0 || b.y!=0 || b.z!=0 || Math.abs(b.width-f.width)+Math.abs(b.depth-f.depth)+Math.abs(b.height-f.height)>1e-7 ||
            Math.abs(p.x-f.x)+Math.abs(p.y-f.y)+Math.abs(p.z-f.z)+Math.abs(p.qx)+Math.abs(p.qy)+Math.abs(p.qz)>1e-7 || Math.abs(Math.abs(p.qw)-1)>1e-8 || v.x!=0 || v.y!=1 || v.z!=0)throw "Invalid foot cutting definition";
    }

}
