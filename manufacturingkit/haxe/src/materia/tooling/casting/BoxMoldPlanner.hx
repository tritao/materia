package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingEnums.ContactFace;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelJoints.PanelJointKind;
import materia.tooling.panel.PanelJoints.PanelClosure;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelAssemblyPlan.PanelPlacement;
import materia.tooling.panel.PanelAssemblyPlan.PanelAssemblyRecord;
import materia.tooling.casting.CastingToolingInput.CastingFeatureKind;
import materia.tooling.casting.CastingSetup.CoreStrategy;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.casting.MoldPlan.PenetrationPlan;
import materia.tooling.casting.CoreSupport.CoreSupportOptions;
import materia.tooling.casting.CoreSupport.CoreSupportKind;

/** Explicit orthogonal box template. Unsupported cuts/orientations fail rather than disappear.
 * Four independently liftable walls sit in blind bed sockets outside the casting.
 * The continuous inside lands own casting size; clearance is on the outside of sockets.
 */
class BoxMoldPlanner {
    public static function box(width:Float,depth:Float,height:Float,pose:AssemblyFrame):MoldBox {
        var points=[];
        for(x in [-width/2,width/2])for(y in [-depth/2,depth/2])for(z in [0.0,height])points.push(F.transformPoint(pose,x,y,z));
        var lo={x:1e30,y:1e30,z:1e30},hi={x:-1e30,y:-1e30,z:-1e30};
        for(p in points){lo.x=Math.min(lo.x,p.x);lo.y=Math.min(lo.y,p.y);lo.z=Math.min(lo.z,p.z);hi.x=Math.max(hi.x,p.x);hi.y=Math.max(hi.y,p.y);hi.z=Math.max(hi.z,p.z);}
        for(p in points)if(Math.min(Math.abs(p.x-lo.x),Math.abs(p.x-hi.x))>1e-7 || Math.min(Math.abs(p.y-lo.y),Math.abs(p.y-hi.y))>1e-7 || Math.min(Math.abs(p.z-lo.z),Math.abs(p.z-hi.z))>1e-7)throw "Box mold needs axis-aligned setup geometry";
        return {x:lo.x,y:lo.y,z:lo.z,width:hi.x-lo.x,depth:hi.y-lo.y,height:hi.z-lo.z};
    }
    public static function volume(b:MoldBox):Float return b.width*b.depth*b.height;
    public static function clipped(a:MoldBox,b:MoldBox):MoldBox {
        var x=Math.max(a.x,b.x),y=Math.max(a.y,b.y),z=Math.max(a.z,b.z);
        var out={x:x,y:y,z:z,width:Math.min(a.x+a.width,b.x+b.width)-x,depth:Math.min(a.y+a.depth,b.y+b.depth)-y,height:Math.min(a.z+a.height,b.z+b.height)-z};
        if(out.width<=0 || out.depth<=0 || out.height<=0)throw "Feature misses casting envelope";
        return out;
    }
    public static function plan(input:ToolingDocument,?supportOptions:CoreSupportOptions):MoldPlan {
        var d=ToolingCodec.snapshot(input),r=d.recipe,e=d.input.envelope;
        var options=CoreSupports.options(supportOptions==null?CoreSupports.defaults(r):supportOptions);
        if(e.length!=1 || e[0].profile.length!=4)throw "Box template needs one rectangular envelope";
        var p=e[0].profile,x=p[0].x,y=p[0].y,w=0.0,h=0.0;
        for(v in p){x=Math.min(x,v.x);y=Math.min(y,v.y);}
        for(v in p){w=Math.max(w,v.x-x);h=Math.max(h,v.y-y);}
        for(v in p)if((Math.abs(v.x-x)>1e-8 && Math.abs(v.x-x-w)>1e-8) || (Math.abs(v.y-y)>1e-8 && Math.abs(v.y-y-h)>1e-8))throw "Envelope is not a rectangle";
        var ep=F.compose(d.setup.sourceToMold,e[0].pose),center=F.compose(ep,F.translation(x+w/2,y+h/2,0));
        var b=box(w,h,e[0].height,center),t=r.panel.measuredThickness,c=r.jointClearance,seat=t/2;
        if(seat<r.minimumLigament)throw "Bed sockets leave insufficient stock";
        var margin=2*t+2*r.minimumLigament+r.routerDiameter,ox=b.x-margin,oy=b.y-margin;
        var bed=PanelJoints.blank("bed",b.width+2*margin,b.depth+2*margin,r.panel,r.routerDiameter,r.minimumLigament);
        bed.contactFace=ContactFace.PositiveZ;bed.labelSide=MachiningSide.NegativeZ;bed.contactOpenings=[];
        var assembly:PanelAssemblyRecord={id:"box-panels",members:[{panel:bed,pose:F.translation(ox,oy,b.z-t),withdrawal:{x:0,y:0,z:-1}}],joints:[]};
        var penetrations:Array<PenetrationPlan> = [];
        function wall(id:String,width:Float,pose:AssemblyFrame,sx:Float,sy:Float,sw:Float,sh:Float):Void {
            var panel=PanelJoints.blank(id,width,b.height+(id=="south" || id=="north"?2*seat:seat),r.panel,r.routerDiameter,r.minimumLigament);
            panel.contactFace=ContactFace.NegativeZ;panel.labelSide=MachiningSide.PositiveZ;
            panel.contactOpenings=["bottom-land"];
            panel.operations.push(PanelJoints.operation("bottom-land",Rebate,PanelContours.rectangle(0,0,width,seat),t/2,r.routerDiameter,MachiningSide.NegativeZ,false));
            penetrations.push({panelId:id,operationId:"bottom-land",kind:OutsideCasting,insertId:null,coreId:null,sealMargin:0,sealMaterial:r.releaseSpec});
            var op=PanelJoints.operation(id+"-socket",Pocket,PanelContours.relievedRectangle(sx-ox+sw/2,sy-oy+sh/2,sw,sh,r.routerDiameter/2),seat,r.routerDiameter,MachiningSide.PositiveZ,false,r.routerDiameter);
            bed.operations.push(op);bed.contactOpenings.push(op.id);
            penetrations.push({panelId:bed.id,operationId:op.id,kind:OutsideCasting,insertId:null,coreId:null,sealMargin:0,sealMaterial:r.releaseSpec});
            assembly.members.push({panel:PanelCodec.snapshot(panel),pose:pose,withdrawal:{x:0,y:0,z:1}});
            assembly.joints.push({id:id+"-bed",kind:PanelJointKind.TabSlot,fixedId:bed.id,movingId:id,
                fixedFrame:F.compose(F.inverse(assembly.members[0].pose),pose),movingFrame:F.identity(),clearance:c,closure:PanelClosure.ExternalClamps});
        }
        wall("south",b.width+2*t,F.fromRotationMatrix(b.x-t,b.y,b.z-seat,[1,0,0,0,0,-1,0,1,0]),b.x-t-c,b.y-t-c,b.width+2*t+2*c,t/2+c);
        wall("north",b.width+2*t,F.fromRotationMatrix(b.x+b.width+t,b.y+b.depth,b.z-seat,[-1,0,0,0,0,1,0,1,0]),b.x-t-c,b.y+b.depth+t/2,b.width+2*t+2*c,t/2+c);
        // Side sockets remain separate from long sockets; cutter reliefs stay outside contact lands.
        wall("east",b.depth,F.fromRotationMatrix(b.x+b.width,b.y,b.z-seat,[0,0,1,1,0,0,0,1,0]),b.x+b.width+t/2,b.y,t/2+c,b.depth);
        wall("west",b.depth,F.fromRotationMatrix(b.x,b.y+b.depth,b.z-seat,[0,0,-1,-1,0,0,0,1,0]),b.x-t-c,b.y,t/2+c,b.depth);
        var supportPanels:Array<PanelPlacement> = [];
        var cores:Array<materia.tooling.casting.MoldPlan.CorePlan> = [];
        for(f in d.input.features)if(f.kind==CastingFeatureKind.Void) {
            var strategy=d.setup.features.filter(s->s.featureId==f.id)[0];
            if(strategy.strategy!=CoreStrategy.SacrificialEPS)throw "Box template supports EPS only";
            var cut=box(f.width,f.depth,f.height,F.compose(d.setup.sourceToMold,f.pose)),cb=clipped(cut,b),l=r.linerThickness;
            if(cb.x<=b.x || cb.y<=b.y || cb.x+cb.width>=b.x+b.width || cb.y+cb.depth>=b.y+b.depth)throw "EPS core needs an internal pocket accessible through fill face";
            var kind=CoreSupports.kind(options,cb,b),bridge=kind==CoreSupportKind.Bridge;
            var foam:MoldBox={x:cb.x+l,y:cb.y+l,z:cb.z+(bridge?l:0),width:cb.width-2*l,depth:cb.depth-2*l,
                height:cb.height-l-(bridge?options.adhesiveThickness:0)};
            if(foam.width<=0 || foam.depth<=0 || foam.height<=0 || foam.width>r.core.width || foam.depth>r.core.depth || foam.height>r.core.height)throw "EPS core exceeds stock";
            cores.push({id:"core-"+f.id,featureId:f.id,cavity:cb,foam:foam,materialSpec:r.core.materialSpec,coveringThickness:l,removal:bridge?BreakoutThroughFill:BreakoutThroughBed,
                support:{kind:kind,adhesiveSpec:r.adhesiveSpec,adhesiveThickness:options.adhesiveThickness,fitClearance:options.fitClearance,embedDepth:0,bondAreaMm2:0,socketOperationId:null,foot:null,panelIds:[],bondedPanelIds:[],hardware:[],padClosure:bridge?PanelClosure.ExternalClamps:null},
                cutting:{block:{x:0,y:0,z:0,width:foam.width,depth:foam.depth,height:foam.height},stockToMold:F.translation(foam.x,foam.y,foam.z),wireDirection:{x:0,y:1,z:0}},restraintEvidence:[]});
        }
        if(cores.length!=1 || d.input.inserts.length!=1)throw "Single-pocket template needs one core and one insert";
        CoreSupports.build(cores[0],b,r,options,assembly.members[0],supportPanels,penetrations);
        var insert=d.input.inserts[0],ip=F.compose(d.setup.sourceToMold,insert.pose),ib=box(insert.width,insert.depth,insert.height,ip),face=F.compose(ip,F.translation(0,0,insert.height)),normal=F.transformVector(face,0,0,1);
        if(Math.abs(face.z-b.z)>1e-7 || normal.z> -1+1e-8 || Math.abs(volume(clipped(ib,b))-volume(ib))>1e-5 || Math.abs(insert.boreDiameter-5)>1e-8)throw "Insert mounting face must seat on bed with fixing bore";
        // This template explicitly selects nominal M6 hardware and a 5 mm tap-drill envelope.
        // Tapping/engagement evidence is still required; a through bore is not a qualified thread.
        var fixing=7.0,operation=PanelJoints.operation("insert-fixing",Drill,PanelContours.circle(face.x-ox,face.y-oy,fixing/2),t,fixing);
        bed.operations.push(operation);bed.contactOpenings.push(operation.id);
        penetrations.push({panelId:bed.id,operationId:operation.id,kind:InsertSealed,insertId:insert.id,coreId:null,sealMargin:3,sealMaterial:r.releaseSpec});
        assembly.members[0].panel=PanelCodec.snapshot(bed);
        var result:MoldPlan={id:"single-pocket-box",document:d,documentFingerprint:ToolingCodec.documentFingerprint(d),cavity:b,panels:assembly,supportOptions:options,supportPanels:supportPanels,cores:cores,
            fixtures:[{insertId:insert.id,body:ib,mountingFace:face,panelId:bed.id,operationId:operation.id,fixingDiameter:fixing-1,threadSpec:"M"+Std.string(fixing-1),threadProtection:FixingScrewAndResinSidePlug,finishingAllowance:null,datumEvidence:[]}],penetrations:penetrations,
            pending:["Core support geometry provided; adhesive, pad attachment and uplift capacity not qualified","Independent insert datum fixture accuracy and thread engagement not qualified","Insert anchorage and finishing allowance require design evidence","Seams, release, resin/EPS compatibility and cure temperature require evidence","Compaction, panel/closure loads and handling require qualification"]};
        validate(result);return result;
    }
    public static function validate(plan:MoldPlan):Void {
        if(ToolingCodec.documentFingerprint(plan.document)!=plan.documentFingerprint)throw "Stale mold document";
        materia.tooling.panel.PanelAssemblyPlan.validate(plan.panels);
        if(plan.cores.length!=1 || plan.fixtures.length!=1 || plan.panels.members.length!=5)throw "Incomplete single-pocket mold";
        CoreSupports.options(plan.supportOptions);
        if(plan.supportPanels.length>0)materia.tooling.panel.PanelAssemblyPlan.validate({id:"core-support",members:plan.supportPanels,joints:[]});
        for(c in plan.cores) {
            var b=c.cutting.block,f=c.foam,pose=c.cutting.stockToMold,v=c.cutting.wireDirection;
            for(size in [b.width,b.depth,b.height,f.width,f.depth,f.height])if(!Math.isFinite(size) || size<=0)throw "Invalid core block";
            if(b.x!=0 || b.y!=0 || b.z!=0 || Math.abs(b.width-f.width)+Math.abs(b.depth-f.depth)+Math.abs(b.height-f.height)>1e-7 ||
                Math.abs(pose.x-f.x)+Math.abs(pose.y-f.y)+Math.abs(pose.z-f.z)>1e-7 || Math.abs(pose.qx)+Math.abs(pose.qy)+Math.abs(pose.qz)>1e-8 || Math.abs(Math.abs(pose.qw)-1)>1e-8 ||
                v.x!=0 || v.y!=1 || v.z!=0)throw "Invalid ruled core cutting definition";
            CoreSupports.validateFoot(c,plan.document.recipe,plan.supportOptions,plan.cavity);
            var support=c.support;
            if(support.kind!=CoreSupportKind.Bridge && support.kind!=CoreSupportKind.BedSocket)throw "Invalid core support kind";
            if(CoreSupports.kind(plan.supportOptions,c.cavity,plan.cavity)!=support.kind)throw "Core support disagrees with opening";
            if(!Math.isFinite(support.adhesiveThickness) || support.adhesiveThickness<=0 || !Math.isFinite(support.bondAreaMm2) || support.bondAreaMm2<=0)throw "Invalid core bond dimensions";
            for(id in support.panelIds)if(plan.supportPanels.filter(m->m.panel.id==id).length!=1)throw "Missing support panel";
            for(id in support.bondedPanelIds)if(support.panelIds.indexOf(id)<0)throw "Unknown bonded support panel";
            if(support.kind==CoreSupportKind.Bridge && (support.bondedPanelIds.length!=2 || support.panelIds.length!=4 || support.foot!=null || support.socketOperationId!=null))throw "Incomplete bridge";
            if(support.kind==CoreSupportKind.BedSocket && (support.bondedPanelIds.length!=0 || support.panelIds.length!=0 || support.foot==null || support.socketOperationId==null))throw "Incomplete bed socket";
            var l=c.coveringThickness,cb=c.cavity,bridge=support.kind==CoreSupportKind.Bridge;
            if(!Math.isFinite(l) || l<0 || Math.abs(f.x-cb.x-l)+Math.abs(f.y-cb.y-l)+Math.abs(f.z-cb.z-(bridge?l:0))+Math.abs(f.width+2*l-cb.width)+Math.abs(f.depth+2*l-cb.depth)+Math.abs(f.height+l+(bridge?support.adhesiveThickness:0)-cb.height)>1e-7)throw "Core covering does not reproduce cavity";
        }
        var seen=new Map<String,Bool>();
        for(p in plan.penetrations) {
            var key=p.panelId+"/"+p.operationId;
            if(seen.exists(key))throw "Duplicate penetration";seen.set(key,true);
            var members=plan.panels.members.filter(m->m.panel.id==p.panelId);
            if(members.length!=1 || members[0].panel.contactOpenings.indexOf(p.operationId)<0)throw "Unknown penetration";
            if(p.kind!=OutsideCasting && p.kind!=InsertSealed && p.kind!=CoreSealed)throw "Unsupported penetration class";
            if(p.kind==InsertSealed && (p.insertId==null || plan.fixtures.filter(f->f.insertId==p.insertId).length!=1))throw "Unknown sealing insert";
            if(p.kind==CoreSealed && (p.coreId==null || plan.cores.filter(c->c.id==p.coreId && c.support.socketOperationId==p.operationId).length!=1))throw "Unknown sealing core";
            if(!Math.isFinite(p.sealMargin) || p.sealMargin<0 || StringTools.trim(p.sealMaterial).length==0)throw "Invalid penetration seal";
        }
        for(m in plan.panels.members)for(id in m.panel.contactOpenings)if(!seen.exists(m.panel.id+"/"+id))throw "Unclassified contact opening "+m.panel.id+"/"+id;
    }
}
