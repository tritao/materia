package machinekit.tooling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import machinekit.structural.CastInsert;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.FlatWasher;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.casting.MoldPlan;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.casting.BoxMoldPlanner;
import materia.tooling.casting.CoreSupport.CoreSupportKind;
import materia.tooling.casting.CoreSupport.SupportHardwareKind;
import machinekit.standard.HexNut;
import machinekit.tooling.SweptRemoval.RemovalMember;
import machinekit.tooling.SweptRemoval.RemovalReport;

typedef BoxMoldReview = {var assembly:RemovalReport;var demolding:RemovalReport;var symmetricDifferenceMm3:Float;var pending:Array<String>;var rigidGroups:Array<{id:String,members:Array<String>}>;var assemblyGroups:Array<{id:String,members:Array<String>}>;}

/** CAD realization consumes the neutral plan. Loose screws/clamps are released first;
 * adhesive joints are released explicitly. Cores and supports participate in the
 * appropriate bed/fill withdrawal graph; EPS breakup remains a manual operation.
 */
class BoxMoldCad {
    static function placedBox(b:MoldBox):Part {
        var local=Part.box(b.width,b.depth,b.height,Min,Min,Min),out=local.translated(new Vector(b.x,b.y,b.z));local.close();return out;
    }
    public static function reconstructed(p:MoldPlan):Part {
        var parts:Array<Part> = [placedBox(p.cavity)];
        return Solids.building(parts,items->{
            for(c in p.cores)items.push(placedBox(c.cavity));
            for(f in p.fixtures)items.push(placedBox(f.body));
            return Solids.cut(items[0],items.slice(1));
        });
    }
    public static function source(p:MoldPlan):Part {
        var envelope:Array<Part> = [];
        return Solids.building(envelope,items->{
            for(e in p.document.input.envelope) {
                var local=Part.prism([for(v in e.profile)new Vector(v.x,v.y)],0,e.height);
                var world=PosedParts.moved(local,F.compose(p.document.setup.sourceToMold,e.pose));local.close();items.push(world);
            }
            var body=Solids.union(items);items.resize(0);items.push(body);
            for(f in p.document.input.features) {
                var local=Part.box(f.width,f.depth,f.height),world=PosedParts.moved(local,F.compose(p.document.setup.sourceToMold,f.pose));local.close();items.push(world);
            }
            return Solids.cut(items[0],items.slice(1));
        });
    }
    static function insert(p:MoldPlan):CastInsert {
        var i=p.document.input.inserts[0];
        return new CastInsert(new ComponentValues().setNumber("width",i.width).setNumber("depth",i.depth).setNumber("height",i.height)
            .setNumber("boreDiameter",i.boreDiameter).setNumber("positionTolerance",i.positionTolerance).setToken("role",i.sourceRole));
    }
    public static function assembly(p:MoldPlan,exploded:Bool=false):MachineAssembly {
        BoxMoldPlanner.validate(p);
        var b=p.cavity,a=exploded?new MachineAssembly():PanelKit.assembly(p.panels,false,"coated particleboard");
        if(exploded)for(m in p.panels.members) {
            var pose=F.compose(F.identity(),m.pose);
            switch m.panel.id {
                case "bed":pose.z-=b.height*0.6;
                case "south":pose.y-=b.depth*0.9;
                case "north":pose.y+=b.depth*0.9;
                case "east":pose.x+=b.width*0.9;
                case "west":pose.x-=b.width*0.9;
                default:throw "Unknown box template panel";
            }
            a.addComponent(m.panel.id,ToolingPanel.fromPlan(m.panel,"coated particleboard"),pose);
        }
        for(index in 0...p.supportPanels.length) {
            var m=p.supportPanels[index],pose=F.compose(F.identity(),m.pose);
            if(exploded)pose.z+=(index+1)*b.height*0.5;
            a.addComponent(m.panel.id,ToolingPanel.fromPlan(m.panel),pose);
        }
        for(c in p.cores) {
            var block=c.cutting.block,pose=F.compose(F.identity(),c.cutting.stockToMold);
            if(exploded)pose.z+=b.height*1.3;
            var foot=c.support.foot;
            if(foot!=null) {
                var fb=foot.cutting.block,fp=F.compose(F.identity(),foot.cutting.stockToMold);
                if(exploded)fp.z-=b.height;
                a.addComponent(c.id+"-foot",new ToolingBlock(new ComponentValues().setNumber("width",fb.width).setNumber("depth",fb.depth).setNumber("height",fb.height).setToken("material","EPS tooling")),fp);
            }
            if(!exploded)for(h in c.support.hardware) {
                if(h.kind==SupportHardwareKind.LocatingPin) {
                    a.addComponent(h.id,new ToolingPin(new ComponentValues().setNumber("diameter",h.diameter).setNumber("length",h.length)),h.pose);
                } else {
                    var washer=FlatWasher.metric("M6"),nut=HexNut.metric("M6");
                    var length=5*Math.ceil((h.length+2*washer.thickness+nut.height+1)/5);
                    a.addComponent(h.id,SocketHeadCapScrew.metric("M6",length),F.compose(h.pose,F.translation(0,0,washer.thickness)));
                    a.addComponent(h.id+"-top-washer",washer,h.pose);
                    a.addComponent(h.id+"-bottom-washer",FlatWasher.metric("M6"),F.compose(h.pose,F.translation(0,0,-h.length-washer.thickness)));
                    a.addComponent(h.id+"-nut",nut,F.compose(h.pose,F.translation(0,0,-h.length-washer.thickness-nut.height)));
                }
            }
            a.addComponent(c.id,new ToolingBlock(new ComponentValues().setNumber("width",block.width).setNumber("depth",block.depth).setNumber("height",block.height).setToken("material","EPS tooling")),pose);
        }
        var i=p.document.input.inserts[0],pose=F.compose(p.document.setup.sourceToMold,i.pose);
        if(exploded){pose.x+=b.width*1.4;pose.y-=b.depth*1.4;}
        a.addComponent(i.id,insert(p),pose);
        if(!exploded) {
            var fixture=p.fixtures[0],f=fixture.mountingFace,t=p.document.recipe.panel.measuredThickness;
            var nominal=fixture.fixingDiameter;
            if(nominal!=6)throw "Preview hardware currently supports M6 fixing only";
            var washer=FlatWasher.metric("M6"),length=5*Math.ceil((t+washer.thickness+8)/5);
            var boltPose=F.fromRotationMatrix(f.x,f.y,b.z-t-washer.thickness,[1,0,0,0,-1,0,0,0,-1]);
            a.addComponent("insert-fixing-screw",SocketHeadCapScrew.metric("M6",length),boltPose);
            a.addComponent("insert-fixing-washer",washer,F.translation(f.x,f.y,b.z-t-washer.thickness));
            a.addComponent("resin-side-bore-plug",new ToolingPin(new ComponentValues().setNumber("diameter",i.boreDiameter).setNumber("length",2)),pose);
            var poses=a.solvedPoses(),bedPose=poses.get("bed");
            for(row in a.components())if(p.panels.members.filter(m->m.panel.id==row.id).length==0) {
                var local=F.compose(F.inverse(bedPose),poses.get(row.id));
                a.addMemberConnector("bed",row.id+"-datum",local);
                a.addMemberConnector(row.id,"tooling-datum",F.identity());
                a.addMate(row.id+"-positioned","fixed","bed",row.id+"-datum",row.id,"tooling-datum");
            }
        }
        return a;
    }
    static function cutVolume(p:MoldPlan,panelId:String,opId:String):Part {
        var m=p.panels.members.filter(m->m.panel.id==panelId)[0],op=m.panel.operations.filter(o->o.id==opId)[0],t=m.panel.stock.measuredThickness;
        var z0=op.side==MachiningSide.PositiveZ?t-op.depth:0.0,z1=op.side==MachiningSide.PositiveZ?t:op.depth;
        var local=PanelCad.prism(op,z0,z1),world=PosedParts.moved(local,m.pose);local.close();return world;
    }
    public static function penetrations(p:MoldPlan):Void {
        BoxMoldPlanner.validate(p);
        var envelope=placedBox(p.cavity);
        try {
            for(record in p.penetrations) {
                var cut=cutVolume(p,record.panelId,record.operationId);
                try {
                    if(record.kind==OutsideCasting) {
                        var overlap=cut.intersect(envelope),v=overlap.volume();overlap.close();
                        if(v>1e-5)throw "Outside-casting cut enters cavity: "+record.operationId;
                    } else if(record.kind==CoreSealed) {
                        var core=p.cores.filter(c->c.id==record.coreId)[0],foot=core.support.foot;
                        if(foot==null || core.support.kind!=CoreSupportKind.BedSocket)throw "Missing bed locating foot";
                        var bounds=cut.shape.bounds(),lo=Vector.fromNative(bounds.get_min()),hi=Vector.fromNative(bounds.get_max()),cb=core.cavity;
                        if(record.panelId!="bed" || core.support.socketOperationId!=record.operationId ||
                            lo.x<cb.x+record.sealMargin || hi.x>cb.x+cb.width-record.sealMargin || lo.y<cb.y+record.sealMargin || hi.y>cb.y+cb.depth-record.sealMargin ||
                            Math.abs(hi.z-p.cavity.z)>1e-7 || Math.abs(lo.z-p.cavity.z+core.support.embedDepth)>1e-7)throw "Core socket escapes sealing footprint";
                    } else {
                        var fixture=p.fixtures.filter(f->f.insertId==record.insertId)[0],i=p.document.input.inserts.filter(i->i.id==record.insertId)[0];
                        var member=p.panels.members.filter(m->m.panel.id==record.panelId)[0],op=member.panel.operations.filter(o->o.id==record.operationId)[0];
                        var inverse=F.inverse(fixture.mountingFace),margin=record.sealMargin;
                        if(record.panelId!=fixture.panelId || record.operationId!=fixture.operationId || member.panel.id!="bed")throw "Insert seal references wrong fixing hole";
                        var transform=F.compose(inverse,member.pose),z=member.panel.stock.measuredThickness;
                        var ux=F.transformVector(transform,1,0,0),uy=F.transformVector(transform,0,1,0),sign=ux.x*uy.y-ux.y*uy.x;
                        if(Math.abs(Math.abs(sign)-1)>1e-7)throw "Insert sealing face not parallel to bed";
                        var transformed=[for(seg in op.contour) {
                            var s=F.transformPoint(transform,seg.start.x,seg.start.y,z),e=F.transformPoint(transform,seg.end.x,seg.end.y,z),center=seg.center;
                            var lc=center==null?null:F.transformPoint(transform,center.x,center.y,z);
                            if(Math.abs(s.z)>1e-7 || Math.abs(e.z)>1e-7)throw "Insert sealing face misses opening";
                            {kind:seg.kind,start:{x:s.x,y:s.y},end:{x:e.x,y:e.y},center:lc==null?null:{x:lc.x,y:lc.y},sweep:sign*seg.sweep};
                        }];
                        var bounds=materia.tooling.panel.PanelGeometry.bounds(transformed);
                        if(bounds.x< -i.width/2+margin || bounds.y< -i.depth/2+margin || bounds.x+bounds.width>i.width/2-margin || bounds.y+bounds.height>i.depth/2-margin)throw "Insert sealing margin exceeded";
                        if(!op.through || Math.abs(fixture.mountingFace.z-p.cavity.z)>1e-7)throw "Insert seal face misses bed";
                    }
                    cut.close();
                }catch(e:Dynamic){cut.close();throw e;}
            }
            envelope.close();
        }catch(e:Dynamic){envelope.close();throw e;}
    }
    /** Cover every intended closed face with actual panels; the single fixing seal is
     * represented only by its explicitly checked insert footprint. Fill +Z remains open. */
    static function coverage(p:MoldPlan,panels:Array<Part>):Void {
        var b=p.cavity,s=0.1;
        var slabs:Array<MoldBox> = [
            {x:b.x-s,y:b.y,z:b.z,width:s,depth:b.depth,height:b.height},
            {x:b.x+b.width,y:b.y,z:b.z,width:s,depth:b.depth,height:b.height},
            {x:b.x,y:b.y-s,z:b.z,width:b.width,depth:s,height:b.height},
            {x:b.x,y:b.y+b.depth,z:b.z,width:b.width,depth:s,height:b.height},
            {x:b.x,y:b.y,z:b.z-s,width:b.width,depth:b.depth,height:s}];
        for(k in 0...slabs.length) {
            var slab=placedBox(slabs[k]),tools=panels.copy(),seals:Array<Part> = [];
            try {
                if(k==4) {
                    var insert=p.fixtures[0].body;
                    var seal=placedBox({x:insert.x,y:insert.y,z:b.z-s,width:insert.width,depth:insert.depth,height:s});seals.push(seal);tools.push(seal);
                    for(core in p.cores)if(core.support.kind==CoreSupportKind.BedSocket) {
                        var cb=core.cavity,coreSeal=placedBox({x:cb.x,y:cb.y,z:b.z-s,width:cb.width,depth:cb.depth,height:s});seals.push(coreSeal);tools.push(coreSeal);
                    }
                }
                var missing=slab.subtractAll(tools),volume=missing.volume();missing.close();
                if(volume>1e-5)throw "Uncovered mold boundary face "+k;
                for(seal in seals)seal.close();slab.close();
            }catch(e:Dynamic){for(seal in seals)seal.close();slab.close();throw e;}
        }
    }
    public static function review(p:MoldPlan):BoxMoldReview {
        penetrations(p);
        var original=source(p),rebuilt=reconstructed(p),difference=0.0;
        try {
            var missing=original.subtractAll([rebuilt]),excess=rebuilt.subtractAll([original]);
            difference=missing.volume()+excess.volume();missing.close();excess.close();
            if(difference>1e-5)throw "Reconstructed casting differs from source";
            original.close();rebuilt.close();
        }catch(e:Dynamic){original.close();rebuilt.close();throw e;}
        var parts:Array<Part> = [],members:Array<RemovalMember> = [];
        try {
            for(m in p.panels.members) {
                var local=ToolingPanel.fromPlan(m.panel).geometry(),world=PosedParts.moved(local,m.pose);local.close();parts.push(world);
                members.push({id:m.panel.id,part:world,direction:new Vector(m.withdrawal.x,m.withdrawal.y,m.withdrawal.z)});
            }
            coverage(p,parts);
            var rigidGroups:Array<{id:String,members:Array<String>}> = [];
            for(core in p.cores) {
                var bonded:Array<Part> = [];
                for(m in p.supportPanels)if(core.support.panelIds.indexOf(m.panel.id)>=0) {
                    var local=ToolingPanel.fromPlan(m.panel).geometry(),world=PosedParts.moved(local,m.pose);local.close();parts.push(world);
                    if(core.support.bondedPanelIds.indexOf(m.panel.id)>=0)bonded.push(new Part(world.shape.cloneShape()));
                    else members.push({id:m.panel.id,part:world,direction:new Vector(0,0,1)});
                }
                if(bonded.length>0) {
                    var group=Solids.union(bonded),id=core.id+"-bridge-carrier";parts.push(group);
                    members.push({id:id,part:group,direction:new Vector(0,0,1)});
                    rigidGroups.push({id:id,members:core.support.bondedPanelIds.copy()});
                }
            }
            var casting=reconstructed(p),steel=insert(p).geometry(),worldSteel=PosedParts.moved(steel,F.compose(p.document.setup.sourceToMold,p.document.input.inserts[0].pose));steel.close();
            var cured=Solids.union([casting,worldSteel]);parts.push(cured);
            var core=p.cores[0],covered=placedBox(core.cavity),foot=core.support.foot;
            if(foot!=null) {
                var fb=foot.foam;
                // Occupancy includes the two declared adhesive lines, with the finished
                // EPS foot between them. Both bonds must be released before withdrawal.
                covered=Solids.union([covered,placedBox({x:fb.x,y:fb.y,z:p.cavity.z-core.support.embedDepth,width:fb.width,depth:fb.depth,height:core.support.embedDepth})]);
            }
            parts.push(covered);
            var direction=core.support.kind==CoreSupportKind.BedSocket?-1.0:1.0;
            members.push({id:core.id,part:covered,direction:new Vector(0,0,direction)});
            members.push({id:"cured-casting",part:cured,direction:new Vector(0,0,1)});
            var demolding=SweptRemoval.analyze(members);
            members.pop();
            var assemblyGroups:Array<{id:String,members:Array<String>}> = [];
            if(core.support.kind==CoreSupportKind.Bridge) {
                // Bond the EPS to the reusable fixture before installing the hanging unit.
                members.pop();
                var id=core.id+"-bridge-carrier",index=-1;
                for(k in 0...members.length)if(members[k].id==id)index=k;
                if(index<0)throw "Missing hanging fixture group";
                var fixture=members[index];members.splice(index,1);
                var unit=Solids.union([new Part(fixture.part.shape.cloneShape()),new Part(covered.shape.cloneShape())]);parts.push(unit);
                members.push({id:core.id+"-suspended-unit",part:unit,direction:new Vector(0,0,1)});
                assemblyGroups.push({id:core.id+"-suspended-unit",members:[core.id].concat(core.support.bondedPanelIds)});
            } else members[members.length-1].direction=new Vector(0,0,1);
            var steelBody=placedBox(p.fixtures[0].body);parts.push(steelBody);members.push({id:p.fixtures[0].insertId,part:steelBody,direction:new Vector(0,0,1)});
            // For installation the bed approaches downward first, unlike its downward
            // demolding withdrawal. Reverse the corresponding +Z clearance motion.
            members[0].direction=new Vector(0,0,1);
            var assemblyReport=SweptRemoval.analyze(members);
            for(part in parts)part.close();
            return {assembly:assemblyReport,demolding:demolding,symmetricDifferenceMm3:difference,pending:p.pending.copy(),rigidGroups:rigidGroups,assemblyGroups:assemblyGroups};
        }catch(e:Dynamic){for(part in parts)part.close();throw e;}
    }
}
