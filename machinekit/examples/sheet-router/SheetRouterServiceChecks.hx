import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;

typedef SheetServiceRoute={var id:String;var diameterMm:Float;var points:Array<Array<Float>>;}
typedef SheetServiceProbe={var id:String;var part:Part;var lo:Array<Float>;var hi:Array<Float>;}

/** Conservative native-B-rep checks of service routes, removal spaces and top access. */
class SheetRouterServiceChecks {
    public static function routes(machine:SheetRouter):Array<SheetServiceRoute> {
        var result:Array<SheetServiceRoute> = [],table=machine.zeroPose("vacuumTable"),left=machine.zeroPose("frameLeft").x,right=machine.zeroPose("frameRight").x,back=machine.zeroPose("frameBack").y;
        for(i in 0...6) {
            var row=Std.int(i/2),side=i%2==0?-1:1,port=F.compose(table,machine.table.zoneFrame(i));
            var y=port.y+SheetRouterServiceTable.portOffsetY,valve=machine.zeroPose('zoneValve${i+1}'),crossZ=row==1?400.0:300.0;
            var inner=side<0?left+125:right-125,outside=valve.x+side*150;
            result.push({id:'zone${i+1}',diameterMm:60,points:[
                [port.x,y,table.z],[port.x,y,crossZ],[inner,y,crossZ],[inner,valve.y,crossZ],
                [outside,valve.y,crossZ],[outside,valve.y,650],[valve.x,valve.y,650],[valve.x,valve.y,valve.z+120]
            ]});
            var header=i%2==0?"leftVacuumHeader":"vacuumHeader",out=i%2==0?row+1:4-row;
            var h=F.compose(machine.zeroPose(header),machine.component(header).connector('out$out').frame);
            result.push({id:'feed${i+1}',diameterMm:60,points:[[h.x,h.y,h.z],[h.x,h.y,300],[h.x,valve.y,300],[valve.x,valve.y,300],[valve.x,valve.y,valve.z]]});
        }
        var h=F.compose(machine.zeroPose("vacuumHeader"),machine.component("vacuumHeader").connector("out1").frame);
        var l=F.compose(machine.zeroPose("leftVacuumHeader"),machine.component("leftVacuumHeader").connector("input").frame);
        result.push({id:"leftTrunk",diameterMm:90,points:[[h.x,h.y,h.z],[h.x,h.y,450],[right+800,h.y,450],[right+800,back+180,450],[left-850,back+180,450],[left-850,l.y,450],[left-125,l.y,450],[left-125,l.y,l.z],[l.x,l.y,l.z]]});
        var pump=F.compose(machine.zeroPose("vacuumBlower"),machine.component("vacuumBlower").connector("vacuum").frame);
        var inlet=F.compose(machine.zeroPose("vacuumHeader"),machine.component("vacuumHeader").connector("input").frame);
        result.push({id:"pumpFeed",diameterMm:90,points:[[pump.x,pump.y,pump.z],[pump.x,pump.y,650],[right+1000,pump.y,650],[right+1000,inlet.y,650],[right+1000,inlet.y,inlet.z],[inlet.x,inlet.y,inlet.z]]});
        return result;
    }
    static function rounded(input:Array<Array<Float>>,radius:Float,id:String):Array<Array<Float>> {
        var p:Array<Vector> = [];
        for(q in input) {var v=new Vector(q[0],q[1],q[2]);if(p.length==0||v.subtract(p[p.length-1]).length()>0.01)p.push(v);}
        var i=1;
        while(i<p.length-1) {if(p[i].subtract(p[i-1]).normalized().dot(p[i+1].subtract(p[i]).normalized())>0.9999)p.splice(i,1);else i++;}
        var out=[input[0]];
        for(j in 2...p.length-1)if(p[j].subtract(p[j-1]).length()<2*radius-0.01)
            throw 'Service route '+id+' has overlapping adjacent bends at '+j;
        for(j in 1...p.length-1) {
            var before=p[j].subtract(p[j-1]),after=p[j+1].subtract(p[j]);
            if(before.length()<radius||after.length()<radius)throw "Service route "+id+" cannot fit bend R"+radius+" at "+j+": "+before.length()+" / "+after.length();
            var a=before.normalized(),b=after.normalized();if(Math.abs(a.dot(b))>0.001)throw "Expected orthogonal service route";
            var centre=p[j].subtract(a.scale(radius)).add(b.scale(radius));
            for(k in 0...10) {
                var theta=Math.PI*k/18,v=centre.subtract(b.scale(radius*Math.cos(theta))).add(a.scale(radius*Math.sin(theta)));
                out.push([v.x,v.y,v.z]);
            }
        }
        var last=p[p.length-1];out.push([last.x,last.y,last.z]);return out;
    }
    static function bounds(p:Part):{lo:Array<Float>,hi:Array<Float>} {
        var box=p.shape.bounds(),a=box.get_min(),b=box.get_max();return {lo:[a.get_x(),a.get_y(),a.get_z()],hi:[b.get_x(),b.get_y(),b.get_z()]};
    }
    static function overlaps(a:SheetServiceProbe,b:{lo:Array<Float>,hi:Array<Float>}):Bool {
        for(i in 0...3)if(a.lo[i]>b.hi[i]||a.hi[i]<b.lo[i])return false;return true;
    }
    public static function check(machine:SheetRouter,prep:SheetRouterFramePreparation):Dynamic {
        var paths=routes(machine),probes:Array<SheetServiceProbe> = [],assertions=0,errors:Array<String> = [];
        for(path in paths) {
            var points=rounded(path.points,path.diameterMm==60?75:100,path.id),r=path.diameterMm/2+5;
            for(i in 1...points.length) {
                var a=points[i-1],b=points[i],lo=[for(k in 0...3)Math.min(a[k],b[k])-r],hi=[for(k in 0...3)Math.max(a[k],b[k])+r];
                var blank=Part.box(hi[0]-lo[0],hi[1]-lo[1],hi[2]-lo[2]),part=blank.translated(new Vector((lo[0]+hi[0])/2,(lo[1]+hi[1])/2,lo[2]));blank.close();
                probes.push({id:path.id+"/"+i,part:part,lo:lo,hi:hi});
            }
        }
        function clear(id:String,part:Part,checks:Array<SheetServiceProbe>):Void {
            var box=bounds(part);
            for(probe in checks)if(overlaps(probe,box)) {
                var common=probe.part.shape.common(part.shape),volume=common.volume();common.close();assertions++;
                if(volume>0.1)errors.push(probe.id+" versus "+id+": "+volume+" mm3");
            }
        }
        for(stock in prep.stocks)clear(stock.id,stock.raw,probes);
        var removal:Array<SheetServiceProbe> = [];
        for(i in 0...6) {
            var p=machine.zeroPose('zoneValve${i+1}'),side=i%2==0?-1:1,lo=[side<0?p.x-450:p.x-55,p.y-60,p.z-10],hi=[side<0?p.x+55:p.x+450,p.y+60,p.z+135];
            var box=Part.box(hi[0]-lo[0],hi[1]-lo[1],hi[2]-lo[2]),part=box.translated(new Vector((lo[0]+hi[0])/2,p.y,lo[2]));box.close();
            removal.push({id:'valveWithdrawal${i+1}',part:part,lo:lo,hi:hi});
        }
        for(stock in prep.stocks)clear(stock.id,stock.raw,removal);
        for(id in ["vacuumServiceStandLeft","vacuumServiceStandRight","chainYFixedBracket","chainXFixedBracket","vacuumBlower","vacuumHeader","leftVacuumHeader","cabinet","rackSupport"]) {
            var raw=machine.component(id).geometry(),placed=PosedParts.moved(raw,machine.zeroPose(id));raw.close();
            // Hoses intentionally enter their own pump/header ports; test other routes.
            var selected=[for(p in probes)if(!(id=="vacuumBlower"&&StringTools.startsWith(p.id,"pumpFeed/"))&&!(id=="vacuumHeader"&&(StringTools.startsWith(p.id,"feed")||StringTools.startsWith(p.id,"leftTrunk/")||StringTools.startsWith(p.id,"pumpFeed/")))&&!(id=="leftVacuumHeader"&&(StringTools.startsWith(p.id,"feed")||StringTools.startsWith(p.id,"leftTrunk/"))))p];
            clear(id,placed,selected);clear(id,placed,removal);placed.close();
        }
        var state=new AssemblyState(machine.definition());
        var moving:Map<String,Part> = new Map();
        for(id in ["beam","beamFootLeft","beamFootRight","xCarriage","zCarriage","spindleMount","spindle","shoe"])moving.set(id,machine.component(id).geometry());
        for(y in [0.0,machine.routerSpec.workY/2,machine.spec.travelY])for(x in [0.0,800.0,1600.0])for(z in [0.0,200.0]) {
            state.setJoint("x",x);state.setJoint("y",y);state.setJoint("z",z);state.forwardKinematics();
            for(id in ["beam","beamFootLeft","beamFootRight","xCarriage","zCarriage","spindleMount","spindle","shoe"]) {
                var placed=PosedParts.moved(moving.get(id),state.worldPose(id));clear(id+" at "+x+"/"+y+"/"+z,placed,probes);placed.close();
            }
            for(axis in ["x","y"]) {
                var position=axis=="x"?x:y,chain=machine.cableChainGeometry(axis,position);
                var binding=machine.cableChainBindings[axis=="x"?1:0];
                var placed=PosedParts.moved(chain,state.worldPose(binding.member));chain.close();clear("chain "+axis,placed,probes);placed.close();
            }
        }
        var tablePart=machine.table.geometry(),placedTable=PosedParts.moved(tablePart,machine.zeroPose("vacuumTable"));tablePart.close();
        for(i in 0...6) {
            var f=F.compose(machine.zeroPose("vacuumTable"),machine.table.zoneFrame(i)),bottom=machine.zeroPose("vacuumTable").z+16;
            var lance=Part.cylinderSpan(20,bottom,machine.boardTop+200,f.x,f.y),common=lance.shape.common(placedTable.shape),volume=common.volume();common.close();lance.close();assertions++;
            if(volume>0.1)errors.push("Top plenum inspection lance blocked at zone "+(i+1));
        }
        placedTable.close();for(p in moving)p.close();for(p in probes)p.part.close();for(p in removal)p.part.close();
        return {status:errors.length==0?"service access checks pass":"service access checks fail",assertions:assertions,errors:errors,routes:paths,
            basis:"DN50 branches (60 mm envelope), DN75 trunk (90 mm); R75/R100 elbows; 5 mm inflated clearance; 18 gantry poses per bed; external valve withdrawal; top D40 inspection lance",valveReachMm:450};
    }
}
