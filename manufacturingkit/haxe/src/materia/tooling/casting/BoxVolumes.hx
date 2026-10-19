package materia.tooling.casting;

import materia.tooling.casting.MoldPlan.MoldBox;

/** Exact orthogonal box set operations. Outputs are disjoint finished volumes, not meshes. */
class BoxVolumes {
    public static function intersection(a:MoldBox,b:MoldBox):Null<MoldBox> {
        var x=Math.max(a.x,b.x),y=Math.max(a.y,b.y),z=Math.max(a.z,b.z);
        var w=Math.min(a.x+a.width,b.x+b.width)-x,d=Math.min(a.y+a.depth,b.y+b.depth)-y,h=Math.min(a.z+a.height,b.z+b.height)-z;
        return w>1e-8 && d>1e-8 && h>1e-8?{x:x,y:y,z:z,width:w,depth:d,height:h}:null;
    }
    public static function difference(a:MoldBox,b:MoldBox):Array<MoldBox> {
        var i=intersection(a,b);if(i==null)return [copy(a)];
        var out:Array<MoldBox> = [];
        function piece(x:Float,y:Float,z:Float,w:Float,d:Float,h:Float):Void {if(w>1e-8 && d>1e-8 && h>1e-8)out.push({x:x,y:y,z:z,width:w,depth:d,height:h});}
        piece(a.x,a.y,a.z,i.x-a.x,a.depth,a.height);
        piece(i.x+i.width,a.y,a.z,a.x+a.width-i.x-i.width,a.depth,a.height);
        piece(i.x,a.y,a.z,i.width,i.y-a.y,a.height);
        piece(i.x,i.y+i.depth,a.z,i.width,a.y+a.depth-i.y-i.depth,a.height);
        piece(i.x,i.y,a.z,i.width,i.depth,i.z-a.z);
        piece(i.x,i.y,i.z+i.height,i.width,i.depth,a.z+a.height-i.z-i.height);
        return out;
    }
    public static function subtract(add:Array<MoldBox>,cuts:Array<MoldBox>):Array<MoldBox> {
        var out:Array<MoldBox> = [];
        for(box in add) {
            var next=[copy(box)];
            for(c in out.concat(cuts)) {
                var pieces:Array<MoldBox> = [];for(p in next)for(q in difference(p,c))pieces.push(q);next=pieces;
            }
            for(p in next)out.push(p);
            if(out.length>2048)throw "Too many orthogonal volume pieces";
        }
        return out;
    }
    public static function groups(boxes:Array<MoldBox>):Array<Array<MoldBox>> {
        var remaining=boxes.copy(),out:Array<Array<MoldBox>> = [];
        remaining.sort((a,b)->a.z!=b.z?Reflect.compare(a.z,b.z):a.y!=b.y?Reflect.compare(a.y,b.y):Reflect.compare(a.x,b.x));
        function connected(a:MoldBox,b:MoldBox):Bool {
            var x=Math.min(a.x+a.width,b.x+b.width)-Math.max(a.x,b.x),y=Math.min(a.y+a.depth,b.y+b.depth)-Math.max(a.y,b.y),z=Math.min(a.z+a.height,b.z+b.height)-Math.max(a.z,b.z);
            return x>=-1e-7 && y>=-1e-7 && z>=-1e-7 && ((x>1e-7?1:0)+(y>1e-7?1:0)+(z>1e-7?1:0)>=2);
        }
        while(remaining.length>0) {
            var group=[remaining.shift()],changed=true;
            while(changed) {
                changed=false;
                var i=remaining.length-1;
                while(i>=0) {
                    var match=false;for(a in group)if(connected(a,remaining[i]))match=true;
                    if(match){group.push(remaining[i]);remaining.splice(i,1);changed=true;}i--;
                }
            }
            out.push(group);
        }
        return out;
    }
    public static function copy(b:MoldBox):MoldBox return {x:b.x,y:b.y,z:b.z,width:b.width,depth:b.depth,height:b.height};
    public static function volume(boxes:Array<MoldBox>):Float {var v=0.0;for(b in boxes)v+=b.width*b.depth*b.height;return v;}
}
