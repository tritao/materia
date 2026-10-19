package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.Diagnostics;
import machinekit.assembly.PosedParts;
import machinekit.component.Solids;
import machinekit.design.DesignChecks as D;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Process/assembly checks for the study cartridge. Limits are geometric design
 * choices, not supplier approval or bolt strength calculations. Front access is
 * required; no assumed removal of the ballnut from its screw.
 */
class MillYNutDesignChecks {
	public static function check(axis:MillXYStage,frame:MillFrame):Diagnostics {
		var out=new Diagnostics(), cache=new PosedParts(), owned:Array<Part> = [];
		try run(axis,frame,out,cache,owned) catch(e:Dynamic) { for(p in owned) p.close(); cache.close(); throw e; }
		for(p in owned) p.close(); cache.close(); return out;
	}

	static function run(axis:MillXYStage,frame:MillFrame,out:Diagnostics,cache:PosedParts,owned:Array<Part>):Void {
		var l=axis.layout, adapter=cache.local(axis.component("nutBracketY")), saddle=cache.local(axis.saddle);
		var flangeIndex=0;
		for(p in l.nut.boltPattern()) {
			var flangeScrew:machinekit.standard.SocketHeadCapScrew=cast axis.component('nutYBolt${flangeIndex++}');
			if(flangeScrew.spec.size!="M5") out.error("design.fastener-size","Y nut flange","M5 screw required for M5 adapter tap");
			D.blindFastener(out,"Y nut M5",flangeScrew.length,l.nut.flangeThickness,
				MillYNutMount.NUT_THREAD_DEPTH,8.5,1.5,MillYNutMount.THICKNESS,5,1,2);
			var ring=Solids.cut(Part.cylinderSpan(5.1,0,MillYNutMount.NUT_DRILL_DEPTH,p.x,p.y),[Part.cylinderSpan(2.1,-1,MillYNutMount.NUT_DRILL_DEPTH+1,p.x,p.y)]); owned.push(ring);
			D.material(out,'M5 (${p.x},${p.y}): 3 mm ligament',ring,adapter);
			var bore=Part.cylinderSpan(2.1,0,MillYNutMount.NUT_DRILL_DEPTH,p.x,p.y); owned.push(bore);
			D.access(out,"actual M5 blind drilling",bore,[{id:"adapter",part:adapter}]);
			var back=Part.cylinderSpan(2.1,MillYNutMount.NUT_DRILL_DEPTH,MillYNutMount.NUT_DRILL_DEPTH+2,p.x,p.y); owned.push(back);
			D.material(out,"M5 actual 2 mm back wall",back,adapter);
			var drill=Part.cylinderSpan(3,-40,0,p.x,p.y); owned.push(drill);
			D.access(out,"adapter drill/tap approach",drill,[{id:"adapter",part:adapter}]);
		}
		for(p in MillYNutMount.anchors()) {
			D.minimum(out,"M6 counterbore to plate edge",50-Math.abs(p.x)-5.5,4);
			var anchor:machinekit.standard.SocketHeadCapScrew=cast axis.component('nutBracketYBolt${p.x}-${p.y}');
			if(anchor.spec.size!="M6") out.error("design.fastener-size","Y nut saddle anchor","M6 screw required for M6 saddle tap");
			D.blindFastener(out,"Y saddle M6",anchor.length,MillYNutMount.ANCHOR_SEAT,22,23,1.5,MillYNutMount.BOSS_DEPTH,9,2,3);
			var ring=Solids.cut(Part.cylinderSpan(9.5,6,12,p.x,p.y),[Part.cylinderSpan(5.5,5,13,p.x,p.y)]); owned.push(ring);
			D.material(out,"M6 counterbore: 4 mm ligament",ring,adapter);
			var through=Part.cylinderSpan(3.3,0,12,p.x,p.y); owned.push(through);
			var counter=Part.cylinderSpan(5.5,6,12,p.x,p.y); owned.push(counter);
			D.access(out,"actual M6 adapter drilling",through,[{id:"adapter",part:adapter}]);
			D.access(out,"actual M6 head recess",counter,[{id:"adapter",part:adapter}]);
			var seat=Solids.cut(Part.cylinderSpan(5,5,6,p.x,p.y),[Part.cylinderSpan(3.3,4,7,p.x,p.y)]); owned.push(seat);
			D.material(out,"M6 head bearing seat",seat,adapter);
			var castingRing=Solids.cut(Part.cylinderAlongY(7.5,-l.saddleDepth/2,-l.saddleDepth/2+24.5,p.x,p.y),
				[Part.cylinderAlongY(2.5,-l.saddleDepth/2-1,-l.saddleDepth/2+25.5,p.x,p.y)]); owned.push(castingRing);
			D.material(out,"M6 casting tap: 5 mm ligament",castingRing,saddle);
			var bore=Part.cylinderAlongY(2.5,-l.saddleDepth/2,-l.saddleDepth/2+24.5,p.x,p.y); owned.push(bore);
			D.access(out,"actual M6 saddle drilling",bore,[{id:"saddle",part:saddle}]);
			var back=Part.cylinderAlongY(2.5,-l.saddleDepth/2+24.5,-l.saddleDepth/2+27.5,p.x,p.y); owned.push(back);
			D.material(out,"M6 actual 3 mm back wall",back,saddle);
		}
		// The port is intentionally open. Check the two retained cheeks against
		// the real cut solid, rather than exempting a dangerously thin closed hole.
		for(x in [-16.0,16.0]) {
			var cheek=Part.box(10,16,40).translated(new Vector(x,-l.saddleDepth/2+8,36)); owned.push(cheek);
			D.material(out,"grease notch retained 10 mm cheek",cheek,saddle);
		}
		for(side in [-1,1]) {
			var land=Part.box(l.xRailLength,l.block.spec.railWidth,3).translated(new Vector(0,side*l.xRailSpacing/2,l.xRailPlane-l.saddleBottom-3)); owned.push(land);
			D.material(out,"continuous 3 mm X rail bearing land",land,saddle);
		}
		var poses=axis.solvedPoses(), mount:AssemblyFrame=poses.get("nutBracketY");
		var nut=cache.posed(axis.component("nutY"),poses.get("nutY")); owned.push(nut);
		var plate=cache.posed(axis.component("nutBracketY"),mount); owned.push(plate);
		var host=cache.posed(axis.saddle,poses.get("saddle")); owned.push(host);
		// Bench assembly: keys approach from the barrel side before the saddle
		// is introduced. This is deliberately not claimed as in-place screw access.
		var boltIndex=0;
		for(p in l.nut.boltPattern()) {
			var bolt:machinekit.standard.SocketHeadCapScrew=cast axis.component('nutYBolt${boltIndex}');
			D.coaxial(out,"nut flange/adapter/screw",poses.get('nutYBolt${boltIndex++}'),F.compose(mount,F.translation(p.x,p.y,0)));
			D.minimum(out,"nut flange diametral screw clearance",5.5-bolt.diameter,0.3);
			var key=Part.cylinderSpan(2.5,-65,-l.nut.flangeThickness,p.x,p.y); owned.push(key);
			var placed=PosedParts.moved(key,mount); owned.push(placed);
			D.access(out,"bench ballnut screw key approach",placed,[{id:"nut",part:nut},{id:"adapter",part:plate}]);
		}
		// Continuous conservative envelope of the nut, flange and rear screw
		// heads during front insertion/removal, followed by the locating pilots.
		var cartridge=Part.cylinderSpan(27.5,-l.nut.bodyLength-l.nut.flangeThickness,80); owned.push(cartridge);
		var insertion=PosedParts.moved(cartridge,mount); owned.push(insertion);
		D.access(out,"nut cartridge front insertion",insertion,[{id:"saddle",part:host}]);
		for(p in MillYNutMount.pilots()) {
			var pin=Part.cylinderSpan(3,-8,80,p.x,p.y); owned.push(pin);
			var path=PosedParts.moved(pin,mount); owned.push(path);
			D.access(out,"locating pilot insertion",path,[{id:"saddle",part:host}]);
		}
		for(p in MillYNutMount.anchors()) {
			var id='nutBracketYBolt${p.x}-${p.y}';
			var bolt:machinekit.standard.SocketHeadCapScrew=cast axis.component(id);
			D.coaxial(out,"adapter/saddle/screw",poses.get(id),F.compose(mount,F.translation(p.x,p.y,0)));
			D.minimum(out,"adapter diametral M6 clearance",6.6-bolt.diameter,0.4);
			var key=Part.cylinderSpan(2.5,9,80,p.x,p.y); owned.push(key);
			var placed=PosedParts.moved(key,mount); owned.push(placed);
			D.access(out,"installed M6 key approach",placed,[{id:"adapter",part:plate},{id:"nut",part:nut},{id:"saddle",part:host}]);
		}
		var model=new AssemblyModel("mm"); axis.addTo(model,"");
		var state=new AssemblyState(model.definition("Y-nut-maintenance")); state.setJoint("x",l.xHalfTravel); state.forwardKinematics();
		var obstacles:Array<{id:String,part:Part}> = [];
		for(member in axis.components()) {
			var part=cache.posed(member.component,state.worldPose(member.id)); owned.push(part);
			obstacles.push({id:member.id,part:part});
		}
		var framePoses=frame.solvedPoses();
		for(member in frame.components()) {
			var part=cache.posed(member.component,framePoses.get(member.id)); owned.push(part);
			obstacles.push({id:"frame/"+member.id,part:part});
		}
		// Z is parked at its retracted zero position for maintenance.
		if(frame.zAxis!=null) {
			var zAxis=new MillZAxis(frame.zAxis), zPoses=zAxis.solvedPoses();
			for(member in zAxis.components()) {
				var part=cache.posed(member.component,zPoses.get(member.id)); owned.push(part);
				obstacles.push({id:"zAxis/"+member.id,part:part});
			}
		}
		// Assumed service-tool requirement: 8 mm slim extension, 10 mm elbow,
		// 40 mm handling space outside the front face. Approach from the front
		// 5 mm above the seated position, lower onto nipple, reverse to remove.
		var grease=F.compose(poses.get("nutY"),l.nut.connector("lubricator").frame);
		var z=l.saddleBottom, front=mount.y;
		var stem=Part.box(10,65,69.5).translated(new Vector(0,grease.y-30,z+30.5)); owned.push(stem);
		var elbow=Part.box(12,72,17).translated(new Vector(0,front-25,z+89)); owned.push(elbow);
		var hand=Part.box(42,42,47).translated(new Vector(0,front-50,z+74)); owned.push(hand);
		// 20 mm minimum hose centreline bend radius + 5 mm tube radius +
		// 1 mm clearance, outside the front face (continuous bend envelope).
		var hoseBend=Part.box(52,52,52).translated(new Vector(0,front-60,z+70)); owned.push(hoseBend);
		D.access(out,"grease extension front approach",stem,obstacles);
		// Coupler mouth surrounds the nipple: mating clearance is modelled,
		// rather than stopping the tool short of the actual connection.
		var sleeve=Solids.cut(Part.cylinderSpan(5,z+25.5,z+100,grease.x,grease.y),
			[Part.cylinderSpan(1.7,z+25,z+27.2,grease.x,grease.y)]); owned.push(sleeve);
		D.access(out,"grease connection and 5 mm disconnect stroke",sleeve,obstacles);
		D.access(out,"grease elbow/hose approach",elbow,obstacles);
		D.access(out,"grease connector handling",hand,obstacles);
		D.access(out,"grease hose minimum bend radius",hoseBend,obstacles);
	}
}
