import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.component.MachineKitComponents;
import machinekit.milling.*;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

class MillXYStageChecks {
	static var assertions = 0;

	static function check(ok:Bool, label:String):Void {
		assertions++;
		if (!ok) throw label;
	}

	static function near(a:Float, b:Float, label:String):Void check(Math.abs(a - b) < 1e-6, '$label ($a vs $b)');

	public static function main():Void {
		AxisBuilderChecks.run();
		assertions += MillDesignCheckTests.run();
		var reference = new BenchMill(false, false), f = MillFramePreview.layout(reference), z = MillFramePreview.zLayout(
			reference,
			f
		);
		var l = new MillXYStageLayout(f, z.spindleY, BenchMill.RAIL), axis = new MillXYStage(l), frame = new MillFrame(f, z, l);
		axis.validate();
		frame.validate();
		near(l.tableTop, 448, "stack height");
		near(f.yRailSpacing, 220, "broad Y guide stance");
		near(l.xRailSpacing, 160, "X guides support separated table ribs");
		check(2*l.yBlockSpan > 80, "Y saddle has a longer guide footprint");
		near(l.xMotorZ, 292, "X return drive independent of lowered Y shaft");
		near(l.yScrewZ, l.saddleBottom, "Y screw at saddle underside between rails");
		near(axis.axisOvertravel("x"), l.xOvertravel, "X bearing/guide overrun limit");
		near(axis.axisOvertravel("y"), l.yOvertravel, "Y bearing overrun limit");
		for (m in axis.components()) {
			var p = m.component.geometry();
			check(p.valid() && p.solidCount() == 1, 'connected valid solid ${m.id}');
			p.close();
		}
		var restored = MachineAssembly.decode(axis.encode(), MachineKitComponents.standard());
		restored.validate();
		check(restored.components().length == axis.components().length, "all stage recipes roundtrip");
		for (assembly in [cast(axis, MachineAssembly), restored]) {
			var model = new AssemblyModel("mm");
			assembly.addTo(model, "");
			var state = new AssemblyState(model.definition("xy-stage"));
			for (ix in 0...11) for (iy in 0...11) {
				var x = -l.xHalfTravel + ix * 2 * l.xHalfTravel / 10, y = -l.yHalfTravel + iy * 2 * l.yHalfTravel / 10;
				state.setJoint("x", x);
				state.setJoint("y", y);
				state.forwardKinematics();
				var table = state.worldPose("table"),
					saddle = state.worldPose("saddle"), motorX = state.worldPose("motorX"), motorY = state.worldPose("motorY");
				near(table.x, -x, "table follows X");
				near(table.y, l.centreY - y, "table follows Y");
				near(table.z, l.tableBottom, "table stays level");
				near(saddle.x, 0, "saddle independent of X");
				near(saddle.y, l.centreY - y, "saddle follows Y");
				near(motorX.x, -195, "X motor independent of X");
				near(motorX.y, l.centreY - y, "X drive rides saddle");
				near(motorY.y, l.yMotorFace, "Y motor remains fixed");
				near(state.joint("motorX-turn"), -state.joint("screwX-turn"), "opposed motor axis follows return belt");
				near(state.joint("screwX-turn"), l.nut.rotationFor(-x), "X screw signed lead");
				near(state.joint("screwY-turn"), l.nut.rotationFor(y), "Y screw signed lead");
			}
		}
		var poses:Map<String, AssemblyFrame> = axis.solvedPoses(), cache = new PosedParts();
		var pairs = [["fixedBearingY", "fixedPedestalY"], ["floatingBearingY", "floatingPedestalY"],
			["motorMountY", "fixedPedestalY"], ["motorY", "motorMountY"], ["fixedBearingX", "fixedPedestalX"],
			["floatingBearingX", "floatingPedestalX"], ["motorMountX", "saddle"], ["motorX", "motorMountX"],
			["nutY", "nutBracketY"], ["nutBracketY", "saddle"], ["nutX", "nutBracketX"], ["nutBracketX", "table"],
			["saddle", "blockY0"], ["table", "blockX0"], ["fixedPedestalX", "saddle"], ["floatingPedestalX", "saddle"],
			["motorCoverX", "motorMountX"]];
		for (i in 1...4) { pairs.push(["saddle", "blockY"+i]); pairs.push(["table", "blockX"+i]); }
		for (pair in pairs) {
			var a = cache.posed(axis.component(pair[0]), poses.get(pair[0])), b = cache.posed(
				axis.component(pair[1]),
				poses.get(pair[1])
			);
			near(a.distance(b), 0, 'supported contact $pair');
			var common = a.intersect(b);
			near(common.volume(), 0, 'no overlap at $pair');
			common.close();
			a.close();
			b.close();
		}
		for (id in ["motorX", "motorShaftX", "motorPulleyX", "screwPulleyX", "beltX"]) {
			near(cache.overlap(axis.component(id), poses.get(id), axis.saddle, poses.get("saddle")), 0, 'return drive clears saddle: $id');
		}
		var motorBody=cache.posed(axis.component("motorX"),poses.get("motorX"));
		var saddleBody=cache.posed(axis.saddle,poses.get("saddle"));
		var rebuiltSaddle=cache.posed(restored.components().filter(m->m.id=="saddle")[0].component,poses.get("saddle"));
		for (i in 0...4) for (hole in 1...5) {
			var point=F.compose(poses.get('blockY$i'),l.block.connector('mount$hole').frame);
			var sy=point.y<l.centreY?-1:1;
			var stem=cadkit.modeling.Part.cylinderSpan(2.5,l.saddleBottom+14,l.saddleBottom+MillCrossSaddle.WRENCH_HEIGHT,point.x,point.y);
			var handle=cadkit.modeling.Part.cylinderAlong(2.5,new cadkit.modeling.Vector(point.x,point.y,l.saddleBottom+MillCrossSaddle.WRENCH_HEIGHT),
				new cadkit.modeling.Vector(0,sy,0),l.saddleDepth/2-Math.abs(point.y-l.centreY)+40);
			for (tool in [stem,handle]) {
				for (body in [saddleBody,rebuiltSaddle,motorBody]) {
					var overlap=tool.intersect(body); near(overlap.volume(),0,"Y block has side-entry Allen-key access"); overlap.close();
				}
				tool.close();
			}
			var topProbe=cadkit.modeling.Part.cylinderSpan(2.5,l.xRailPlane-10,l.xRailPlane+1,point.x,point.y);
			var topCommon=topProbe.intersect(saddleBody); check(topCommon.volume()>1,"Y block access leaves closed top deck"); topCommon.close(); topProbe.close();
			var bolt:AssemblyFrame=poses.get('blockYBolt$i-$hole');
			near(bolt.z-MillCrossSaddle.BLOCK_SCREW_LENGTH,l.saddleBottom-4,"M4 screw engages 4 mm of block thread");
		}
		motorBody.close(); saddleBody.close(); rebuiltSaddle.close();
		var adapter=cache.posed(axis.component("nutBracketY"),poses.get("nutBracketY"));
		var rebuiltAdapter=cache.posed(restored.components().filter(m->m.id=="nutBracketY")[0].component,poses.get("nutBracketY"));
		near(rebuiltAdapter.volume(),adapter.volume(),"registered adapter, service slot and pilots survive recipe rebuild"); rebuiltAdapter.close();
		var nutBody=cache.posed(axis.component("nutY"),poses.get("nutY"));
		var nutSaddle=cache.posed(axis.saddle,poses.get("saddle"));
		var legacyAdapter=cache.posed(new MillYNutMount(new machinekit.component.ComponentValues().setBoolean("reinforced",true)),poses.get("nutBracketY"));
		var legacySaddle=axis.saddle.values().setBoolean("nutServiceAccess",false);
		var oldHost=new MillCrossSaddle(legacySaddle).geometry();
		var ring=machinekit.component.Solids.cut(cadkit.modeling.Part.cylinderSpan(8,36,76,0,-l.saddleDepth/2+5),
			[cadkit.modeling.Part.cylinderSpan(5,35,77,0,-l.saddleDepth/2+5)]);
		var rejected=new machinekit.assembly.Diagnostics();
		machinekit.design.DesignChecks.material(rejected,"old grease bore minimum 3 mm wall",ring,oldHost);
		check(rejected.hasErrors(),"actual tangent grease bore fails minimum wall check"); ring.close(); oldHost.close();
		var badNotch=machinekit.component.Solids.cut(axis.saddle.geometry(),[cadkit.modeling.Part.cylinderSpan(10,24,81,0,-l.saddleDepth/2+6)]);
		var railLand=cadkit.modeling.Part.box(l.xRailLength,l.block.spec.railWidth,3).translated(new cadkit.modeling.Vector(0,-l.xRailSpacing/2,77));
		var railSupport=new machinekit.assembly.Diagnostics();
		machinekit.design.DesignChecks.material(railSupport,"rail mounting land",railLand,badNotch);
		check(railSupport.hasErrors(),"over-deep service notch is rejected beneath X rail mounting land"); railLand.close(); badNotch.close();
		var front:AssemblyFrame=poses.get("nutBracketY");
		var blockedGrease=cadkit.modeling.Part.box(10,65,72.9).translated(new cadkit.modeling.Vector(0,front.y-25,l.saddleBottom+27.1));
		var blockedAccess=new machinekit.assembly.Diagnostics();
		machinekit.design.DesignChecks.access(blockedAccess,"front grease approach",blockedGrease,[{id:"old adapter",part:legacyAdapter}]);
		check(blockedAccess.hasErrors(),"old closed adapter rejects front grease tool approach"); blockedGrease.close(); legacyAdapter.close();
		var nutPose:AssemblyFrame=poses.get("nutY");
		near(nutPose.x,0,"Y nut remains centered between guide rails");
		near(nutPose.z,l.yScrewZ,"Y nut stays coaxial with the screw");
		var mountPose:AssemblyFrame=poses.get("nutBracketY");
		for(member in axis.components()) if(StringTools.startsWith(member.id,"nutBracketYBolt") || StringTools.startsWith(member.id,"nutYBolt")) {
			var bolt=cache.posed(member.component,poses.get(member.id));
			var parent=StringTools.startsWith(member.id,"nutYBolt")?nutBody:adapter;
			near(bolt.distance(parent),0,"Y nut fastener has a physical bearing seat");
			var common=bolt.intersect(parent); near(common.volume(),0,"Y nut fastener clears its clamped part"); common.close(); bolt.close();
			check(!MillXYStageIntegrity.yFixed(member.id),"Y nut fasteners belong to moving saddle");
			if(StringTools.startsWith(member.id,"nutYBolt")) {
				var screw:machinekit.standard.SocketHeadCapScrew=cast member.component;
				near(screw.length-l.nut.flangeThickness,6,"nut flange screw engages six millimetres of steel");
				check(screw.length-l.nut.flangeThickness<MillYNutMount.NUT_THREAD_DEPTH,"nut screw cannot bottom in blind adapter tap");
			}
		}
		for(p in MillYNutMount.anchors()) {
			var tool=cadkit.modeling.Part.cylinderAlongY(3,mountPose.y-70,mountPose.y-9,p.x,l.saddleBottom+p.y);
			for(body in [adapter,nutBody]) { var common=tool.intersect(body); near(common.volume(),0,"front key reaches recessed saddle bolt"); common.close(); }
			tool.close();
		}
		var grease=F.compose(poses.get("nutY"),l.nut.connector("lubricator").frame);
		var access=cadkit.modeling.Part.cylinderSpan(4,grease.z+3.1,l.xRailPlane+20,grease.x,grease.y);
		for(body in [nutSaddle,adapter]) {
			var greaseCommon=access.intersect(body); near(greaseCommon.volume(),0,"grease coupler reaches fitting through central passage and locating lip"); greaseCommon.close();
		}
		var serviceModel=new AssemblyModel("mm"); axis.addTo(serviceModel,"");
		var serviceState=new AssemblyState(serviceModel.definition("nut-service")); serviceState.setJoint("x",l.xHalfTravel); serviceState.forwardKinematics();
		for(id in ["railXFront","railXRear","blockX0","blockX1","blockX2","blockX3"]) {
			var part=cache.posed(axis.component(id),serviceState.worldPose(id)), common=access.intersect(part);
			near(common.volume(),0,"grease tool clears guides with X parked for service"); common.close(); part.close();
		}
		access.close(); adapter.close(); nutBody.close(); nutSaddle.close();
		near(cache.overlap(axis.component("nutY"), poses.get("nutY"), axis.saddle, poses.get("saddle")), 0, "nut body and fitting clear underside saddle tunnel");
		for (id in ["motorShaftX", "motorPulleyX", "screwPulleyX", "beltX", "motorMountX", "fixedBearingX"]) {
			near(cache.overlap(axis.component(id), poses.get(id), axis.component("motorCoverX"), poses.get("motorCoverX")), 0, 'return hood clears $id');
		}
		for (id in ["motorX", "motorY", "motorCoverX", "motorCoverY"]) {
			var b = cache.box(axis.component(id), poses.get(id));
			check(b.minX >= -f.baseWidth / 2 && b.maxX <= f.baseWidth / 2, 'drive stays inside base width: $id');
			var travel = id.indexOf("X") >= 0 ? l.yHalfTravel : 0;
			check(b.minY - travel >= f.front && b.maxY + travel <= f.rear + (id == "motorCoverY" ? 4 : 0), 'drive stays inside base depth over full Y stroke: $id');
		}
		cache.close();
		var audit = MillXYStageIntegrity.check(axis, frame);
		audit.throwIfErrors();
		check(!audit.hasErrors(), "continuous full Cartesian stroke audit");
		var blockedService=new MillXYStage(l);
		blockedService.addComponent("greaseHandlingObstruction",new MillPanel(10,10,10),F.translation(0,l.centreY-l.saddleDepth/2-50,l.saddleBottom+90));
		var serviceAudit=MillYNutDesignChecks.check(blockedService,frame);
		check(serviceAudit.items.filter(item->item.code=="design.access-blocked" && item.subject.indexOf("grease connector handling/greaseHandlingObstruction")>=0).length>0,
			"production service audit rejects blocked connector handling space");
		var uncut = MillXYStageIntegrity.check(axis, new MillFrame(f, z));
		check(uncut.hasErrors(), "missing bearing steel seats rejected");
		check(uncut.items.filter(item -> item.code == "mill-xy.full-stroke-collision").length > 0, "unrecessed frame collides with raised Y drive");
		near(l.yHalfTravel * 2, 160, "usable Y travel increases with table coverage retained");
		near(l.tableDepth / 2 - l.yHalfTravel, 25, "tool axis retains table-edge margin at both stops");
		near(f.columnFront - l.block.spec.railHeight - (l.centreY + l.yHalfTravel + l.tableDepth / 2), 3, "rear table clears lower Z rails");
		check(l.yScrewZ + 33 + l.clearance <= l.yBayTop, "motor cage fits below base roof");
		check(l.yBayTop > f.railPlane, "motor bay enters column sole above base");
		var blocked = new MillFrame(f, z, l);
		blocked.addComponent("obstruction", new MillPanel(20, 20, 20), F.translation(0, l.centreY, l.tableBottom + 5));
		var blockedAudit = MillXYStageIntegrity.check(axis, blocked);
		check(
			blockedAudit.items.filter(item -> item.code == "mill-xy.full-stroke-collision").length > 0,
			"audit detects an actual obstructed XY sweep"
		);
		var extractionBlocked = new MillFrame(f, z, l);
		extractionBlocked.addComponent("extractionObstruction", new MillPanel(20, 20, 20), F.translation(0, l.yServiceRear + 20, l.yScrewZ - 10));
		var extractionAudit = MillXYStageIntegrity.check(axis, extractionBlocked);
		check(extractionAudit.items.filter(item -> item.subject.indexOf("rear motor extraction/frame/extractionObstruction") >= 0).length > 0,
			"audit rejects blocked rear motor withdrawal");
		var concept = MillFramePreview.concept(reference, frame), model = new AssemblyModel("mm");
		concept.addTo(model, "");
		var state = new AssemblyState(model.definition("xyz-mill"));
		state.setJoint("xyStage/x", 125);
		state.setJoint("xyStage/y", 75);
		state.setJoint("zAxis/z", -250);
		state.forwardKinematics();
		near(state.worldPose("xyStage/table").x, -125, "integrated X");
		near(state.worldPose("xyStage/table").y, l.centreY - 75, "integrated Y");
		near(state.worldPose("zAxis/head").z, z.headBottom - 250, "integrated Z");
		Sys.println('Mill XY stage: $assertions assertions passed');
	}
}
