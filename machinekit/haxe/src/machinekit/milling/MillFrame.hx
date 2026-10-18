package machinekit.milling;

import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import machinekit.structural.CastInsert;
import machinekit.structural.CastingFeatures;
import machinekit.structural.CastingFeatures.CastingBox;
import machinekit.structural.MineralFoundation;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import materia.assembly.AssemblyFrames as F;

/** Mill structural layout: ribbed base, separate column and finished steel interfaces.
 * Casting construction and persistence remain in the shared structural components.
 */
class MillFrame extends MachineAssembly {
	public final layout:MillFrameLayout;
	public final base:MineralFoundation;
	public final column:MineralFoundation;
	public final zAxis:Null<MillZAxisLayout>;
	public final xyAxis:Null<MillXYStageLayout>;

	public function new(layout:MillFrameLayout, ?zAxis:MillZAxisLayout, ?xyAxis:MillXYStageLayout) {
		super();
		if (layout == null) throw "Mill frame needs a layout";
		this.layout = layout;
		if (zAxis != null && zAxis.frame != layout) throw "Frame and Z datums must share one layout";
		this.zAxis = zAxis;
		if (xyAxis != null && xyAxis.frame != layout) throw "Frame and XY datums must share one layout";
		this.xyAxis = xyAxis;
		var l = layout, baseVoids:Array<CastingBox> = [], baseInserts:Array<CastInsertPlacement> = [];
		var beamOuter = (l.yRailSpacing + l.seatWidth) / 2;
		var outerWidth = l.baseWidth / 2 - l.wall - beamOuter;
		var channels = [{x: 0.0, width: l.yRailSpacing - l.seatWidth},
			{x: -(beamOuter + outerWidth / 2), width: outerWidth},
			{x: beamOuter + outerWidth / 2, width: outerWidth}];
		var pocketFront = l.front + (l.footPockets ? 160 : l.wall), pocketRear = l.columnFront - l.wall;
		var pocketLength = (pocketRear - pocketFront - l.wall) / 2;
		// Open underside pockets leave full-height longitudinal rail beams and a
		// transverse rib. The rear column support stays solid through the base.
		for (i in 0...channels.length) for (row in 0...2) {
			var channel = channels[i];
			if (xyAxis != null && i == 0) continue; // The lowered drive needs a continuous central sole.
			baseVoids.push(CastingFeatures.box('underside-$i-$row', channel.width, pocketLength,
				l.baseHeight - l.roof + 1, channel.x,
				pocketFront + pocketLength / 2 + row * (pocketLength + l.wall) - l.baseCentreY, -1));
		}
		for (side in [-1, 1]) baseInserts.push(insert(side < 0 ? "railYLeft" : "railYRight",
			l.seatWidth, l.yRailLength, l.insertThickness, "Y guide seat",
			F.translation(side * l.yRailSpacing / 2, l.yRailStart + l.yRailLength / 2 - l.baseCentreY,
				l.railPlane - l.insertThickness)));
		if (xyAxis != null) for (end in ["Fixed", "Floating"]) {
			var fixed = end == "Fixed", y = fixed ? xyAxis.yFixedY : xyAxis.yFloatingY;
			baseInserts.push(insert('driveY$end', 100, fixed ? xyAxis.fixed.length : xyAxis.floating.length,
				l.insertThickness, "Y drive bearing seat", F.translation(0, y - l.baseCentreY, xyAxis.yMountPlane - l.insertThickness)));
		}
		if (xyAxis != null) {
			var a = xyAxis, floor = a.yMountPlane;
			// Open-top moving bridge channel; it ends before the column support.
			var channelEnd = l.columnFront - l.wall;
			baseVoids.push(CastingFeatures.box("y-bridge-channel", a.yChannelWidth, channelEnd - l.yRailStart,
				l.baseHeight - floor + 1, 0, (channelEnd + l.yRailStart) / 2 - l.baseCentreY, floor));
			// A small shaft passage beneath the intact front cross-web joins the two bays.
			var bayFront = a.yFixedY - a.fixed.length / 2 - a.clearance;
			baseVoids.push(CastingFeatures.box("y-shaft-tunnel", 24, bayFront - channelEnd + 2, 24,
				0, (bayFront + channelEnd) / 2 - l.baseCentreY, a.yScrewZ - 12));
			baseVoids.push(CastingFeatures.box("rear-y-drive-bay", a.yBayWidth, l.rear + 1 - bayFront, a.yBayTop - floor,
				0, (bayFront + l.rear + 1) / 2 - l.baseCentreY, floor));
			var frontBayEnd = a.yFloatingY + a.floating.length / 2 + a.clearance;
			baseVoids.push(CastingFeatures.box("front-y-bearing-bay", 106, frontBayEnd - l.front + 1, a.yBayTop - floor,
				0, (frontBayEnd + l.front - 1) / 2 - l.baseCentreY, floor));
			for (end in ["Front", "Rear"]) for (sx in [-1, 1]) for (z in [a.yHatchLowerZ, a.yHatchUpperZ]) {
				var rear = end == "Rear", y = rear ? l.rear - 8 : l.front + 8;
				baseInserts.push({id: 'hatch$end${sx < 0 ? "Left" : "Right"}${z == a.yHatchLowerZ ? "Lower" : "Upper"}',
					part: new CastInsert(new ComponentValues().setNumber("width", 20).setNumber("depth", 20)
						.setNumber("height", 10).setNumber("boreDiameter", 3.3).setToken("role", "service hatch anchor")),
					pose: F.fromRotationMatrix(sx * 75, y - l.baseCentreY, z,
						rear ? [1,0,0, 0,0,1, 0,-1,0] : [-1,0,0, 0,0,-1, 0,-1,0])});
			}
		}
		// Four independent steel pairs provide a broad column mounting interface.
		var footWidth = l.wall, footDepth = l.roof;
		for (sx in [-1, 1]) for (sy in [-1, 1]) {
			var id = corner(sx, sy), x = sx * (l.columnWidth / 2 - l.wall),
				y = l.columnCentreY + sy * (l.columnDepth / 2 - l.roof);
			baseInserts.push(insert('column$id', footWidth, footDepth, l.insertThickness, "column mounting pad",
				F.translation(x, y - l.baseCentreY, l.railPlane - l.insertThickness)));
			if (l.footPockets) {
				var fx = sx * (l.baseWidth / 2 - l.wall), fy = (sy < 0 ? l.front + 80 : l.rear - 80) - l.baseCentreY;
				baseVoids.push(CastingFeatures.box('foot-access-$id', 2 * l.wall + 2, 90, 100, fx, fy, 40));
				baseVoids.push(CastingFeatures.box('foot-stem-$id', 20, 20, 42, fx, fy, -1));
				baseInserts.push({id: 'foot$id', part: new CastInsert(new ComponentValues().setNumber("width", l.wall)
					.setNumber("depth", l.wall).setNumber("height", l.insertThickness).setNumber("boreDiameter", 18)
					.setToken("role", "leveling screw seat")), pose: F.translation(fx, fy, 40 - l.insertThickness)});
			} else baseInserts.push(insert('foot$id', l.wall, l.wall, l.insertThickness, "foundation foot seat",
				down(sx * (l.baseWidth / 2 - l.wall / 2),
					(sy < 0 ? l.front + l.wall / 2 : l.rear - l.wall / 2) - l.baseCentreY, l.insertThickness)));
		}
		base = new MineralFoundation(body(l.baseWidth, l.baseDepth, l.baseHeight, baseVoids), baseInserts);
		include("base", base, F.translation(0, l.baseCentreY, 0));
		if (l.footPockets) for (sx in [-1, 1]) for (sy in [-1, 1]) {
			var id = corner(sx, sy);
			addComponent('levelingFoot$id', new MillLevelingFoot());
			addMate('leveling-foot-$id', "fixed", 'base/foot$id', "top", 'levelingFoot$id', "mount");
		}

		var columnVoids:Array<CastingBox> = [], columnInserts:Array<CastInsertPlacement> = [];
		var frontWeb = zAxis == null ? l.roof : Math.max(l.roof, zAxis.channelBack + l.wall - (l.columnCentreY - l.columnDepth / 2));
		if (zAxis != null) {
			var channelFront = l.columnCentreY - l.columnDepth / 2 - 1;
			columnVoids.push(CastingFeatures.box("z-drive-channel", zAxis.channelWidth, zAxis.channelBack - channelFront,
				l.columnHeight + 2, 0, (channelFront + zAxis.channelBack) / 2 - l.columnCentreY, -1));
			for (side in [-1, 1]) for (end in ["Lower", "Upper"]) {
				var start = end == "Lower" ? 0.0 : l.zRailStart + l.zRailLength;
				columnInserts.push(insert('drive$end${side < 0 ? "Left" : "Right"}', l.seatWidth, 20, l.insertThickness,
					"Z bearing mount anchor", F.fromRotationMatrix(side * l.zRailSpacing / 2,
						-l.columnDepth / 2 + l.insertThickness - l.seatRise, start + 10, [1,0,0, 0,0,-1, 0,1,0])));
			}
		}
		var chamberHeight = (l.columnHeight - 2 * l.roof - l.wall) / 2;
		// Rear-open chambers leave a thick front web, side walls, sole, top and
		// cross rib. This avoids prescribing inaccessible sealed internal cores.
		for (row in 0...2) columnVoids.push(CastingFeatures.box('rear-chamber-$row',
			l.columnWidth - 2 * l.wall, l.columnDepth - frontWeb + 1, chamberHeight,
			0, (frontWeb + 1) / 2, l.roof + row * (chamberHeight + l.wall)));
		for (side in [-1, 1]) columnInserts.push(insert(side < 0 ? "railZLeft" : "railZRight",
			l.seatWidth, l.zRailLength, l.insertThickness, "Z guide seat",
			F.fromRotationMatrix(side * l.zRailSpacing / 2, -l.columnDepth / 2 + l.insertThickness - l.seatRise,
				l.zRailStart + l.zRailLength / 2, [1,0,0, 0,0,-1, 0,1,0])));
		for (sx in [-1, 1]) for (sy in [-1, 1]) columnInserts.push(insert('foot${corner(sx, sy)}',
			footWidth, footDepth, l.insertThickness, "column sole pad",
			down(sx * (l.columnWidth / 2 - l.wall), sy * (l.columnDepth / 2 - l.roof), l.insertThickness)));
		column = new MineralFoundation(body(l.columnWidth, l.columnDepth, l.columnHeight, columnVoids), columnInserts, "footLeftFront");
		include("column", column, F.translation(0, l.columnCentreY, l.railPlane));
		// Mating frames coincide; the physical finished faces oppose each other.
		var first = columnInserts.filter(row -> row.id == "footLeftFront")[0];
		addMemberConnector("column/footLeftFront", "baseSeat", F.compose(F.inverse(first.pose),
			Solids.axial(first.pose.x, first.pose.y, 0)));
		addMate("column-on-base", "fixed", "base/columnLeftFront", "top", "column/footLeftFront", "baseSeat");
		exposeConnector("mount", "base/casting", "mount");
		for (id in ["railYLeft", "railYRight"]) exposeConnector(id, 'base/$id', "top");
		for (id in ["railZLeft", "railZRight"]) exposeConnector(id, 'column/$id', "top");
		for (sx in [-1, 1]) for (sy in [-1, 1]) exposeConnector('foot${corner(sx, sy)}', 'base/foot${corner(sx, sy)}', "top");
	}

	static function corner(x:Int, y:Int):String return (x < 0 ? "Left" : "Right") + (y < 0 ? "Front" : "Rear");
	static function down(x:Float, y:Float, z:Float):materia.assembly.AssemblyRecord.AssemblyFrame return F.fromRotationMatrix(x, y, z, [1,0,0, 0,-1,0, 0,0,-1]);
	static function insert(id:String, width:Float, depth:Float, thickness:Float, role:String,
		pose:materia.assembly.AssemblyRecord.AssemblyFrame):CastInsertPlacement
		return {id: id, part: new CastInsert(new ComponentValues().setNumber("width", width).setNumber("depth", depth)
			.setNumber("height", thickness).setToken("role", role)), pose: pose};
	static function body(width:Float, depth:Float, height:Float, voids:Array<CastingBox>):ComponentValues
		return new ComponentValues().setNumber("width", width).setNumber("depth", depth).setNumber("height", height)
			.setToken("voids", CastingFeatures.encode(voids));
}
