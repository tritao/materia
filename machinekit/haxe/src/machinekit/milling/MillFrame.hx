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
			baseVoids.push(CastingFeatures.box('underside-$i-$row', channel.width, pocketLength,
				l.baseHeight - l.roof + 1, channel.x,
				pocketFront + pocketLength / 2 + row * (pocketLength + l.wall) - l.baseCentreY, -1));
		}
		if (xyAxis != null) for (side in [-1,1]) {
			var shoulderOuter = (l.yRailSpacing+l.seatWidth)/2+12;
			var width = l.baseWidth/2-shoulderOuter;
			baseVoids.push(CastingFeatures.box('guide-shoulder-outside-$side',width,l.yRailLength,21,
				side*(shoulderOuter+width/2),l.yRailStart+l.yRailLength/2-l.baseCentreY,l.baseHeight-20));
		}
		if (xyAxis != null) {
			var start = xyAxis.yFloatingY+xyAxis.yFloatingDepth/2+12, end = l.yRailStart+l.yRailLength;
			baseVoids.push(CastingFeatures.box("between-y-shoulders",l.yRailSpacing-l.seatWidth-24,end-start,21,
				0,(start+end)/2-l.baseCentreY,l.baseHeight-20));
		}
		for (side in [-1, 1]) baseInserts.push(insert(side < 0 ? "railYLeft" : "railYRight",
			l.seatWidth, l.yRailLength, l.insertThickness, "Y guide seat",
			F.translation(side * l.yRailSpacing / 2, l.yRailStart + l.yRailLength / 2 - l.baseCentreY,
				l.railPlane - l.insertThickness)));
		if (xyAxis != null) for (end in ["Fixed", "Floating"]) {
			var fixed = end == "Fixed", y = fixed ? xyAxis.yFixedY : xyAxis.yFloatingY;
			baseInserts.push(insert('driveY$end', 100, fixed ? xyAxis.yFixedDepth : xyAxis.yFloatingDepth,
				l.insertThickness, "Y drive bearing seat", F.translation(0, y - l.baseCentreY, xyAxis.yMountPlane - l.insertThickness)));
		}
		if (xyAxis != null) {
			var a = xyAxis;
			var bayFront = a.yFixedY - a.yFixedDepth / 2 - a.clearance;
			// Only shallow bearing/motor-foot recesses remain in the base.
			baseVoids.push(CastingFeatures.box("rear-y-drive-seat", a.yBayWidth, a.yServiceRear + 1 - bayFront,
				l.baseHeight + 1 - a.yMountPlane, 0, (bayFront+a.yServiceRear+1)/2-l.baseCentreY, a.yMountPlane));
			baseVoids.push(CastingFeatures.box("front-y-bearing-seat", 106, a.yFloatingDepth+2*a.clearance,
				l.baseHeight + 1 - a.yMountPlane, 0, a.yFloatingY-l.baseCentreY, a.yMountPlane));
		}

		// Four independent steel pairs provide a broad column mounting interface.
		var flared = l.columnFootWidth > l.columnWidth;
		var footWidth = l.wall, footDepth = l.roof;
		for (sx in [-1, 1]) for (sy in [-1, 1]) {
			var id = corner(sx, sy), x = sx * (flared ? (l.columnWidth+l.columnFootWidth)/4 : l.columnWidth/2-l.wall),
				y = l.columnCentreY + sy * (l.columnDepth / 2 - l.roof);
			baseInserts.push({id: 'column$id', part: new CastInsert(new ComponentValues().setNumber("width",footWidth).setNumber("depth",footDepth)
				.setNumber("height",l.insertThickness).setNumber("boreDiameter",flared ? 10.2 : 0).setToken("role","column mounting anchor — nominal M12 tap drill")),
				pose:F.translation(x,y-l.baseCentreY,l.railPlane-l.insertThickness)});
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
		}
		if (xyAxis != null) {
			var a = xyAxis, bayFront = a.yFixedY-a.yFixedDepth/2-a.clearance;
   if(a.supplierY) {
    // The shorter selected motor moves the bracket rearward. Keep a real
    // shaft passage through the web separating the Z channel and rear bay.
    var tunnelFront=l.columnFront-1,tunnelBack=bayFront+1;
    columnVoids.push(CastingFeatures.box("selected-y-shaft-passage",22,tunnelBack-tunnelFront,22,
     0,(tunnelFront+tunnelBack)/2-l.columnCentreY,a.yScrewZ-l.railPlane-11));
   }
			columnVoids.push(CastingFeatures.box("rear-y-motor-bay", a.yBayWidth, a.yServiceRear+1-bayFront,
				a.yBayTop-l.railPlane+1, 0, (bayFront+a.yServiceRear+1)/2-l.columnCentreY, -1));
			for (sx in [-1,1]) for (z in [a.yHatchLowerZ,a.yHatchUpperZ]) columnInserts.push({
				id:'hatchRear${sx < 0 ? "Left" : "Right"}${z == a.yHatchLowerZ ? "Lower" : "Upper"}',
				part:new CastInsert(new ComponentValues().setNumber("width",20).setNumber("depth",20).setNumber("height",10)
					.setNumber("boreDiameter",3.3).setToken("role","rear motor hatch anchor")),
				pose:F.fromRotationMatrix(sx*75,a.yServiceRear-8-l.columnCentreY,z-l.railPlane,[1,0,0, 0,0,1, 0,-1,0])});
		}
		var chamberHeight = (l.columnHeight - 2 * l.roof - l.wall) / 2;
		// Rear-open chambers leave a thick front web, side walls, sole, top and
		// cross rib. This avoids prescribing inaccessible sealed internal cores.
		for (row in 0...2) {
			var start = l.roof + row*(chamberHeight+l.wall), top = start+chamberHeight;
			if (row == 0 && xyAxis != null) start = Math.max(start,xyAxis.yBayTop-l.railPlane+l.roof);
			columnVoids.push(CastingFeatures.box('rear-chamber-$row',l.columnWidth-2*l.wall,l.columnDepth-frontWeb+1,
				top-start,0,(frontWeb+1)/2,start));
		}

  // One physical strip per side carries the guide and both bearing-plate anchors.
  var stripStart=zAxis==null?l.zRailStart:0.0;
  var stripEnd=l.zRailStart+l.zRailLength+(zAxis==null?0:20);
  var stripCentre=(stripStart+stripEnd)/2;
		for (side in [-1, 1]) columnInserts.push(insert(side < 0 ? "railZLeft" : "railZRight",
			l.seatWidth, stripEnd-stripStart, l.insertThickness, zAxis==null?"Z guide seat":"continuous Z guide and bearing mounting strip",
			F.fromRotationMatrix(side * l.zRailSpacing / 2, -l.columnDepth / 2 + l.insertThickness - l.seatRise,
				stripCentre, [1,0,0, 0,0,-1, 0,1,0])));
		for (sx in [-1, 1]) for (sy in [-1, 1]) columnInserts.push(insert('foot${corner(sx, sy)}',
			footWidth, footDepth, l.insertThickness, "column sole pad",
			down(sx * (flared ? (l.columnWidth+l.columnFootWidth)/4 : l.columnWidth/2-l.wall), sy * (l.columnDepth / 2 - l.roof), l.insertThickness)));
		if (flared) for (row in columnInserts) if (StringTools.startsWith(row.id,"foot")) {
			var side=row.pose.x<0?-1:1, inner=Math.abs(row.pose.x)-20, outer=l.columnFootWidth/2+1;
			columnVoids.push(CastingFeatures.box('bolt-access-${row.id}',outer-inner,46,
				MillFrameLayout.COLUMN_ACCESS_TOP-l.insertThickness,side*(inner+outer)/2,row.pose.y,l.insertThickness));
		}
		if (flared) {
			columnInserts = [for (row in columnInserts) StringTools.startsWith(row.id,"foot") ? {
				id:row.id,pose:row.pose,part:new CastInsert(new ComponentValues().setNumber("width",footWidth).setNumber("depth",footDepth)
					.setNumber("height",l.insertThickness).setNumber("boreDiameter",13).setToken("role","column sole clamping pad"))} : row];
		}
		var columnValues = body(l.columnWidth,l.columnDepth,l.columnHeight,columnVoids);
		if (flared) columnValues.setNumber("footWidth",l.columnFootWidth).setNumber("footHeight",MillFrameLayout.COLUMN_FOOT_HEIGHT).setNumber("footShoulder",MillFrameLayout.COLUMN_FOOT_SHOULDER);
		column = new MineralFoundation(columnValues, columnInserts, "footLeftFront");
  for(side in ["Left","Right"]){
   var id='railZ$side',top=Solids.axial(0,0,l.insertThickness);
   column.addMemberConnector(id,"railSeat",F.compose(F.translation(0,l.zRailStart+l.zRailLength/2-stripCentre,0),top));
   if(zAxis!=null)for(end in ["Lower","Upper"]){
    var centre=end=="Lower"?10.0:l.zRailStart+l.zRailLength+10;
    var region=F.translation(0,centre-stripCentre,0);
    column.addMemberConnector(id,'bearing$end',region);
    column.addMemberConnector(id,'bearing${end}Face',F.compose(region,top));
   }
  }
		include("column", column, F.translation(0, l.columnCentreY, l.railPlane));
		// Mating frames coincide; the physical finished faces oppose each other.
		var first = columnInserts.filter(row -> row.id == "footLeftFront")[0];
		addMemberConnector("column/footLeftFront", "baseSeat", F.compose(F.inverse(first.pose),
			Solids.axial(first.pose.x, first.pose.y, 0)));
		addMate("column-on-base", "fixed", "base/columnLeftFront", "top", "column/footLeftFront", "baseSeat");
		if (flared) for (sx in [-1,1]) for (sy in [-1,1]) {
			var id = corner(sx,sy), anchor = columnInserts.filter(row -> row.id == 'foot$id')[0];
			var bolt = 'columnBolt$id';
			addComponent(bolt,machinekit.standard.SocketHeadCapScrew.metric("M12",35));
			addMemberConnector('column/foot$id',"boltSeat",F.compose(F.inverse(anchor.pose),F.translation(anchor.pose.x,anchor.pose.y,l.insertThickness)));
			addMemberConnector(bolt,"seat",F.identity());
			addMate('column-clamp-$id',"fixed",'column/foot$id',"boltSeat",bolt,"seat");
		}
		exposeConnector("mount", "base/casting", "mount");
		for (id in ["railYLeft", "railYRight"]) exposeConnector(id, 'base/$id', "top");
		for (id in ["railZLeft", "railZRight"]) exposeConnector(id, 'column/$id', "railSeat");
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
