import machinekit.milling.MillFrame;
import machinekit.milling.MillFrameLayout;
import machinekit.milling.MillZAxisLayout;
import machinekit.milling.MillXYStageLayout;
import machinekit.milling.MillRailPlacement;

import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import machinekit.structural.CastInsert;
import materia.casting.InsertSpec.InsertRole;
import materia.casting.CastingPatterns;
import machinekit.structural.CastingFeaturePatterns;
import machinekit.structural.InsertHoles;
import materia.casting.CastingFeature;
import machinekit.structural.MineralFoundation;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import materia.assembly.AssemblyFrames as F;

/** Example-owned casting recipe and source construction. The library frame
 * consumes the authored foundations and keeps their machine-facing datums. */
class BenchMillFrame extends MillFrame {
	public final casting:MillCastingDetails;

	public static function create(layout:MillFrameLayout,?zAxis:MillZAxisLayout,?xyAxis:MillXYStageLayout,
		?casting:MillCastingDetails):BenchMillFrame {
		var chosen=casting==null ? new MillCastingDetails() : casting;
		var sizing=new MillCastingSizing(layout,chosen,zAxis,xyAxis);
		var source=new MillCastingSource(layout,chosen,sizing,zAxis,xyAxis);
		return new BenchMillFrame(layout,source,chosen,zAxis,xyAxis);
	}

	function new(layout:MillFrameLayout,source:MillCastingSource,casting:MillCastingDetails,
		zAxis:Null<MillZAxisLayout>,xyAxis:Null<MillXYStageLayout>) {
		super(layout,source.base,source.column,source,zAxis,xyAxis);
		this.casting=casting;
	}
}

/** Realizes the declared footprints, holes and pockets returned by sizing. */
private class MillCastingSource extends MachineAssembly {
	public final base:MineralFoundation;
	public final column:MineralFoundation;

	public function new(l:MillFrameLayout, details:MillCastingDetails, sizing:MillCastingSizing,
		?zAxis:MillZAxisLayout, ?xyAxis:MillXYStageLayout) {
		super();
		var baseInserts=sizing.baseInserts, columnInserts=sizing.columnInserts;
		var stripCentre=sizing.stripCentre, flared=sizing.flared;
		base = new MineralFoundation(sizing.baseValues, baseInserts);
		include("base", base, F.translation(0, l.baseCentreY, 0));
		if (l.footPockets)
			for (sx in [-1, 1])
				for (sy in [-1, 1]) {
					var id = corner(sx, sy);
					addComponent('levelingFoot$id', machinekit.structural.LevelingFoot.of(details.levelingFoot));
					addMate('leveling-foot-$id', "fixed", 'base/foot$id', "top", 'levelingFoot$id', "mount");
				}

		column = new MineralFoundation(sizing.columnValues, columnInserts, "footLeftFront");
		for (side in ["Left", "Right"]) {
			var id = 'railZ$side', top = Solids.axial(0, 0, l.insertThickness);
			column.addMemberConnector(id, "railSeat", F.compose(F.translation(0, l.zRailStart + l.zRailLength / 2 - stripCentre, 0), top));
			if (zAxis != null)
				for (end in ["Lower", "Upper"]) {
					var centre = end == "Lower" ? details.zBearingAnchorOffset : l.zRailStart + l.zRailLength + details.zBearingAnchorOffset;
					var region = F.translation(0, centre - stripCentre, 0);
					column.addMemberConnector(id, 'bearing$end', region);
					column.addMemberConnector(id, 'bearing${end}Face', F.compose(region, top));
				}
		}
		include("column", column, F.translation(0, l.columnCentreY, l.railPlane));
		// Mating frames coincide; the physical finished faces oppose each other.
		var first = columnInserts.filter(row -> row.id == "footLeftFront")[0];
		addMemberConnector("column/footLeftFront", "baseSeat", F.compose(F.inverse(first.pose), Solids.axial(first.pose.x, first.pose.y, 0)));
		addMate("column-on-base", "fixed", "base/columnLeftFront", "top", "column/footLeftFront", "baseSeat");
		if (flared)
			for (sx in [-1, 1])
				for (sy in [-1, 1]) {
					var id = corner(sx, sy),
						anchor = columnInserts.filter(row -> row.id == 'foot$id')[0];
					var bolt = 'columnBolt$id';
					addComponent(bolt, machinekit.standard.SocketHeadCapScrew.metric(details.columnAnchorThread.thread.designation,
						details.columnBoltLengthIncrement * Math.ceil((l.insertThickness
							+ machinekit.design.ThreadedHolePolicy.resolve(details.columnAnchorThread.thread, C45Bright).minEngagement)
							/ details.columnBoltLengthIncrement),
						machinekit.standard.PropertyClass.Steel88));
					addMemberConnector('column/foot$id', "boltSeat",
						F.compose(F.inverse(anchor.pose), F.translation(anchor.pose.x, anchor.pose.y, l.insertThickness)));
					addMemberConnector(bolt, "seat", F.identity());
					addMate('column-clamp-$id', "fixed", 'column/foot$id', "boltSeat", bolt, "seat");
				}
		exposeConnector("mount", "base/casting", "mount");
		for (id in ["railYLeft", "railYRight"])
			exposeConnector(id, 'base/$id', "top");
		for (id in ["railZLeft", "railZRight"])
			exposeConnector(id, 'column/$id', "railSeat");
		for (sx in [-1, 1])
			for (sy in [-1, 1])
				exposeConnector('foot${corner(sx, sy)}', 'base/foot${corner(sx, sy)}', "top");
	}

	static function corner(x:Int,y:Int):String
		return (x<0 ? "Left" : "Right")+(y<0 ? "Front" : "Rear");
}

/** One structural sizing pass, before constructing the source foundations.
 * Searches resolve design dimensions here; workshop evidence is never assumed. */
private class MillCastingSizing {
	public final baseValues:ComponentValues;
	public final baseInserts:Array<CastInsertPlacement>;
	public final columnValues:ComponentValues;
	public final columnInserts:Array<CastInsertPlacement>;
	public final stripCentre:Float;
	public final flared:Bool;

	public function new(layout:MillFrameLayout, details:MillCastingDetails, ?zAxis:MillZAxisLayout, ?xyAxis:MillXYStageLayout) {
		if (layout == null)
			throw "Mill frame needs a layout";
		if (zAxis != null && zAxis.frame != layout)
			throw "Frame and Z datums must share one layout";
		if (xyAxis != null && xyAxis.frame != layout)
			throw "Frame and XY datums must share one layout";
		var l = layout;
		var baseVoids:Array<CastingFeature> = [], baseInserts:Array<CastInsertPlacement> = [];
		var pocketFront = l.front + (l.footPockets ? details.undersidePocketSetback : l.wall),
			pocketRear = l.columnFront - l.wall;
		var pocketLength = (pocketRear - pocketFront - l.wall) / 2;
		// Six authored pockets leave the two full-height rail beams in place.
		// Their walls are explicit source stations, not subtraction fragments.
		var pockets=[
			{name:"left",low:-l.baseWidth/2+l.wall,high:-(l.yRailSpacing+l.seatWidth)/2},
			{name:"centre",low:-(l.yRailSpacing-l.seatWidth)/2,high:(l.yRailSpacing-l.seatWidth)/2},
			{name:"right",low:(l.yRailSpacing+l.seatWidth)/2,high:l.baseWidth/2-l.wall}
		];
		for (row in 0...2)for(pocket in pockets)
			baseVoids.push(CastingFeaturePatterns.underside('underside-row-$row-${pocket.name}',pocket.high-pocket.low,
				pocketLength,l.baseHeight-l.roof,(pocket.low+pocket.high)/2,
				pocketFront+pocketLength/2+row*(pocketLength+l.wall)-l.baseCentreY));
		if (xyAxis != null)
			for (side in [-1, 1]) {
				var shoulderOuter = (l.yRailSpacing + l.seatWidth) / 2 + details.shoulderClearance;
				var width = l.baseWidth / 2 - shoulderOuter;
				baseVoids.push(CastingFeaturePatterns.box('guide-shoulder-outside-$side', InterfaceRecess,
					[CastingPatterns.face(side < 0 ? "body.x-min" : "body.x-max"), CastingPatterns.face("body.z-max")],
					width, l.yRailLength, details.shoulderDepth, side * (shoulderOuter + width / 2),
					l.yRailStart
					+ l.yRailLength / 2
					- l.baseCentreY, l.baseHeight
					- details.shoulderDepth));
			}
		if (xyAxis != null) {
			var start = xyAxis.yFloatingY + xyAxis.yFloatingDepth / 2 + details.shoulderClearance,
				end = l.yRailStart + l.yRailLength;
			baseVoids.push(CastingFeaturePatterns.box("between-y-shoulders",InterfaceRecess, [CastingPatterns.face("body.z-max")], l.yRailSpacing
				- l.seatWidth
				- 2 * details.shoulderClearance, end
				- start, details.shoulderDepth, 0,
				(start + end) / 2
				- l.baseCentreY, l.baseHeight
				- details.shoulderDepth));
		}
		for (side in [-1, 1])
			baseInserts.push(insert(side < 0 ? "railYLeft" : "railYRight", l.seatWidth, l.yRailLength, l.insertThickness, GuideSeat, "Y guide seat", [CastingPatterns.face("body.z-max")],
				F.translation(side * l.yRailSpacing / 2, l.yRailStart + l.yRailLength / 2 - l.baseCentreY, l.railPlane - l.insertThickness)));
		if (xyAxis != null)
			for (end in ["Fixed", "Floating"]) {
				var fixed = end == "Fixed",
					y = fixed ? xyAxis.yFixedY : xyAxis.yFloatingY;
				baseInserts.push(insert('driveY$end', details.driveBearingSeatWidth, fixed ? xyAxis.yFixedDepth : xyAxis.yFloatingDepth, l.insertThickness, BearingSeat,
					"Y drive bearing seat", [CastingPatterns.feature(fixed ? "rear-y-drive-seat" : "front-y-bearing-seat")], F.translation(0, y - l.baseCentreY, xyAxis.yMountPlane - l.insertThickness)));
			}
		if (xyAxis != null) {
			var a = xyAxis;
			var bayFront = a.yFixedY - a.yFixedDepth / 2 - a.clearance;
			// Only shallow bearing/motor-foot recesses remain in the base.
			baseVoids.push(CastingFeaturePatterns.box("rear-y-drive-seat",InterfaceRecess, [CastingPatterns.face("body.z-max")],
				a.yBayWidth, a.yServiceRear
				+ details.rearBayDesignClearance
				- bayFront, l.baseHeight
				- a.yMountPlane, 0,
				(bayFront + a.yServiceRear + details.rearBayDesignClearance) / 2
				- l.baseCentreY, a.yMountPlane));
			baseVoids.push(CastingFeaturePatterns.box("front-y-bearing-seat",InterfaceRecess, [CastingPatterns.face("body.z-max")], details.frontBearingSeatWidth, a.yFloatingDepth
				+ 2 * a.clearance, l.baseHeight
				- a.yMountPlane, 0,
				a.yFloatingY
				- l.baseCentreY, a.yMountPlane));
		}

		// Four independent steel pairs provide a broad column mounting interface.
		var flared = l.columnFootWidth > l.columnWidth;
		this.flared=flared;
		var footWidth = flared ? Math.max(l.wall,details.minimumClearancePadSide(details.columnAnchorThread.clearanceDiameter,details.columnAnchorThickness)) : l.wall,
			footDepth = Math.max(l.roof, details.minimumColumnPadDepth(l.insertThickness));
		for (sx in [-1, 1])
			for (sy in [-1, 1]) {
				var id = corner(sx, sy),
					x = sx * (flared ? (l.columnWidth + l.columnFootWidth) / 4 : l.columnWidth / 2 - l.wall),
					y = l.columnCentreY + sy * (l.columnDepth / 2 - l.roof);
				baseInserts.push({
					id: 'column$id',
					openings: [CastingPatterns.face("body.z-max")],
					part: new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(footWidth, footDepth, details.columnAnchorThickness),
						flared ? InsertHoles.tappedBlind(details.columnAnchorThread, details.columnAnchorThickness) : InsertHoles.fromNeutral([]))
						.setToken("role", InsertRole.ColumnMount)
						.setToken("label", "column mounting anchor")),
					pose: F.translation(x, y - l.baseCentreY, l.railPlane - details.columnAnchorThickness)
				});
				if (l.footPockets) {
					var fx = sx * (l.baseWidth / 2 - l.wall),
						fy = (sy < 0 ? l.front + details.footEndSetback : l.rear - details.footEndSetback) - l.baseCentreY;
					var foot = CastingFeaturePatterns.foot({access: 'foot-access-$id', stem: 'foot-stem-$id', register: null},
						fx, fy, sx * l.baseWidth / 2, true, 2 * (l.wall + details.footInnerClearance), details.footAccessDepth,
						details.footAccessFloor, details.footAccessFloor + details.footAccessHeight, details.footStemSize, details.footStemTop,
						[CastingPatterns.face(sx < 0 ? "body.x-min" : "body.x-max")], [CastingPatterns.face("body.z-min")]);
					for (feature in foot)
						baseVoids.push(feature);
					baseInserts.push({
						id: 'foot$id',
						openings: [CastingPatterns.feature('foot-access-$id'), CastingPatterns.feature('foot-stem-$id')],
						part: new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(Math.max(l.wall,details.minimumClearancePadSide(details.levelingClearanceDiameter,l.insertThickness)),
							Math.max(l.wall,details.minimumClearancePadSide(details.levelingClearanceDiameter,l.insertThickness)),l.insertThickness),
							InsertHoles.through(details.levelingClearanceDiameter, l.insertThickness))
							.setToken("role", InsertRole.LevelingSeat)
							.setToken("label", "leveling screw seat")),
						pose: F.translation(fx, fy, details.footAccessFloor - l.insertThickness)
					});
				} else
					baseInserts.push(insert('foot$id', l.wall, l.wall, l.insertThickness, LevelingSeat, "foundation foot seat",
						[CastingPatterns.face("body.z-min"),CastingPatterns.face(sx<0 ? "body.x-min" : "body.x-max"),
							CastingPatterns.face(sy<0 ? "body.y-min" : "body.y-max")],
						down(sx * (l.baseWidth / 2 - l.wall / 2), (sy < 0 ? l.front + l.wall / 2 : l.rear - l.wall / 2) - l.baseCentreY, l.insertThickness)));
			}
		if (xyAxis!=null) baseInserts=mountRails(baseInserts,MillRailPlacement.y(xyAxis.guided()),
			F.translation(0,l.baseCentreY,0),"xyStage",details);
		baseInserts=baseInserts.concat(MillLiftingPattern.placements(body(l.baseWidth,l.baseDepth,l.baseHeight,baseVoids),baseInserts,details));
		baseInserts = machinekit.structural.FoundationSizing.anchors(baseInserts, details.anchorRule, body(l.baseWidth, l.baseDepth, l.baseHeight, baseVoids));
		this.baseValues=body(l.baseWidth,l.baseDepth,l.baseHeight,baseVoids);
		this.baseInserts=baseInserts;

		var columnVoids:Array<CastingFeature> = [],
			columnInserts:Array<CastInsertPlacement> = [];
		var frontWeb = zAxis == null ? l.roof : Math.max(l.roof, zAxis.channelBack + l.wall - (l.columnCentreY - l.columnDepth / 2));
		if (zAxis != null) {
			var channelFront = l.columnCentreY - l.columnDepth / 2;
			columnVoids.push(CastingFeaturePatterns.box("z-drive-channel", ServiceOpening,
				[CastingPatterns.face("body.y-min"), CastingPatterns.face("body.z-max"), CastingPatterns.face(flared ? "foot.z-min" : "body.z-min")].concat(flared ? [CastingPatterns.face("foot.y-min")] : []),
				zAxis.channelWidth, zAxis.channelBack
				- channelFront, l.columnHeight, 0,
				(channelFront + zAxis.channelBack) / 2
				- l.columnCentreY, 0));
		}
		if (xyAxis != null) {
			var a = xyAxis, bayFront = a.yFixedY - a.yFixedDepth / 2 - a.clearance;
			if (a.supplierY) {
				// The shorter selected motor moves the bracket rearward. Keep a real
				// shaft passage through the web separating the Z channel and rear bay.
				var tunnelFront = l.columnFront, tunnelBack = bayFront;
				columnVoids.push(CastingFeaturePatterns.box("selected-y-shaft-passage",ServiceOpening, [CastingPatterns.face(flared ? "foot.y-min" : "body.y-min")], details.selectedShaftClearance, tunnelBack
					- tunnelFront, details.selectedShaftClearance, 0,
					(tunnelFront + tunnelBack) / 2
					- l.columnCentreY, a.yScrewZ
					- l.railPlane
					- details.selectedShaftClearance / 2));
			}
			var rearBay = CastingFeaturePatterns.box("rear-y-motor-bay", ServiceOpening,
				[CastingPatterns.face(flared ? "foot.y-max" : "body.y-max"), CastingPatterns.face(flared ? "foot.z-min" : "body.z-min")],
				a.yBayWidth, a.yServiceRear
				+ details.rearBayDesignClearance
				- bayFront, a.yBayTop
				- l.railPlane, 0,
				(bayFront + a.yServiceRear + details.rearBayDesignClearance) / 2
				- l.columnCentreY, 0);
			if (zAxis != null && zAxis.channelBack > bayFront) {
				// Directly author the rear span and two cheeks beside the channel.
				// Their disjoint union is the original motor bay minus the channel.
				var half = a.yBayWidth / 2, notch = zAxis.channelWidth / 2;
				var front = bayFront - l.columnCentreY;
				var back = a.yServiceRear + details.rearBayDesignClearance - l.columnCentreY;
				var notchBack = zAxis.channelBack - l.columnCentreY;
				if (notch >= half || notchBack >= back)
					throw "Z channel must leave the declared rear motor bay connected";
				rearBay = CastingFeaturePatterns.box("rear-y-motor-bay", ServiceOpening,
					[CastingPatterns.face(flared ? "foot.y-max" : "body.y-max"),
					 CastingPatterns.face(flared ? "foot.z-min" : "body.z-min")],
					a.yBayWidth, back - notchBack, a.yBayTop - l.railPlane,
					0, (notchBack + back) / 2, 0);
				for (side in [-1, 1])
					columnVoids.push(CastingFeaturePatterns.box("rear-y-motor-cheek-" + (side < 0 ? "left" : "right"),
						ServiceOpening, [CastingPatterns.face(flared ? "foot.z-min" : "body.z-min"),
							CastingPatterns.feature("rear-y-motor-bay"), CastingPatterns.feature("z-drive-channel")],
						half - notch, notchBack - front, a.yBayTop - l.railPlane,
						side * (half + notch) / 2, (front + notchBack) / 2, 0));
			}
			columnVoids.push(rearBay);
			var hatchWall=details.hatchAnchorThread.thread.majorDiameter/2
				+machinekit.design.ThreadedHolePolicy.resolve(details.hatchAnchorThread.thread,C45Bright).minWall;
			var maximumHatchOffset=details.hatchAnchorSideOffset+details.hatchAnchorWidth/2-hatchWall;
			for (sx in [-1, 1]) {
				var offset=Math.max(details.hatchAnchorSideOffset,a.yBayWidth/2+details.hatchAnchorWidth/2);
				var part:CastInsert=null,settled=false,lastFailure="";
				while (offset<=maximumHatchOffset+1e-8) {
					var holes=InsertHoles.records(InsertHoles.tappedBlind(details.hatchAnchorThread,details.hatchAnchorThickness));
					holes[0].frame.x=sx*(details.hatchAnchorSideOffset-offset);
					part=new CastInsert(CastInsert.precastHoles(
						CastInsert.boxValues(details.hatchAnchorWidth,details.hatchAnchorDepth,details.hatchAnchorThickness),
						InsertHoles.fromNeutral(holes)).setToken("role",InsertRole.HatchMount)
						.setToken("label","rear motor hatch anchor"));
					var land:Float;
					try land=machinekit.tooling.InsertAnchorGeometry.minimumOuterLand("hatch-layout",part,details.anchorRule,0).x
					catch (error:Dynamic) {lastFailure=Std.string(error);offset+=1;continue;}
					var needed=Math.max(details.hatchAnchorSideOffset,a.yBayWidth/2+details.hatchAnchorWidth/2+land);
					if (needed<=offset+1e-8) {settled=true;break;}
					offset=needed;
				}
				if (!settled) throw "No hatch footprint placement retains its fixed service hole and policy wall: "+lastFailure;
				for (z in [a.yHatchLowerZ, a.yHatchUpperZ])
					columnInserts.push({
						id: 'hatchRear${sx < 0 ? "Left" : "Right"}${z == a.yHatchLowerZ ? "Lower" : "Upper"}',
						openings: [CastingPatterns.face(flared ? "foot.y-max" : "body.y-max")]
							.concat(z == a.yHatchLowerZ ? [CastingPatterns.face(flared ? "foot.z-min" : "body.z-min")] : []),
						part: new CastInsert(part.values()),
						pose: F.fromRotationMatrix(sx*offset,
							a.yServiceRear+details.hatchAnchorServiceFaceOffset-details.hatchAnchorThickness-l.columnCentreY,
							z-l.railPlane,[1,0,0,0,0,1,0,-1,0])
					});
			}
		}
		var chamberHeight = (l.columnHeight - 2 * l.roof - l.wall) / 2;
		// Rear-open chambers leave a thick front web, side walls, sole, top and
		// cross rib. This avoids prescribing inaccessible sealed internal cores.
		for (row in 0...2) {
			var start = l.roof + row * (chamberHeight + l.wall),
				top = start + chamberHeight;
			if (row == 0 && xyAxis != null)
				start = Math.max(start, xyAxis.yBayTop - l.railPlane + l.roof);
			if (top <= start || frontWeb >= l.columnDepth)
				throw "Column backing consumes rear chamber: rear-chamber-" + row;
			var chamberOpenings = [CastingPatterns.face("body.y-max")];
			if (flared && start < MillFrameLayout.COLUMN_FOOT_HEIGHT)
				chamberOpenings = chamberOpenings.concat([CastingPatterns.face("foot.y-max")]);
			columnVoids.push(CastingFeaturePatterns.box('rear-chamber-$row', Lightening, chamberOpenings,
				l.columnWidth-2*l.wall,l.columnDepth-frontWeb,top-start,0,frontWeb/2,start));
		}

		// One physical strip per side carries the guide and both bearing-plate anchors.
		var stripStart = zAxis == null ? l.zRailStart : 0.0;
		var stripEnd = l.zRailStart + l.zRailLength + (zAxis == null ? 0 : details.zStripBearingExtension);
		var stripCentre = (stripStart + stripEnd) / 2;
		for (side in [-1, 1])
			columnInserts.push(insert(side < 0 ? "railZLeft" : "railZRight", l.seatWidth, stripEnd - stripStart, l.insertThickness, GuideSeat,
				zAxis == null ? "Z guide seat" : "continuous Z guide and bearing mounting strip",
				[CastingPatterns.face("body.y-min")]
					.concat(stripEnd >= l.columnHeight ? [CastingPatterns.face("body.z-max")] : [])
					.concat(flared && stripStart < MillFrameLayout.COLUMN_FOOT_HEIGHT ? [CastingPatterns.face("foot.y-min")] : [])
					.concat(stripStart == 0 ? [CastingPatterns.face(flared ? "foot.z-min" : "body.z-min")] : []),
				F.fromRotationMatrix(side * l.zRailSpacing / 2, -l.columnDepth / 2 + l.insertThickness - l.seatRise, stripCentre,
					[1, 0, 0, 0, 0, -1, 0, 1, 0])));
		for (sx in [-1, 1])
			for (sy in [-1, 1])
				columnInserts.push(insert('foot${corner(sx, sy)}', footWidth, footDepth, l.insertThickness, ColumnMount, "column sole pad", [CastingPatterns.face(flared ? "foot.z-min" : "body.z-min")],
					down(sx * (flared ? (l.columnWidth + l.columnFootWidth) / 4 : l.columnWidth / 2 - l.wall), sy * (l.columnDepth / 2 - l.roof),
						l.insertThickness)));
		if (flared)
			for (row in columnInserts)
				if (StringTools.startsWith(row.id, "foot")) {
					var side = row.pose.x < 0 ? -1 : 1,
						inner = Math.abs(row.pose.x) - details.columnBoltAccessHalfWidth,
						outer = l.columnFootWidth / 2;
					columnVoids.push(CastingFeaturePatterns.box('bolt-access-${row.id}', ServiceOpening,
						[CastingPatterns.face(side < 0 ? "foot.x-min" : "foot.x-max")],
						outer - inner, details.columnBoltAccessDepth, MillFrameLayout.COLUMN_ACCESS_TOP - l.insertThickness,
						side * (inner + outer) / 2, row.pose.y, l.insertThickness));
				}
		if (flared) {
			columnInserts = [
				for (row in columnInserts)
					StringTools.startsWith(row.id, "foot") ? {
						id: row.id,
						openings: row.openings,
						pose: row.pose,
						part: new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(footWidth, footDepth, l.insertThickness),
							InsertHoles.through(details.columnAnchorThread.clearanceDiameter, l.insertThickness))
							.setToken("role", InsertRole.ColumnMount)
							.setToken("label", "column sole clamping pad"))
					} : row
			];
		}
		var columnValues = body(l.columnWidth, l.columnDepth, l.columnHeight, columnVoids);
		if (flared)
			columnValues.setNumber("footWidth", l.columnFootWidth)
				.setNumber("footHeight", MillFrameLayout.COLUMN_FOOT_HEIGHT)
				.setNumber("footShoulder", MillFrameLayout.COLUMN_FOOT_SHOULDER);
		if (zAxis!=null) columnInserts=mountRails(columnInserts,MillRailPlacement.z(zAxis),
			F.translation(0,l.columnCentreY,l.railPlane),"zAxis",details);
		columnInserts=columnInserts.concat(MillLiftingPattern.placements(columnValues,columnInserts,details));
		columnInserts = machinekit.structural.FoundationSizing.anchors(columnInserts, details.anchorRule, columnValues);
		this.columnValues=columnValues;
		this.columnInserts=columnInserts;
		this.stripCentre=stripCentre;
	}

	static function corner(x:Int, y:Int):String
		return (x < 0 ? "Left" : "Right") + (y < 0 ? "Front" : "Rear");

	static function mountRails(rows:Array<CastInsertPlacement>,rails:Array<MillRailPlacement.MillPlacedRail>,
		bodyPose:materia.assembly.AssemblyRecord.AssemblyFrame,namespace:String,details:MillCastingDetails):Array<CastInsertPlacement> {
		var result=rows.copy();
		for (placed in rails) {
			var matches=[for (index in 0...result.length) if (result[index].id==placed.insertId) index];
			if (matches.length!=1) throw "Placed rail needs one source seat: "+placed.insertId;
			var index=matches[0],row=result[index],hole=placed.rail.mountingHole();
			var fingerprint=MillRailPlacement.fingerprint(placed.rail,placed.memberId);
			var toInsert=F.compose(F.inverse(F.compose(bodyPose,row.pose)),placed.pose);
			var part=machinekit.structural.InsertMountPattern.derive(row.part,placed.rail,
				namespace+"/"+placed.memberId,fingerprint,toInsert,[for (connector in placed.rail.mountingConnectors()) connector.name],
				placed.rail.mountingThread(),hole==null ? null : hole.diameter,hole==null ? null : hole.reference,
				details.permittedRailHoleRoutes,details.railHoleBudget);
			result[index]={id:row.id,pose:row.pose,openings:row.openings,part:part};
		}
		return result;
	}


	static function down(x:Float, y:Float, z:Float):materia.assembly.AssemblyRecord.AssemblyFrame
		return F.fromRotationMatrix(x, y, z, [1, 0, 0, 0, -1, 0, 0, 0, -1]);

	static function insert(id:String, width:Float, depth:Float, thickness:Float, role:materia.casting.InsertSpec.InsertRole, label:String, openings:Array<CastingOpening>,
			pose:materia.assembly.AssemblyRecord.AssemblyFrame):CastInsertPlacement
		return {
			id: id,
			part: new CastInsert(CastInsert.boxValues(width, depth, thickness)
				.setToken("role", role)
				.setToken("label", label)),
			pose: pose,
			openings: openings
		};

	static function body(width:Float, depth:Float, height:Float, voids:Array<CastingFeature>):ComponentValues
		return new ComponentValues().setNumber("width", width)
			.setNumber("depth", depth)
			.setNumber("height", height)
			.setToken("features", CastingPatterns.encode(voids));
}
