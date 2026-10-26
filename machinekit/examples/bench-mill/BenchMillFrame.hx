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
		var stripCentre=sizing.stripCentre;
		base = new MineralFoundation(sizing.baseValues, baseInserts);
		if (l.footPockets)
			for (sx in [-1, 1]) for (sy in [-1, 1]) {
				var id = corner(sx, sy);
				base.addComponent('foot$id', machinekit.structural.LevelingFoot.of(
					Plate(details.footPlateSide, details.footPlateSide, details.footPlateThickness,
						details.levelingClearanceDiameter)),
					F.translation(sx * (l.baseWidth / 2 - l.wall),
						(sy < 0 ? l.front + details.footEndSetback : l.rear - details.footEndSetback) - l.baseCentreY,
						details.footRoof - details.footPlateThickness));
			}
		include("base", base, F.translation(0, l.baseCentreY, 0));
		if (l.footPockets)
			for (sx in [-1, 1])
				for (sy in [-1, 1]) {
					var id = corner(sx, sy);
					addComponent('levelingFoot$id', machinekit.structural.LevelingFoot.of(details.levelingFoot));
					addMate('leveling-foot-$id', "fixed", 'base/foot$id', "mount", 'levelingFoot$id', "mount");
					var thread = details.levelingThread;
					addComponent('footSupportNut$id', machinekit.structural.LevelingFoot.of(
						Nut(Hexagon(details.footNutAcrossFlats), details.footSupportNutHeight, thread)));
					addMate('foot-support-$id', "fixed", 'base/foot$id', "mount", 'footSupportNut$id', "top");
					addComponent('footLockNut$id', machinekit.structural.LevelingFoot.of(
						Nut(Hexagon(details.footNutAcrossFlats), details.footLockNutHeight, thread)));
					addMate('foot-lock-$id', "fixed", 'footSupportNut$id', "mount", 'footLockNut$id', "top");
				}

		column = new MineralFoundation(sizing.columnValues, columnInserts, "footLeftFront", sizing.columnPostCure);
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
		include("column", column, F.translation(0, l.columnCentreY, l.columnMountPlane));
		// Mating frames coincide; the physical finished faces oppose each other.
		var first = columnInserts.filter(row -> row.id == "footLeftFront")[0];
		addMemberConnector("column/footLeftFront", "baseSeat", F.compose(F.inverse(first.pose), Solids.axial(first.pose.x, first.pose.y, -l.columnJointGapMm)));
		addMate("column-on-base", "fixed", "base/columnLeftFront", "top", "column/footLeftFront", "baseSeat");
			for (sx in [-1, 1])
				for (sy in [-1, 1]) {
					var id = corner(sx, sy),
						anchor = columnInserts.filter(row -> row.id == 'foot$id')[0];
					var bolt = 'columnBolt$id';
					addComponent(bolt, machinekit.standard.SocketHeadCapScrew.metric(details.columnAnchorThread.thread.designation,
						details.columnBoltLengthIncrement * Math.ceil((l.insertThickness+l.columnJointGapMm
							+ machinekit.design.ThreadedHolePolicy.resolve(details.columnAnchorThread.thread, C45Bright).minEngagement)
							/ details.columnBoltLengthIncrement),
						machinekit.standard.PropertyClass.Steel88));
					addMemberConnector('column/foot$id', "boltSeat",
						F.compose(F.inverse(anchor.pose), F.translation(anchor.pose.x, anchor.pose.y, l.insertThickness)));
					addMemberConnector(bolt, "seat", F.identity());
					addMate('column-clamp-$id', "fixed", 'column/foot$id', "boltSeat", bolt, "seat");
				}
		if (l.columnJointGapMm>0) {
			var joint:materia.casting.FoundationJoint={id:"base-column",lowerBody:"base/casting",upperBody:"column/casting",
				route:Grouted,gapMm:l.columnJointGapMm,gapToleranceMm:0.1,seats:[],jacks:[],anchors:[],cureBeforeTorque:true};
			for (sx in [-1,1]) for (sy in [-1,1]) {
				var id=corner(sx,sy),lower="base/column"+id,upper="column/foot"+id;
				joint.seats.push({id:id,lower:{member:lower,feature:"joint-seat"},upper:{member:upper,feature:"joint-seat"}});
				joint.anchors.push({id:id,screw:"columnBolt"+id,target:{member:lower,feature:"fixing"},
					through:[{member:upper,feature:"fixing"}]});
				if (sx>0 && sy>0) continue;
				var row=columnInserts.filter(row -> row.id=="foot"+id)[0];
				var hole=row.part.insertSpec().holes.filter(hole -> hole.id=="jack")[0];
				var point=F.transformPoint(row.pose,hole.frame.x,hole.frame.y,hole.frame.z);
				var length=details.columnBoltLengthIncrement*Math.ceil((l.insertThickness+l.columnJointGapMm)/details.columnBoltLengthIncrement);
				var screw="columnJack"+id;
				addComponent(screw,machinekit.standard.SocketSetScrew.forThread(details.jointJackThread.thread.mating(),length),F.translation(point.x,l.columnCentreY+point.y,l.railPlane+length));
				addMemberConnector(upper,"jackSeat",F.compose(F.inverse(row.pose),
					F.translation(point.x,point.y,length-l.columnJointGapMm)));
				addMemberConnector(screw,"nominalSeat",F.identity());
				addMate("column-jack-"+id,"fixed",upper,"jackSeat",screw,"nominalSeat");
				joint.jacks.push({id:id,screw:screw,receiver:{member:upper,feature:"jack"},bearing:{member:lower,feature:"joint-seat"}});
			}
			addDesignRequirement(machinekit.design.DesignRequirement.FoundationConnection("base-column-joint",joint));
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
	public final columnPostCure:Array<machinekit.structural.MineralFoundation.PostCureInsertPlacement>;

	public function new(layout:MillFrameLayout, details:MillCastingDetails, ?zAxis:MillZAxisLayout, ?xyAxis:MillXYStageLayout, ?sharedDiameter:Float) {
		if (sharedDiameter==null) {
			var sizes=details.anchorRule.allowedNominalDiametersMm.copy(),failures:Array<String>=[];
			sizes.sort(Reflect.compare);
			for (size in sizes) try {
				var resolved=new MillCastingSizing(layout,details,zAxis,xyAxis,size);
				baseValues=resolved.baseValues;baseInserts=resolved.baseInserts;
				columnValues=resolved.columnValues;columnInserts=resolved.columnInserts;
				columnPostCure=resolved.columnPostCure;stripCentre=resolved.stripCentre;
				return;
			} catch (error:Dynamic) {
				if (Std.isOfType(error,machinekit.tooling.InsertAnchorGeometry.InsertAnchorScreenDefect)) throw error;
				failures.push(Std.string(size)+" mm: "+Std.string(error));
			}
			throw "No declared common mill anchor thread fits the source: "+failures.join("; ");
		}
		var anchorRule=machinekit.structural.FoundationSizing.singleThreadRule(details.anchorRule,sharedDiameter);
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
				F.translation(side * l.yRailSpacing / 2, l.yRailStart + l.yRailLength / 2 - l.baseCentreY, l.railPlane - l.insertThickness),details.precisionFinishingStockMm));
		if (xyAxis != null)
			for (end in ["Fixed", "Floating"]) {
				var fixed = end == "Fixed",
					y = fixed ? xyAxis.yFixedY : xyAxis.yFloatingY;
				baseInserts.push(insert('driveY$end', details.driveBearingSeatWidth, fixed ? xyAxis.yFixedDepth : xyAxis.yFloatingDepth, l.insertThickness, BearingSeat,
					"Y drive bearing seat", [CastingPatterns.feature(fixed ? "rear-y-drive-seat" : "front-y-bearing-seat")], F.translation(0, y - l.baseCentreY, xyAxis.yMountPlane - l.insertThickness),details.precisionFinishingStockMm));
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
		var clampX = l.columnWidth / 2 - l.wall;
		if (l.columnFootWidth > l.columnWidth) clampX = (l.columnWidth + l.columnFootWidth) / 4;
		var footWidth = Math.max(l.wall,details.minimumClearancePadSide(details.columnAnchorThread.clearanceDiameter,details.columnAnchorThickness)),
			footDepth = Math.max(l.roof, details.minimumColumnPadDepth(l.insertThickness,anchorRule));
		var padPrototype=new CastInsert(CastInsert.precastHoles(
			CastInsert.boxValues(footWidth,footDepth,details.columnAnchorThickness),
			InsertHoles.tappedBlind(details.columnAnchorThread,details.columnAnchorThickness))
			.setToken("role",InsertRole.ColumnMount).setToken("label","column mounting anchor"));
		footWidth=Math.max(footWidth,Math.ceil(machinekit.tooling.InsertAnchorGeometry.minimumAnchorBandWidth(
			solePad(footWidth,footDepth,l.insertThickness,details),anchorRule)));
		var maximumPadWidth=l.columnFootWidth-2*clampX;
		footWidth=machinekit.structural.FoundationSizing.anchorPadWidth("column-joint-pad",padPrototype,
			footWidth,footDepth,details.columnAnchorThickness,maximumPadWidth,details.columnBoltLengthIncrement,anchorRule);
		for (sx in [-1, 1])
			for (sy in [-1, 1]) {
				var id = corner(sx, sy),
					x = sx * clampX,
					y = l.columnCentreY + sy * (l.columnDepth / 2 - l.roof);
				baseInserts.push({
					id: 'column$id',
					openings: [CastingPatterns.face("body.z-max")],
					part: jointSeat(new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(footWidth, footDepth, details.columnAnchorThickness),
						InsertHoles.tappedBlind(details.columnAnchorThread, details.columnAnchorThickness))
						.setToken("role", InsertRole.ColumnMount)
						.setToken("label", "column mounting anchor")), details.columnAnchorThickness),
					pose: F.translation(x, y - l.baseCentreY, l.railPlane - details.columnAnchorThickness)
				});
				if (l.footPockets) {
					var fx = sx * (l.baseWidth / 2 - l.wall),
						fy = (sy < 0 ? l.front + details.footEndSetback : l.rear - details.footEndSetback) - l.baseCentreY;
					var foot = CastingFeaturePatterns.foot({access: 'foot-access-$id', stem: 'foot-stem-$id', register: null},
						fx, fy, sx * l.baseWidth / 2, true, 2 * l.wall, details.footAccessDepth,
						details.footAccessFloor, details.footRoof, details.footStemSize, details.footStemTop,
						[CastingPatterns.face(sx < 0 ? "body.x-min" : "body.x-max")],
						[CastingPatterns.face("body.z-min")]);
					for (feature in foot) baseVoids.push(feature);
				} else
					baseInserts.push(insert('foot$id', l.wall, l.wall, l.insertThickness, LevelingSeat, "foundation foot seat",
						[CastingPatterns.face("body.z-min"),CastingPatterns.face(sx<0 ? "body.x-min" : "body.x-max"),
							CastingPatterns.face(sy<0 ? "body.y-min" : "body.y-max")],
						down(sx * (l.baseWidth / 2 - l.wall / 2), (sy < 0 ? l.front + l.wall / 2 : l.rear - l.wall / 2) - l.baseCentreY, l.insertThickness)));
			}
		if (xyAxis!=null) baseInserts=mountRails(baseInserts,MillRailPlacement.y(xyAxis.guided()),
			F.translation(0,l.baseCentreY,0),"xyStage",details);
		baseInserts=baseInserts.concat(MillLiftingPattern.placements(body(l.baseWidth,l.baseDepth,l.baseHeight,baseVoids,"bench-mill-base"),baseInserts,details,{width:l.baseWidth,depth:l.baseDepth},anchorRule));
		baseInserts = machinekit.structural.FoundationSizing.anchors(baseInserts, anchorRule, body(l.baseWidth, l.baseDepth, l.baseHeight, baseVoids,"bench-mill-base"));
		this.baseValues=body(l.baseWidth,l.baseDepth,l.baseHeight,baseVoids,"bench-mill-base");
		this.baseInserts=baseInserts;

		var columnVoids:Array<CastingFeature> = [],
			columnInserts:Array<CastInsertPlacement> = [];
		var frontWeb = zAxis == null ? l.roof : Math.max(l.roof, zAxis.channelBack + l.wall - (l.columnCentreY - l.columnDepth / 2));
		if (zAxis != null) {
			var channelFront = l.columnCentreY - l.columnDepth / 2;
			columnVoids.push(CastingFeaturePatterns.box("z-drive-channel", ServiceOpening,
				[CastingPatterns.face("body.y-min"), CastingPatterns.face("body.z-max"), CastingPatterns.face("body.z-min")],
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
				var tunnelFront = zAxis == null ? l.columnFront : zAxis.channelBack, tunnelBack = bayFront;
				var shaftOpenings = zAxis == null
					? [CastingPatterns.face("body.y-min"), CastingPatterns.feature("rear-y-motor-bay")]
					: [CastingPatterns.feature("z-drive-channel"), CastingPatterns.feature("rear-y-motor-bay")];
				columnVoids.push(CastingFeaturePatterns.box("selected-y-shaft-passage",ServiceOpening, shaftOpenings, details.selectedShaftClearance, tunnelBack
					- tunnelFront, details.selectedShaftClearance, 0,
					(tunnelFront + tunnelBack) / 2
					- l.columnCentreY, a.yScrewZ
					- l.columnMountPlane
					- details.selectedShaftClearance / 2));
			}
			var rearBay = CastingFeaturePatterns.box("rear-y-motor-bay", ServiceOpening,
				[CastingPatterns.face("body.y-max"), CastingPatterns.face("body.z-min")],
				a.yBayWidth, a.yServiceRear
				+ details.rearBayDesignClearance
				- bayFront, a.yBayTop
				- l.columnMountPlane, 0,
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
					[CastingPatterns.face("body.y-max"),
					 CastingPatterns.face("body.z-min")],
					a.yBayWidth, back - notchBack, a.yBayTop - l.columnMountPlane,
					0, (notchBack + back) / 2, 0);
				for (side in [-1, 1])
					columnVoids.push(CastingFeaturePatterns.box("rear-y-motor-cheek-" + (side < 0 ? "left" : "right"),
						ServiceOpening, [CastingPatterns.face("body.z-min"),
							CastingPatterns.feature("rear-y-motor-bay"), CastingPatterns.feature("z-drive-channel")],
						half - notch, notchBack - front, a.yBayTop - l.columnMountPlane,
						side * (half + notch) / 2, (front + notchBack) / 2, 0));
			}
			columnVoids.push(rearBay);

		}
		var chamberHeight = (l.columnHeight - 2 * l.roof - l.wall) / 2;
		// Rear-open chambers leave a thick front web, side walls, sole, top and
		// cross rib. This avoids prescribing inaccessible sealed internal cores.
		for (row in 0...2) {
			var start = l.roof + row * (chamberHeight + l.wall),
				top = start + chamberHeight;
			if (row==0) {
				start=Math.max(start,MillFrameLayout.COLUMN_ACCESS_TOP+l.roof);
				if (xyAxis!=null) start=Math.max(start,xyAxis.yBayTop-l.columnMountPlane+l.roof);
			}
			if (top <= start || frontWeb >= l.columnDepth)
				throw "Column backing consumes rear chamber: rear-chamber-" + row;
			var chamberOpenings = [CastingPatterns.face("body.y-max")];
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
					.concat(stripStart == 0 ? [CastingPatterns.face("body.z-min")] : []),
				F.fromRotationMatrix(side * l.zRailSpacing / 2, -l.columnDepth / 2 + l.insertThickness , stripCentre,
					[1, 0, 0, 0, 0, -1, 0, 1, 0]),details.precisionFinishingStockMm));
		for (sx in [-1,1]) for (sy in [-1,1]) {
			var id="foot"+corner(sx,sy);
			var part=solePad(footWidth,footDepth,l.insertThickness,details);
			var pose=down(sx*clampX,sy*(l.columnDepth/2-l.roof),l.insertThickness);
			if (sy>0) pose=F.compose(pose,F.fromRotationMatrix(0,0,0,[-1,0,0,0,-1,0,0,0,1]));
			columnInserts.push({id:id,part:part,pose:pose,openings:[CastingPatterns.face("body.z-min")]});
			var inner=clampX-details.columnBoltAccessHalfWidth,outer=l.columnFootWidth/2;
			columnVoids.push(CastingFeaturePatterns.box("bolt-access-"+id,ServiceOpening,
				[CastingPatterns.face(sx<0 ? "body.x-min" : "body.x-max")],outer-inner,details.columnBoltAccessDepth,
				MillFrameLayout.COLUMN_ACCESS_TOP-l.insertThickness,sx*(inner+outer)/2,pose.y,l.insertThickness));
		}

		// A wide source blank and two explicit side clearances form the step.
		// Extending the rounded cuts across their declared open faces leaves
		// only the two re-entrant radii on the finished casting.
		if (l.columnFootWidth > l.columnWidth) for (side in [-1, 1]) {
			var radius = details.columnFootCornerRadiusMm;
			var width = (l.columnFootWidth - l.columnWidth) / 2 + radius;
			var height = l.columnHeight - MillFrameLayout.COLUMN_FOOT_HEIGHT + radius;
			var feature = CastingFeaturePatterns.box("column-step-clearance-" + (side < 0 ? "left" : "right"),
				Clearance, [CastingPatterns.face(side < 0 ? "body.x-min" : "body.x-max"),
					CastingPatterns.face("body.y-min"), CastingPatterns.face("body.y-max"), CastingPatterns.face("body.z-max")],
				width, height, l.columnDepth);
			feature.geometry.pose = F.fromRotationMatrix(
				side * (l.columnWidth + l.columnFootWidth + 2 * radius) / 4, l.columnDepth / 2,
				(MillFrameLayout.COLUMN_FOOT_HEIGHT + l.columnHeight + radius) / 2,
				[1, 0, 0, 0, 0, -1, 0, 1, 0]);
			feature.cornerRadius = radius;
			columnVoids.push(feature);
		}
		var columnValues = body(l.columnFootWidth, l.columnDepth, l.columnHeight, columnVoids,"bench-mill-column");
		var postCure:Array<machinekit.structural.MineralFoundation.PostCureInsertPlacement>=[];
		var bonds:Array<materia.casting.PostCureBond>=[];
		if (xyAxis!=null) for (side in [-1,1]) for (upper in [false,true]) {
			var z=(upper ? xyAxis.yHatchUpperZ : xyAxis.yHatchLowerZ)-l.columnMountPlane;
			var id="hatchRear"+(side<0 ? "Left" : "Right")+(upper ? "Upper" : "Lower");
			var face=xyAxis.yServiceRear+details.hatchAnchorServiceFaceOffset-l.columnCentreY;
			var part=new CastInsert(CastInsert.precastHoles(CastInsert.manufacturedValues(C45Bright)
				.setToken("body",materia.casting.InsertSpec.InsertBodyCodec.encode(
					Primitive(Cylinder(details.hatchInsertDiameter,details.hatchAnchorThickness)))),
				InsertHoles.tappedBlind(details.hatchAnchorThread,details.hatchAnchorThickness))
				.setToken("role",InsertRole.HatchMount).setToken("label","post-cure bonded hatch insert")
				.setToken("finish",materia.casting.InsertSpec.InsertFinish.AsSupplied));
			var pose=F.fromRotationMatrix(side*details.hatchAnchorSideOffset,
				face-details.hatchAnchorThickness,z,[1,0,0,0,0,1,0,-1,0]);
			postCure.push({id:id,part:part,pose:pose});
			var receiver=InsertHoles.records(InsertHoles.through(details.hatchReceiverDiameter,details.hatchReceiverDepth))[0];
			receiver.id=id+"-receiver";
			receiver.extent=materia.casting.InsertHole.HoleExtent.Blind;
			receiver.limits={policyId:"mill-hatch-bonded-receiver-v1",threaded:null,
				minWall:details.hatchReceiverDiameter*details.hatchReceiverWallDiameters,
				minBack:details.hatchReceiverDiameter*details.hatchReceiverBackDiameters,
				minClearance:details.hatchReceiverDiameter*details.hatchReceiverWallDiameters,overrides:[]};
			receiver.frame=F.fromRotationMatrix(side*details.hatchAnchorSideOffset,l.columnDepth/2,z,
				[1,0,0,0,0,-1,0,1,0]);
			bonds.push({insertId:id,receiver:receiver,
				adhesive:materia.casting.CastingInput.CastingTextInput.Unresolved("Post-cure steel-to-compound bonding adhesive and coupon")});
		}
		columnValues.setToken("postCureBonds",haxe.Json.stringify(materia.casting.PostCureBond.PostCureBonds.records(bonds)));
		this.columnPostCure=postCure;
		if (zAxis!=null) columnInserts=mountRails(columnInserts,MillRailPlacement.z(zAxis),
			F.translation(0,l.columnCentreY,l.columnMountPlane),"zAxis",details);
		columnInserts=columnInserts.concat(MillLiftingPattern.placements(columnValues,columnInserts,details,{width:l.columnWidth,depth:l.columnDepth},anchorRule));
		columnInserts = machinekit.structural.FoundationSizing.anchors(columnInserts, anchorRule, columnValues);
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

	static function solePad(width:Float,depth:Float,thickness:Float,details:MillCastingDetails):CastInsert {
		var holes=InsertHoles.records(InsertHoles.through(details.columnAnchorThread.clearanceDiameter,thickness));
		var fixture=InsertHoles.records(InsertHoles.tappedBlind(details.jointFixtureThread,thickness))[0];
		fixture.id="fixture";fixture.frame.y=-details.jointHolePitchMm;holes.push(fixture);
		var jack=InsertHoles.records(InsertHoles.tappedThrough(details.jointJackThread,thickness))[0];
		jack.id="jack";jack.frame=F.translation(0,details.jointHolePitchMm,0);
		holes.push(jack);
		return jointSeat(new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(width,depth,thickness),
			InsertHoles.fromNeutral(holes)).setToken("role",InsertRole.ColumnMount).setToken("label","column sole clamping pad")),thickness);
	}

	static function jointSeat(part:CastInsert,height:Float):CastInsert {
		var requirement:materia.casting.InterfaceRequirement={id:"joint-seat",
			geometry:materia.casting.InterfaceRequirement.PrecisionGeometry.Plane(F.translation(0,0,height)),
			route:materia.casting.InterfaceRequirement.InterfaceRoute.AsSupplied,allowanceMm:0,
			finishingProcessErrorMm:materia.casting.CastingInput.Unresolved("RM0 joint seat and metrology allocation"),
			permittedFinishes:[materia.casting.InterfaceRequirement.InterfaceFinish.AsSupplied],
			positionToleranceMm:part.parameterNumber("positionTolerance"),datumReference:"primary",relationshipIds:[]};
		return new CastInsert(part.values().copy().setToken("finish",materia.casting.InsertSpec.InsertFinish.AsSupplied)
			.setToken("interfaces",haxe.Json.stringify(materia.casting.InterfaceRequirement.InterfaceRequirements.records([requirement]))));
	}

	static function insert(id:String, width:Float, depth:Float, thickness:Float, role:materia.casting.InsertSpec.InsertRole, label:String, openings:Array<CastingOpening>,
		pose:materia.assembly.AssemblyRecord.AssemblyFrame,finishingStock:Float=0):CastInsertPlacement {
		var part=new CastInsert(CastInsert.boxValues(width,depth,thickness+finishingStock)
			.setToken("role",role).setToken("label",label));
		if (finishingStock>0) {
			var requirement:materia.casting.InterfaceRequirement={id:"finished-seat",
				geometry:materia.casting.InterfaceRequirement.PrecisionGeometry.Plane(F.translation(0,0,thickness)),
				route:materia.casting.InterfaceRequirement.InterfaceRoute.Ground,allowanceMm:finishingStock,
				finishingProcessErrorMm:materia.casting.CastingInput.Unresolved("RM0 finishing-process error allocation"),
				permittedFinishes:[materia.casting.InterfaceRequirement.InterfaceFinish.Ground],
				positionToleranceMm:part.parameterNumber("positionTolerance"),datumReference:"primary",relationshipIds:[]};
			part=new CastInsert(part.values().copy().setToken("interfaces",haxe.Json.stringify(
				materia.casting.InterfaceRequirement.InterfaceRequirements.records([requirement]))));
		}
		return {id:id,part:part,pose:pose,openings:openings};
	}

	static function body(width:Float, depth:Float, height:Float, voids:Array<CastingFeature>,foundationId:String):ComponentValues {
		var scheme:materia.casting.FoundationDatumScheme={id:foundationId+"-datums",foundationId:foundationId,
			primary:{id:"primary",reference:BodyFace("body.z-max"),frame:F.translation(0,0,height)},
			secondary:{id:"secondary",reference:BodyFace("body.x-min"),
				frame:F.fromRotationMatrix(-width/2,0,0,[0,0,-1,1,0,0,0,-1,0])},
			tertiary:{id:"tertiary",reference:BodyFace("body.y-min"),
				frame:F.fromRotationMatrix(0,-depth/2,0,[1,0,0,0,0,-1,0,1,0])}};
		return new ComponentValues().setNumber("width", width).setNumber("depth", depth).setNumber("height", height)
			.setToken("features", CastingPatterns.encode(voids)).setToken("datumScheme",haxe.Json.stringify(
				materia.casting.FoundationDatumScheme.FoundationDatums.record(scheme)));
	}
}
