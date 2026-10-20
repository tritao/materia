package machinekit.milling;

import machinekit.motion.HgReferenceData;

/** Packaging study derived from working travel, not a scaled production design.
 * Guide dimensions come from the pinned local HG25 catalog. Loads, drive sizing,
 * spindle power and structural performance remain separate engineering decisions.
 */
class MillCapacityLayout {
	public final xTravel:Float;
	public final yTravel:Float;
	public final zTravel:Float;
	public final clearance:Float = 10;
	public final overtravel:Float = 5;
	public final tableWidth:Float;
	public final tableDepth:Float;
	public final tableThickness:Float = 50;
	public final saddleWidth:Float;
	public final saddleDepth:Float;
	public final saddleHeight:Float = 100;
	public final riserHeight:Float = 60;
	public final xBlockSpan:Float = 65;
	public final yBlockSpan:Float = 110;
	public final zBlockSpan:Float = 110;
	public final xRailSpacing:Float;
	public final xRailLength:Float;
	public final spindleReach:Float;
	public final spindleY:Float;
	public final saddleBottom:Float;
	public final xRailPlane:Float;
	public final tableBottom:Float;
	public final tableTop:Float;
	public final noseMin:Float = 150;
	public final noseMax:Float;
	public final fixtureHeight:Float = 100;
	public final payloadTargetKg:Float = 150;
	public final gaugeZ:Float;
	public final headBottom:Float;
	public final headHeight:Float = 170;
	public final headRearY:Float;
	public final headDepth:Float;
	public final headWidth:Float = 300;
	public final carriageWidth:Float = 400;
	public final carriageHeight:Float = 340;
	public final carriageThickness:Float = 30;
	public final carriageCentreZ:Float;
	public final frame:MillFrameLayout;

	public function new(xTravel:Float = 400, yTravel:Float = 300, zTravel:Float = 380) {
		for (v in [xTravel,yTravel,zTravel]) if (!Math.isFinite(v) || v <= 0) throw "Capacity travel must be finite and positive";
		this.xTravel=xTravel; this.yTravel=yTravel; this.zTravel=zTravel;
		var guide=HgReferenceData.spec("HGH25CA"), baseHeight=280.0, railPlane=baseHeight+4;
		tableWidth=xTravel+300; tableDepth=yTravel+100;
		xRailSpacing=tableDepth-140;
		xRailLength=up(xTravel+2*(xBlockSpan+guide.blockLength/2+guide.railEndMargin+overtravel));
		saddleWidth=xRailLength+40;
		saddleDepth=Math.max(360,xRailSpacing+guide.blockWidth+40);
		spindleReach=up(yTravel/2+tableDepth/2+guide.railHeight+clearance+overtravel+10);
		spindleY=-spindleReach;
		saddleBottom=railPlane+guide.blockHeight;
		xRailPlane=saddleBottom+saddleHeight;
		tableBottom=xRailPlane+guide.blockHeight+riserHeight;
		tableTop=tableBottom+tableThickness;
		noseMax=noseMin+zTravel; gaugeZ=tableTop+noseMin;
		headBottom=gaugeZ+Bt30Cartridge.SEAT;
		headRearY=-guide.blockHeight-carriageThickness;
		headDepth=headRearY-spindleY+80;
		carriageCentreZ=headBottom+headHeight/2;
		var zReach=zBlockSpan+guide.blockLength/2+guide.railEndMargin+overtravel;
		var zStart=Math.floor((carriageCentreZ-zReach)/10)*10;
		var zEnd=up(carriageCentreZ+zTravel+zReach);
		var yReach=yBlockSpan+guide.blockLength/2+guide.railEndMargin+overtravel;
		// Reserve 60 mm beyond the saddle for the future front nut/support stack.
		var yStart=Math.floor(Math.min(spindleY-yTravel/2-yReach,spindleY-yTravel/2-saddleDepth/2-60)/10)*10;
		var yEnd=up(spindleY+yTravel/2+yReach);
		frame=new MillFrameLayout({yRailSpacing:360,yRailStart:yStart,yRailLength:yEnd-yStart,
			columnFront:0,columnHeight:up(zEnd-railPlane+20),zRailSpacing:300,zRailStart:zStart-railPlane,zRailLength:zEnd-zStart},
			{baseWidth:Math.max(800,saddleWidth+100),baseHeight:baseHeight,columnWidth:460,columnDepth:400,columnFootWidth:680,
			wall:60,roof:80,frontExtension:40,footPockets:true,
			yTravel:new MillYTravel(yTravel,50,saddleDepth,overtravel)});
		if (tableDepth/2-yTravel/2 < 50 || xRailSpacing < guide.blockWidth+clearance ||
			saddleDepth < 2*yBlockSpan+guide.blockLength+clearance ||
			carriageHeight < 2*zBlockSpan+guide.blockLength+clearance ||
			carriageWidth < frame.zRailSpacing+guide.blockWidth+clearance ||
			spindleY+yTravel/2+tableDepth/2+guide.railHeight+clearance > 0 ||
			noseMin <= fixtureHeight+clearance)
			throw "Capacity study cannot retain guide support, fixture or column clearance";
	}

 public function guidedXY():MillGuidedXYStage.MillGuidedXYLayout return {
  centreY:spindleY,yRailSpacing:frame.yRailSpacing,yRailStart:frame.yRailStart,yRailPlane:frame.railPlane,
  yRail:new machinekit.motion.HgRail("HGR25R",frame.yRailLength),xRail:new machinekit.motion.HgRail("HGR25R",xRailLength),
  block:new machinekit.motion.HgBlock("HGH25CA"),xHalfTravel:xTravel/2,yHalfTravel:yTravel/2,
  xOvertravel:overtravel,yOvertravel:overtravel,xBlockSpan:xBlockSpan,yBlockSpan:yBlockSpan,xRailSpacing:xRailSpacing,
  saddleWidth:saddleWidth,saddleDepth:saddleDepth,saddleHeight:saddleHeight,
  tableWidth:tableWidth,tableDepth:tableDepth,tableThickness:tableThickness,riserHeight:riserHeight,driveInterfaces:false
 };

	public function zLayout():MillZAxisLayout return new MillZAxisLayout(frame,headBottom+zTravel,headHeight,spindleY,
		-zTravel,"HGH25CA",new machinekit.motion.HgBlock("HGH25CA"),null,
		{blockSpan:zBlockSpan,carriageWidth:carriageWidth,carriageHeight:carriageHeight,carriageThickness:carriageThickness,
		 rail:new machinekit.motion.HgRail("HGR25R",frame.zRailLength),headBoltLength:25,overtravelLimit:overtravel,
		 head:new machinekit.component.ComponentValues().setNumber("width",headWidth).setNumber("noseWidth",180)
		 .setNumber("reach",headDepth).setNumber("plate",20).setNumber("wall",16).setNumber("rearPlate",20).setNumber("boltZ",Math.min(80,headHeight/2-20))});

	static function up(v:Float):Float return Math.ceil(v/10)*10;
}
