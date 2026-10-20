package machinekit.structural;

import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineComponent;
import machinekit.assembly.AxisBuilder;
import machinekit.structural.TSlotTableParts.TableAngle;
import machinekit.structural.TSlotTableParts.TableSlotNut;
import machinekit.structural.TSlotTableParts.TableTop;
import machinekit.structural.TSlotTableParts.TableExtrusion;
import machinekit.structural.TSlotTableParts.TableRearBrace;
import machinekit.structural.TSlotTableParts.TableFootPlate;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.HexNut;
import machinekit.structural.FrameAssembly.CutListLine;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Outer dimensions of the low base rails. Offsets are relative to the top's centre.
 * The footprint must contain the entire upper frame; extra depth/width can be asymmetric. */
typedef TSlotTableFootprint = {
	var width:Float;
	var depth:Float;
	@:optional var offsetX:Float;
	@:optional var offsetY:Float;
}

typedef TableBox = {var id:String; var frame:AssemblyFrame; var width:Float; var depth:Float; var height:Float;}

typedef TableTopMount = {var x:Float; var y:Float; var clearance:Float; var counterbore:Float; var depth:Float;}

typedef TableMachining = {var member:String; var operation:String; var diameter:Float; var depth:Float; var thread:String;}

/** Floor-standing, square-profile extrusion table. Dimensions are mm, top height includes feet
 * and worktop. Front inset moves the entire front support row toward the back. Connections are derived from an explicit
 * hardware specification; optional rear bracing is selected by the caller, not load-rated. */
class TSlotTable extends MachineAssembly {
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	public final profile:TSlotExtrusion;
	public final topThickness:Float;
	public final frontInset:Float;
	public final footHeight:Float;
	public final lowerRailHeight:Float;
	public final hardware:TSlotTableHardware;
	public final rearBrace:Bool;
	public final hasOutriggers:Bool;
	public final baseWidth:Float;
	public final baseDepth:Float;
	public final baseOffsetX:Float;
	public final baseOffsetY:Float;
	public final postBottom:Float;
	final mounts:Array<TableTopMount> = [];
	final operations:Array<TableMachining> = [];
	final bracketScrew:SocketHeadCapScrew;
	final slotNut:TableSlotNut;
	final frame:FrameAssembly = new FrameAssembly();
	final names:Array<String> = [];
	final boxes:Array<TableBox> = [];
	final groups:Map<String,MachineAssembly> = [];

	public function new(width:Float, depth:Float, height:Float, profileName:String = "HFS5-4040",
			topThickness:Float = 10, frontInset:Float = 0, footHeight:Float = 30, lowerRailHeight:Float = 220, ?hardware:TSlotTableHardware, rearBrace:Bool = false, ?footprint:TSlotTableFootprint) {
		super();
		profile = TSlotExtrusion.forProfile(profileName);
		var s = profile.size;
		this.hardware = hardware == null ? TSlotTableHardware.forProfile(profile) : hardware;
		this.hardware.validate(profile);
		hardware = this.hardware;
		this.rearBrace = rearBrace;
		bracketScrew = SocketHeadCapScrew.metric(hardware.boltSize,hardware.bracketScrewLength(profile));
		slotNut = new TableSlotNut(hardware);
		for (v in [width, depth, height, topThickness, footHeight, lowerRailHeight])
			if (!Math.isFinite(v) || v <= 0) throw "Table dimensions must be finite and positive";
		if (profile.height != s) throw "Table requires a square extrusion profile";
		if (!Math.isFinite(frontInset) || frontInset < 0 || width <= 4 * s + hardware.screwSpec().counterboreDiameter + 2 || depth - frontInset <= 4 * s ||
				lowerRailHeight - 1.5 * s <= footHeight || lowerRailHeight + 2.5 * s >= height - topThickness)
			throw "Table needs room for legs, rails, brackets and feet";
		this.width = width; this.depth = depth; this.height = height;
		this.topThickness = topThickness; this.frontInset = frontInset;
		this.footHeight = footHeight; this.lowerRailHeight = lowerRailHeight;
		hasOutriggers = footprint != null;
		baseWidth = footprint == null ? width : footprint.width;
		baseDepth = footprint == null ? depth-frontInset : footprint.depth;
		baseOffsetX = footprint == null || footprint.offsetX == null ? 0 : footprint.offsetX;
		baseOffsetY = footprint == null ? frontInset/2 : footprint.offsetY == null ? 0 : footprint.offsetY;
		for (v in [baseWidth,baseDepth,baseOffsetX,baseOffsetY])
			if (!Math.isFinite(v)) throw "Base footprint dimensions and offsets must be finite";
		if (baseWidth <= 0 || baseDepth <= 0 || baseOffsetX-baseWidth/2 > -width/2+1e-8 ||
				baseOffsetX+baseWidth/2 < width/2-1e-8 ||
				baseOffsetY-baseDepth/2 > -depth/2+frontInset+1e-8 ||
				baseOffsetY+baseDepth/2 < depth/2-1e-8)
			throw "Base footprint must contain the upper frame";
		postBottom = hasOutriggers ? footHeight+hardware.footEngagement+2+s : footHeight;
		if (hasOutriggers && lowerRailHeight-1.5*s <= postBottom+s)
			throw "Lower rails need clearance above the base and post mounting angles";
		var locknut = HexNut.metric(hardware.footThread);
		if (locknut.acrossFlats/Math.cos(Math.PI/6) >= s || footHeight <= 8 + locknut.height + 2 || height-topThickness-footHeight <= hardware.footEngagement+2)
			throw "Foot needs exposed adjustment space, a locknut and sufficient post engagement";
		var topScrewLength = hardware.topScrewLength(profile,topThickness);
		var recess = topThickness + profile.modeledSlotDepth() - TSlotTableHardware.TIP_CLEARANCE - topScrewLength;
		if (recess > topThickness-2 || profile.size * 0.7 <= hardware.screwSpec().counterboreDiameter)
			throw "Top counterbore does not fit plate or profile";
		var x = width / 2 - s / 2;
		var front = -depth / 2 + s / 2 + frontInset, back = depth / 2 - s / 2;
		var top = height - topThickness;
		var upper = top - s / 2;
		for (ix in [-1, 1]) for (iy in [-1, 1]) {
			var px=ix*x, py=iy<0 ? front : back, id='leg-$ix-$iy';
			member(id,px,py,postBottom,px,py,top,!hasOutriggers);
			if (!hasOutriggers) {
				operations.push({member:id,operation:"drill and tap from bottom",diameter:hardware.footSpec().tapDrill,
					depth:hardware.footEngagement+2,thread:hardware.footThread});
				addFoot(ix,iy,px,py,locknut);
			}
		}
		if (hasOutriggers) buildBase(x,front,back,locknut);

		for (level in [{id: "upper", z: upper}, {id: "lower", z: lowerRailHeight}]) {
			for (iy in [-1, 1]) {
				var py = iy < 0 ? front : back;
				member('${level.id}-cross-$iy', -x + s / 2, py, level.z, x - s / 2, py, level.z);
			}
			for (ix in [-1, 1])
				member('${level.id}-side-$ix', ix * x, front + s / 2, level.z, ix * x, back - s / 2, level.z);
			// Each angle's local X points from the leg toward the rail, local Z is up.
			for (ix in [-1, 1]) for (iy in [-1, 1]) for (axis in 0...2) {
				var px=ix*x, py=iy<0 ? front : back;
				var alongX=axis==0 ? -ix : 0, alongY=axis==1 ? -iy : 0;
				var pose=AxisBuilder.orient(px+alongX*s/2,py+alongY*s/2,level.z-s/2,
					[-alongY,alongX,0.0],[0.0,0.0,1.0]);
				angle('${level.id}-bracket-$ix-$iy-$axis',pose);

			}

		}
		var topScrew=SocketHeadCapScrew.metric(hardware.boltSize,topScrewLength);
		for (iy in [-1,1]) for (ix in [-1,1]) {
			var mx=ix*(width/2-2*s), my=iy<0 ? front : back;
			mounts.push({x:mx,y:my,clearance:topScrew.spec.clearanceMedium,counterbore:topScrew.spec.counterboreDiameter,depth:recess});
			fasten('top-mount-$ix-$iy',AssemblyFrames.translation(mx,my,height-recess),topScrew);
		}
		addPart("top", new TableTop(width,depth,topThickness,mounts), AssemblyFrames.translation(0, 0, top));
		box("top", width, depth, topThickness, 0, 0, top);
		if (rearBrace) {
			var low=lowerRailHeight+1.5*s, high=upper-1.5*s;
			var dz=high-low, length=Math.sqrt(4*x*x+dz*dz), w=s*0.7, t=hardware.bracketThickness;
			if (dz<=2*s) throw "Rear brace needs more height between the rail levels";
			var pose=AxisBuilder.orient(-x,back+s/2,low,[2*x/length,0.0,dz/length],[0.0,1.0,0.0]);
			addPart("rear-brace",new TableRearBrace(length,w,t,hardware.screwSpec().clearanceMedium),pose);
			localBox("rear-brace",w,length+w,t,pose,0,length/2,0);
			for (end in [0,1]) {
				var bearing=AssemblyFrames.compose(pose,AssemblyFrames.translation(0,end*length,t));
				fasten('rear-brace-$end',bearing,null,[0.0,0.0,1.0]);
			}
		}
		finishGroups();
	}

	/** Resolve a manufacturing member name to its stable assembly occurrence path. */
	public function partPath(id:String):String return groupPath(id)+"/"+partName(id);

	function partName(id:String):String {
		for (prefix in ["upper-bracket-","lower-bracket-","post-base-","top-mount-","foot-plate-","foot-locknut-","foot-","leg-"])
			if (StringTools.startsWith(id,prefix)) {
				var corner=~/^(-?1)-(-?1)(.*)$/;
				if (corner.match(id.substr(prefix.length))) return prefix+(corner.matched(1)=="-1" ? "left" : "right")+"-"+
					(corner.matched(2)=="-1" ? "front" : "rear")+corner.matched(3);
			}
		for (prefix in ["base-side-","upper-side-","lower-side-","base-joint-"]) if (StringTools.startsWith(id,prefix)) {
			if (StringTools.endsWith(id,"--1")) return id.substr(0,id.length-3)+"-left";
			if (StringTools.endsWith(id,"-1")) return id.substr(0,id.length-2)+"-right";
		}
		for (prefix in ["upper-cross-","lower-cross-"]) if (StringTools.startsWith(id,prefix))
			return prefix+(id.substr(prefix.length)=="-1" ? "front" : "rear");
		return id;
	}

	function groupPath(id:String):String return [for (segment in logicalGroupPath(id).split("/")) partName(segment)].join("/");

	function logicalGroupPath(id:String):String {
		if (StringTools.startsWith(id,"foot-")) {
			var corner=~/^foot-(?:plate-|locknut-)?(-?1)-(-?1)(?:-.*)?$/;
			if (corner.match(id)) return "feet/"+(corner.matched(1)=="-1" ? "left" : "right")+"-"+
				(corner.matched(2)=="-1" ? "front" : "rear");
		}
		if (id=="top" || StringTools.startsWith(id,"top-mount-")) return "worktop";
		if (StringTools.startsWith(id,"rear-brace")) return "bracing";
		var group=StringTools.startsWith(id,"base-") || StringTools.startsWith(id,"post-base-") ? "base" :
			StringTools.startsWith(id,"lower-") ? "lower-frame" : "upper-frame";
		var connection=~/^(.*(?:bracket-.*|joint-.*|post-base-.*?))-(?:leg|rail)-(?:screw|nut)$/;
		if (connection.match(id)) return group+"/connections/"+connection.matched(1);
		if (id.indexOf("bracket-")>=0 || id.indexOf("joint-")>=0 || StringTools.startsWith(id,"post-base-"))
			return group+"/connections/"+id;
		return group;
	}

	function addPart(id:String,component:MachineComponent,?pose:AssemblyFrame):Void {
		var path=groupPath(id);
		if (!groups.exists(path)) groups.set(path,new MachineAssembly());
		groups.get(path).addComponent(partName(id),component,pose);
	}

	function finishGroups():Void {
		var paths=[for (path in groups.keys()) path];
		for (path in paths) {
			var parent=path;
			while (parent.lastIndexOf("/")>=0) {
				parent=parent.substr(0,parent.lastIndexOf("/"));
				if (!groups.exists(parent)) groups.set(parent,new MachineAssembly());
			}
		}
		paths=[for (path in groups.keys()) path];
		paths.sort((a,b) -> a.split("/").length==b.split("/").length ? Reflect.compare(a,b) : b.split("/").length-a.split("/").length);
		for (path in paths) {
			var slash=path.lastIndexOf("/");
			if (slash<0) include(path,groups.get(path));
			else groups.get(path.substr(0,slash)).include(path.substr(slash+1),groups.get(path));
		}
	}

	function addFoot(ix:Int,iy:Int,x:Float,y:Float,locknut:HexNut):Void {
		addPart('foot-$ix-$iy',LevelingFoot.of(Foot(profile.size,8,
			machinekit.standard.FastenerThreads.metric(hardware.footSpec().diameter,hardware.footSpec().pitch),
			7,footHeight+hardware.footEngagement,footHeight,ThreadRoot,NoCollar)),
			AssemblyFrames.translation(x,y,0));
		box('foot-$ix-$iy',profile.size,profile.size,footHeight+hardware.footEngagement,x,y,0);
		addPart('foot-locknut-$ix-$iy',locknut,AssemblyFrames.translation(x,y,footHeight-locknut.height));
		box('foot-locknut-$ix-$iy',locknut.acrossFlats/Math.cos(Math.PI/6),locknut.acrossFlats/Math.cos(Math.PI/6),
			locknut.height,x,y,footHeight-locknut.height);
	}

	/** Low frame with independent foot locations; cross members directly support every post row. */
	function buildBase(postX:Float,front:Float,back:Float,locknut:HexNut):Void {
		var s=profile.size, plateThickness=hardware.footEngagement+2;
		var railBottom=footHeight+plateThickness, railZ=railBottom+s/2;
		var left=baseOffsetX-baseWidth/2+s/2, right=baseOffsetX+baseWidth/2-s/2;
		var minY=baseOffsetY-baseDepth/2, maxY=baseOffsetY+baseDepth/2;
		var rows=[{id:"support-front",y:front},{id:"support-back",y:back}];
		// A short side-rail overhang needs no extra overlapping cross member.
		if (front-(minY+s/2) >= 3*s+1) rows.push({id:"end-front",y:minY+s/2});
		if (maxY-s/2-back >= 3*s+1) rows.push({id:"end-back",y:maxY-s/2});
		for (ix in [-1,1]) {
			var x=ix<0 ? left : right;
			member('base-side-$ix',x,minY,railZ,x,maxY,railZ);
		}
		for (row in rows) {
			member('base-cross-${row.id}',left+s/2,row.y,railZ,right-s/2,row.y,railZ);
			for (ix in [-1,1]) {
				// A normal corner angle rotated into the horizontal plane joins both rails' side slots.
				var dx=-ix, dy=row.id=="end-front" || row.id=="support-front" ? 1 : -1;
				var pose=AxisBuilder.orient((ix<0 ? left : right)+dx*s/2,row.y+dy*s/2,railZ,
					[0.0,0.0,-1.0*ix*dy],[0.0,-1.0*dy,0.0]);
				angle('base-joint-${row.id}-$ix',pose);
			}
		}
		// Post feet bear on the cross rails; inverted angles clamp each post to its rail.
		for (ix in [-1,1]) for (iy in [-1,1]) {
			var dx=-ix;
			angle('post-base-$ix-$iy',AxisBuilder.orient(ix*postX+dx*s/2,iy<0 ? front : back,postBottom,
				[0.0,-1.0*dx,0.0],[0.0,0.0,-1.0]));
		}
		var length=hardware.topScrewLength(profile,plateThickness);
		var recess=plateThickness+profile.modeledSlotDepth()-TSlotTableHardware.TIP_CLEARANCE-length;
		var screw=SocketHeadCapScrew.metric(hardware.boltSize,length);
		for (ix in [-1,1]) for (iy in [-1,1]) {
			var x=ix<0 ? left : right, y=iy<0 ? minY+s : maxY-s;
			var id='foot-plate-$ix-$iy';
			addPart(id,new TableFootPlate(s,hardware,recess),AssemblyFrames.translation(x,y,footHeight));
			box(id,s,2*s,plateThickness,x,y,footHeight);
			operations.push({member:id,operation:"drill and tap foot adapter through",diameter:hardware.footSpec().tapDrill,
				depth:plateThickness,thread:hardware.footThread});
			for (side in [-1,1])
				fasten('$id-mount-$side',AxisBuilder.orient(x,y+side*s/2,footHeight+recess,
					[0.0,-1.0,0.0],[0.0,0.0,-1.0]),screw,[0.0,1.0,0.0]);
			addFoot(ix,iy,x,y,locknut);
		}
	}

	function angle(id:String,pose:AssemblyFrame):Void {
		var s=profile.size,t=hardware.bracketThickness,w=s*0.7;
		addPart(id,new TableAngle(s,t,w,hardware.screwSpec().clearanceMedium),pose);
		localBox(id+"-horizontal",s,w,t,pose,s/2,0,-t);
		localBox(id+"-vertical",t,w,s,pose,t/2,0,-s);
		fasten(id+"-leg",AssemblyFrames.compose(pose,AxisBuilder.orient(t,0,-s/2,[0.0,1.0,0.0],[1.0,0.0,0.0])));
		fasten(id+"-rail",AssemblyFrames.compose(pose,AxisBuilder.orient(s/2,0,-t,[0.0,-1.0,0.0],[0.0,0.0,-1.0])));
	}

	public function topMounts():Array<TableTopMount> return [for (m in mounts)
		{x:m.x,y:m.y,clearance:m.clearance,counterbore:m.counterbore,depth:m.depth}];
	/** Available extension about the nominal pose, limited by the drilled bore and 1.5D engagement.
	 * Changing an assembled foot extension also changes that corner's top height. */
	public function footAdjustment():{nominal:Float, minimum:Float, maximum:Float} {
		var locknut=HexNut.catalog().get(hardware.footThread);
		return {nominal:footHeight,minimum:Math.max(8+locknut.height+2,footHeight-2),
			maximum:footHeight+hardware.footEngagement-1.5*hardware.footSpec().diameter};
	}

	public function machining():Array<TableMachining> return [for (m in operations)
		{member:m.member,operation:m.operation,diameter:m.diameter,depth:m.depth,thread:m.thread}];

	/** Driver approach is normal to each bearing face. Slot plates are loaded from open rail ends. */
	function fasten(id:String, bearing:AssemblyFrame, ?screw:SocketHeadCapScrew, ?nutAlong:Array<Float>):Void {
		if (screw==null) screw=bracketScrew;
		addPart(id+"-screw",screw,bearing);
		localBox(id+"-screw",screw.spec.headDiameter,screw.spec.headDiameter,screw.length+screw.spec.headHeight,
			bearing,0,0,-screw.length);
		// Screw enters the extrusion's face at a depth `length-slotDepth+tipClearance` from the head bearing.
		var nutPose=AssemblyFrames.compose(bearing,AxisBuilder.orient(0,0,-screw.length+profile.modeledSlotDepth()-TSlotTableHardware.TIP_CLEARANCE-hardware.nutDepth(profile),
			[0.0,1.0,0.0],[0.0,0.0,-1.0]));
		if (nutAlong != null) {
			var inward=AssemblyFrames.transformVector(bearing,0,0,-1);
			var across=[inward.y*nutAlong[2]-inward.z*nutAlong[1],inward.z*nutAlong[0]-inward.x*nutAlong[2],
				inward.x*nutAlong[1]-inward.y*nutAlong[0]];
			nutPose=AxisBuilder.orient(nutPose.x,nutPose.y,nutPose.z,across,[inward.x,inward.y,inward.z]);
		}
		addPart(id+"-nut",slotNut,nutPose);
		localBox(id+"-nut",hardware.nutLength,hardware.nutWidth,hardware.nutThickness,nutPose,0,0,0);
	}

	function localBox(id:String,w:Float,d:Float,h:Float,pose:AssemblyFrame,x:Float,y:Float,z:Float):Void
		boxes.push({id:id,frame:AssemblyFrames.compose(pose,AssemblyFrames.translation(x,y,z)),width:w,depth:d,height:h});

	public function cutList():Array<CutListLine> return frame.cutList();
	public function memberCuts():Array<{name:String, length:Float}>
		return [for (name in names) {name: name, length: frame.cutLength(name)}];

	/** Conservative individual envelopes; slot cavities and foot-pad corners are filled. */
	public function collisionPieces():Array<{id:String, frame:AssemblyFrame, vertices:Array<Float>}> return [for (b in boxes) {
		var vertices:Array<Float> = [];
		for (x in [-b.width / 2, b.width / 2]) for (y in [-b.depth / 2, b.depth / 2]) for (z in [0.0, b.height]) {
			vertices.push(x); vertices.push(y); vertices.push(z);
		}
		{id: collisionPath(b.id), frame: AssemblyFrames.compose(AssemblyFrames.translation(0,0,0),b.frame), vertices: vertices};
	}];

	function collisionPath(id:String):String {
		for (suffix in ["-horizontal","-vertical"]) if (StringTools.endsWith(id,suffix))
			return partPath(id.substr(0,id.length-suffix.length))+suffix;
		return partPath(id);
	}

	function member(id:String, x0:Float, y0:Float, z0:Float, x1:Float, y1:Float, z1:Float, foot:Bool=false):Void {
		frame.point(id + "-start", x0, y0, z0); frame.point(id + "-end", x1, y1, z1);
		frame.member(id, id + "-start", id + "-end", profile); names.push(id);
		var length = frame.length(id);
		var along = [(x1-x0)/length, (y1-y0)/length, (z1-z0)/length];
		addPart(id, new TableExtrusion(profile, length,foot ? hardware : null), AxisBuilder.orient(x0,y0,z0,
			z0 == z1 ? [0.0,0.0,1.0] : [0.0,1.0,0.0], along));
		var s = profile.size;
		box(id, Math.abs(x1-x0) + (x0 == x1 ? s : 0), Math.abs(y1-y0) + (y0 == y1 ? s : 0),
			Math.abs(z1-z0) + (z0 == z1 ? s : 0), (x0+x1)/2, (y0+y1)/2, Math.min(z0,z1) - (z0 == z1 ? s/2 : 0));
	}
	function box(id:String, w:Float, d:Float, h:Float, x:Float, y:Float, z:Float):Void
		boxes.push({id:id, frame:AssemblyFrames.translation(x,y,z), width:w, depth:d, height:h});
}
