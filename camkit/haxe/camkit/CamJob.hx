package camkit;

import toolpathkit.path.MoveKind;
import cadkit.Face;
import toolpathkit.tool.Tool;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.GeometryOffset;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.path.Point3;
import toolpathkit.path.ArcFitting;
import toolpathkit.path.Provenance;
import toolpathkit.path.SpindleDirection;
import toolpathkit.setup.Setup;
import toolpathkit.tool.ToolLibrary;

/** 2.5D toolpath producer. All inputs here are metres and metres/second. */
class CamJob {
	public final safeZ:Float;
	public final spindleRpm:Float;

	/**
		How far a cutting move may round the corner into the next one (G64 P),
		in metres; zero stops exactly at every corner. Plunges and ramps always
		stop exactly.
	**/
	public final blendTolerance:Float;

	/**
		How far an arc may stray from the cutting lines it replaces, in metres:
		a circle that the contours polygonized is cut as G2/G3 again (see
		`ArcFitting`). Zero keeps every line.
	**/
	public final arcTolerance:Float;

	var current:Point3;
	var selectedTool:Int = -1;
	var requestedSpindleRpm:Float;
	var selectedSpindleRpm:Float = -1;

	/** The G43 length in effect; moves are programmed at the tool tip. */
	var activeLength:Float = 0.0;

	var operationNumber:Int = 0;
	final spindleSpinUpSeconds:Float;
	final initialRetract:Bool;
	final cutDirection:Null<CamCutDirection>;
	final approachClearance:Float;
	var ops:Array<ToolpathOp> = [];
	var tools:Array<Tool> = [];

	/**
		`safeZ` is the lowest height for rapid traverses over the work. `initial`
		is where the spindle starts, before any tool length applies; it must be
		clear of the work, since the first tool is loaded there.
	**/
	public function new(safeZ:Float, spindleRpm:Float, ?initial:Point3, blendTolerance:Float = 0.0, arcTolerance:Float = 0.0,
			spindleSpinUpSeconds:Float = 0.0, initialRetract:Bool = false, ?cutDirection:CamCutDirection, approachClearance:Float = 0.001) {
		if (!Math.isFinite(safeZ) || !Math.isFinite(spindleRpm) || spindleRpm <= 0.0)
			throw "CAM needs finite safe Z and positive spindle RPM";
		if (!Math.isFinite(blendTolerance) || blendTolerance < 0.0)
			throw "CAM blend tolerance must be finite and not negative";
		this.blendTolerance = blendTolerance;
		if (!Math.isFinite(arcTolerance) || arcTolerance < 0.0)
			throw "CAM arc tolerance must be finite and not negative";
		if (!Math.isFinite(spindleSpinUpSeconds) || spindleSpinUpSeconds < 0)
			throw "Invalid spindle spin-up dwell";
		this.spindleSpinUpSeconds = spindleSpinUpSeconds;
		this.initialRetract = initialRetract;
		this.cutDirection = cutDirection;
		if (!Math.isFinite(approachClearance) || approachClearance <= 0)
			throw "Invalid CAM approach clearance";
		this.approachClearance = approachClearance;
		this.arcTolerance = arcTolerance;
		this.safeZ = safeZ;
		this.spindleRpm = spindleRpm;
		this.requestedSpindleRpm = spindleRpm;
		current = initial == null ? new Point3(0.0, 0.0, 0.0) : initial;
	}

	/** Modal operation RPM; the next operation emits a speed change even when the tool stays loaded. */
	public function withSpindleSpeed(rpm:Float):CamJob {
		if (!Math.isFinite(rpm) || rpm <= 0) throw "CAM operation RPM must be positive";
		requestedSpindleRpm = rpm;
		return this;
	}

	/** Sheet profile with explicit tabs, optional onion/skin pass, tangent leads and profile ramps. */
	public function sheetProfile(contour:CamContour, tool:Tool, bottom:Float, feed:Float,
			settings:camkit.CamSheetProfile.CamSheetProfileSettings):CamJob {
		require(contour, tool, bottom, feed);
		if (settings == null) throw "Sheet profile settings are required";
		var plan = CamSheetProfile.plan(contour,tool,settings);
		var skin:Float = settings.onionSkin == null ? 0.0 : settings.onionSkin;
		if (!Math.isFinite(skin) || skin < 0 || bottom + skin >= contour.z) throw "Invalid onion-skin thickness";
		for (tab in plan.tabs) if (bottom + tab.thickness >= contour.z) throw "Tabs must be thinner than the sheet";
		var step:Float = settings.stepDown == null ? 0.002 : settings.stepDown;
		var levels = depthLevels(contour.z,bottom+skin,step);
		if (skin > 0 && settings.skinPass == true) levels.push(bottom);
		var entryFeed = checkedEntryFeed(feed,settings.plungeFeed);
		var direction = settings.direction == null ? CamCutDirection.Climb : settings.direction;
		if (settings.spindleRpm != null) withSpindleSpeed(settings.spindleRpm);
		var span = nextSpan(settings.featureRef); selectTool(tool,span);
		for (i in 0...levels.length) {
			var level = levels[i], prior = i == 0 ? contour.z : levels[i-1];
			var pass = CamSheetProfile.pass(plan,level,prior,bottom,plan.rampLength > 0);
			var start = GeometryTools.pointAt(pass[0],0);
			var leadIn = plan.leadRadius > 0 ? CamSheetProfile.lead(plan,start.z,direction,false) : null;
			var entry = leadIn == null ? start : GeometryTools.pointAt(leadIn,0);
			rapidToSafeXY(entry.x,entry.y,span);
			rapid(new Point3(entry.x,entry.y,Math.min(safeZ,prior+approachClearance)),span);
			feedTo(entry,entryFeed,span);
			if (leadIn != null) feedGeometry(leadIn,feed,span);
			for (g in pass) sheetGeometry(g,feed,entryFeed,span);
			// A ramp descends over its first run; finish the entire perimeter at this level.
			if (plan.rampLength > 0) for (g in CamSheetProfile.pass(plan,level,level,bottom,false)) sheetGeometry(g,feed,entryFeed,span);
			if (plan.leadRadius > 0) feedGeometry(CamSheetProfile.lead(plan,level,direction,true),feed,span);
			rapid(new Point3(current.x,current.y,safeZ),span);
		}
		return this;
	}

	function sheetGeometry(g:PathGeometry,feed:Float,plungeFeed:Float,span:Provenance):Void {
		var a=GeometryTools.pointAt(g,0),b=GeometryTools.pointAt(g,GeometryTools.length(g));
		var vertical=switch(g){case Line(_,_):Math.abs(a.x-b.x)<1e-12&&Math.abs(a.y-b.y)<1e-12;case _:false;};
		var kind=b.z<a.z-1e-12?(vertical?MoveKind.Plunge:MoveKind.Ramp):MoveKind.Cut;
		ops.push(ToolpathOp.Move(kind,g,vertical?plungeFeed:feed,Math.abs(a.z-b.z)>1e-12?0:blendTolerance,span));
		current=b;
	}

	/** Bidirectional raster surfacing of the declared rectangle; cutter centres reach every edge. */
	public function surface(low:Point3,high:Point3,tool:Tool,depth:Float,feed:Float,stepOver:Float,
			stepDown:Float=0.002,?plungeFeed:Float,?featureRef:String):CamJob {
		if (low==null||high==null||high.x<=low.x||high.y<=low.y||Math.abs(low.z-high.z)>1e-10||
			!Math.isFinite(depth)||depth>=low.z||!Math.isFinite(feed)||feed<=0||!Math.isFinite(stepOver)||
			stepOver<=0||tool==null||stepOver>tool.diameter||safeZ<=low.z) throw "Invalid surfacing rectangle or cutting parameters";
		for(p in [low,high])if(!Math.isFinite(p.x)||!Math.isFinite(p.y)||!Math.isFinite(p.z))throw "Surfacing bounds must be finite";
		checkReach(tool,low.z-depth);
		var entryFeed=checkedEntryFeed(feed,plungeFeed),span=nextSpan(featureRef);
		var levels=depthLevels(low.z,depth,stepDown),rows=Std.int(Math.ceil((high.y-low.y)/stepOver));
		if (rows>100000)throw "Surfacing row limit exceeded";
		selectTool(tool,span);
		for(i in 0...levels.length) {
			var level=levels[i],prior=i==0?low.z:levels[i-1],left=low.x-tool.diameter/2,right=high.x+tool.diameter/2;
			rapidToSafeXY(left,low.y,span);rapid(new Point3(left,low.y,Math.min(safeZ,prior+approachClearance)),span);
			feedTo(new Point3(left,low.y,level),entryFeed,span);
			for(row in 0...rows+1) {
				var y=low.y+(high.y-low.y)*row/rows,x=row%2==0?right:left;
				feedTo(new Point3(current.x,y,level),feed,span);feedTo(new Point3(x,y,level),feed,span);
			}
			rapid(new Point3(current.x,current.y,safeZ),span);
		}
		return this;
	}

	/** Offset closed profile; `outside` and `inside` use the cutter radius. */
	public function profile(contour:CamContour, tool:Tool, depth:Float, feed:Float, ?side:String = "outside", ?stepDown:Float = 0.002,
			?plungeFeed:Null<Float>, ?featureRef:String, ?startDepth:Float):CamJob {
		require(contour, tool, depth, feed);
		var entryFeed = checkedEntryFeed(feed, plungeFeed);
		if (side != "outside" && side != "inside" && side != "on")
			throw 'Unknown CAM profile side "$side"';
		var levels = depthLevels(startDepth == null ? contour.z : startDepth, depth, stepDown);
		if (side != "on")
			offsetPath(contour, tool.diameter * 0.5, levels[0], side == "outside");
		var reliefs = side == "on" ? new Array<Point3>() : CamExactBoundary.reliefCenters(contour, tool.diameter * 0.5, side == "outside");
		var span = nextSpan(featureRef);
		selectTool(tool, span);
		for (index in 0...levels.length) {
			var level = levels[index];
			var prior = index == 0 ? (startDepth == null ? contour.z : startDepth) : levels[index - 1];
			cutReliefs(reliefs, level, entryFeed, span);
			if (side == "on")
				cutLoop(contour, level, feed, entryFeed, span, prior);
			else
				cutOffset(contour, tool.diameter * 0.5, level, feed, entryFeed, span, side == "outside", prior);
		}
		return this;
	}

	/** Profile a planar CAD face, completing each hole before releasing its outer edge. */
	public function profileFace(face:Face, tool:Tool, depth:Float, feed:Float, ?stepDown:Float = 0.002, ?unit:String = "mm",
			?chordToleranceMetres:Float = 0.00005, ?plungeFeed:Null<Float>):CamJob {
		var boundaries = CamContour.fromFaceBoundaries(face, unit, chordToleranceMetres);
		var outer = boundaries[0];
		require(outer, tool, depth, feed);
		checkedEntryFeed(feed, plungeFeed);
		depthLevels(outer.z, depth, stepDown);
		offsetPath(outer, tool.diameter * 0.5, depth, true);
		for (i in 1...boundaries.length) {
			require(boundaries[i], tool, depth, feed);
			depthLevels(boundaries[i].z, depth, stepDown);
			offsetPath(boundaries[i], tool.diameter * 0.5, depth, false);
		}
		for (i in 1...boundaries.length)
			profile(boundaries[i], tool, depth, feed, "inside", stepDown, plungeFeed, faceRef(face));
		profile(outer, tool, depth, feed, "outside", stepDown, plungeFeed, faceRef(face));
		return this;
	}

	/** Clear a pocket with a separate plunge feed and ramps where spans allow. */
	public function pocket(contour:CamContour, tool:Tool, depth:Float, feed:Float, stepOver:Float, ?stepDown:Float = 0.002, ?plungeFeed:Null<Float>,
			?featureRef:String):CamJob {
		require(contour, tool, depth, feed);
		if (!Math.isFinite(stepOver) || stepOver <= 0.0 || stepOver > tool.diameter)
			throw "CAM pocket step-over must be positive and no larger than tool diameter";
		var entryFeed = checkedEntryFeed(feed, plungeFeed);
		var levels = depthLevels(contour.z, depth, stepDown);
		var reliefs = CamExactBoundary.reliefCenters(contour, tool.diameter * 0.5, false);
		if (contour.exactBoundary != null && offsetPath(contour, tool.diameter * 0.5, levels[0], false).length == 0) {
			var span = nextSpan(featureRef);
			selectTool(tool, span);
			for (level in levels)
				cutReliefs(reliefs, level, entryFeed, span);
			return this;
		}
		if (hasConcaveCorner(contour)) {
			var radius = tool.diameter * 0.5;
			var passes = CamPocketPlanner.plan(contour, radius, stepOver);
			offsetPath(contour, radius, levels[0], false);
			var span = nextSpan(featureRef);
			selectTool(tool, span);
			for (index in 0...levels.length) {
				var level = levels[index];
				cutReliefs(reliefs, level, entryFeed, span);
				var prior = index == 0 ? contour.z : levels[index - 1];
				cutPocketPasses(passes, level, prior, contour.z, feed, entryFeed, span, contour, [], radius);
				cutPocketBoundary(contour, radius, level, prior, feed, entryFeed, span, false);
			}
			return this;
		}
		var span = nextSpan(featureRef);
		selectTool(tool, span);
		var inset = tool.diameter * 0.5;
		var rounds = 0;
		var rings:Array<CamContour> = [];
		while (rounds < 10000) {
			var path:CamContour;
			try
				path = contour.inset(inset)
			catch (error:Dynamic) {
				var message = Std.string(error);
				if (message.indexOf("exhausted") >= 0
					|| message.indexOf("zero area") >= 0
					|| message.indexOf("distinct vertices") >= 0
					|| message.indexOf("self-intersects") >= 0)
					break;
				throw error;
			}
			rings.push(path);
			inset += stepOver;
			rounds++;
		}
		if (rounds == 0)
			throw "Tool does not fit inside CAM pocket";
		if (rounds >= 10000)
			throw "CAM pocket exceeds 10000 clearing rings";
		for (index in 0...levels.length) {
			var level = levels[index];
			cutReliefs(reliefs, level, entryFeed, span);
			var prior = index == 0 ? contour.z : levels[index - 1];
			for (ring in rings)
				cutPocketRing(ring, level, prior, contour.z, feed, entryFeed, span);
		}
		return this;
	}

	/** Clear a face around its internal islands, then finish every boundary. */
	public function pocketFace(face:Face, tool:Tool, depth:Float, feed:Float, stepOver:Float, ?stepDown:Float = 0.002, ?unit:String = "mm",
			?chordToleranceMetres:Float = 0.00005, ?plungeFeed:Null<Float>):CamJob {
		var boundaries = CamContour.fromFaceBoundaries(face, unit, chordToleranceMetres);
		return pocketContours(boundaries[0], boundaries.slice(1), tool, depth, feed, stepOver, stepDown, plungeFeed, faceRef(face));
	}

	/** Clear explicit contour space while keeping its islands. All coordinates
	 * are metres in one XY work plane; setup/stock validation remains with Setup.
	 */
	public function pocketContours(outer:CamContour, islands:Array<CamContour>, tool:Tool, depth:Float, feed:Float,
			stepOver:Float, stepDown:Float = 0.002, ?plungeFeed:Null<Float>, ?featureRef:String):CamJob {
		if (islands == null)
			throw "CAM pocket needs an island contour list";
		if (islands.length == 0)
			return pocket(outer, tool, depth, feed, stepOver, stepDown, plungeFeed, featureRef);
		require(outer, tool, depth, feed);
		if (!Math.isFinite(stepOver) || stepOver <= 0.0 || stepOver > tool.diameter)
			throw "CAM pocket step-over must be positive and no larger than tool diameter";
		var entryFeed = checkedEntryFeed(feed, plungeFeed);
		var radius = tool.diameter * 0.5;
		var levels = depthLevels(outer.z, depth, stepDown);
		for (island in islands) {
			require(island, tool, depth, feed);
			if (Math.abs(island.z - outer.z) > 1e-8)
				throw "CAM pocket boundaries must share a work plane";
		}
		var passes = CamPocketPlanner.plan(outer, radius, stepOver, islands);
		var outerPath = offsetPath(outer, radius, levels[0], false);
		for (island in islands) {
			GeometryOffset.validateProfile(island.vertices, outerPath, radius, false);
			var islandPath = offsetPath(island, radius, levels[0], true);
			GeometryOffset.validateProfile(outer.vertices, islandPath, radius, true);
			for (other in islands)
				if (other != island)
					GeometryOffset.validateProfile(other.vertices, islandPath, radius, false);
		}
		var span = nextSpan(featureRef);
		selectTool(tool, span);
		for (index in 0...levels.length) {
			var level = levels[index];
			var prior = index == 0 ? outer.z : levels[index - 1];
			cutPocketPasses(passes, level, prior, outer.z, feed, entryFeed, span, outer, islands, radius);
			for (island in islands)
				cutPocketBoundary(island, radius, level, prior, feed, entryFeed, span, true);
			cutPocketBoundary(outer, radius, level, prior, feed, entryFeed, span, false);
		}
		return this;
	}

	function cutPocketPasses(passes:Array<{y:Float, left:Float, right:Float}>, level:Float, prior:Float, surface:Float, feed:Float, plungeFeed:Float,
			span:Provenance, contour:CamContour, islands:Array<CamContour>, radius:Float):Void {
		var first = true;
		for (pass in passes) {
			var forward = cutDirection != CamCutDirection.Conventional;
			var start = new Point3(forward ? pass.left : pass.right, pass.y, level);
			var end = new Point3(forward ? pass.right : pass.left, pass.y, level);
			var local = !first && CamPocketPlanner.canLink(contour, islands, radius, current, start);
			if (local) {
				var z = Math.min(safeZ, prior + approachClearance);
				rapid(new Point3(current.x, current.y, z), span);
				feedTo(new Point3(start.x, start.y, z), feed, span);
			}
			enterPocket(start, end, prior, surface, feed, plungeFeed, span, local);
			feedTo(end, feed, span);
			first = false;
		}
		rapid(new Point3(current.x, current.y, safeZ), span);
	}

	function cutPocketRing(contour:CamContour, level:Float, prior:Float, surface:Float, feed:Float, plungeFeed:Float, span:Provenance):Void {
		if (cutDirection != null && ((contour.signedArea > 0) != (cutDirection == CamCutDirection.Climb))) {
			var reversed = contour.vertices.copy();
			reversed.reverse();
			contour = new CamContour(reversed);
		}
		var first = contour.vertices[0], next = contour.vertices[1];
		var start = new Point3(first.x, first.y, level);
		enterPocket(start, new Point3(next.x, next.y, level), prior, surface, feed, plungeFeed, span);
		for (i in 1...contour.vertices.length) {
			var point = contour.vertices[i];
			feedTo(new Point3(point.x, point.y, level), feed, span);
		}
		feedTo(start, feed, span);
		rapid(new Point3(start.x, start.y, safeZ), span);
	}

	function cutPocketBoundary(contour:CamContour, radius:Float, level:Float, prior:Float, feed:Float, plungeFeed:Float, span:Provenance, outside:Bool):Void {
		var path = offsetPath(contour, radius, level, outside);
		var start = GeometryTools.pointAt(path[0], 0.0);
		// A chord across an arc can enter the retained island. Only a straight
		// boundary supports the existing linear entry ramp; arcs enter vertically.
		var next = switch path[0] {
			case Line(_, end): end;
			case _: start;
		};
		enterPocket(start, next, prior, contour.z, feed, plungeFeed, span);
		for (geometry in path)
			feedGeometry(geometry, feed, span);
		rapid(new Point3(start.x, start.y, safeZ), span);
	}

	/** Approach above stock, then ramp and retrace or plunge at its own feed. */
	function enterPocket(start:Point3, firstEnd:Point3, prior:Float, surface:Float, feed:Float, plungeFeed:Float, span:Provenance,
			localLink:Bool = false):Void {
		if (!localLink)
			rapidToSafeXY(start.x, start.y, span);
		rapid(new Point3(start.x, start.y, Math.min(safeZ, prior + approachClearance)), span);
		feedTo(new Point3(start.x, start.y, prior), plungeFeed, span);
		var drop = prior - start.z;
		var slope = Math.min(0.1, plungeFeed / feed);
		var run = drop / slope;
		var dx = firstEnd.x - start.x, dy = firstEnd.y - start.y;
		var length = Math.sqrt(dx * dx + dy * dy);
		if (cutDirection == null && drop > 1e-10 && run <= length + 1e-12) {
			var fraction = Math.min(1.0, run / length);
			var rampEnd = new Point3(start.x + dx * fraction, start.y + dy * fraction, start.z);
			feedTo(rampEnd, feed, span);
			// The ramp leaves a shallow wedge. Recut it at the requested depth.
			feedTo(start, feed, span);
		} else
			feedTo(start, plungeFeed, span);
	}

	static function checkedEntryFeed(feed:Float, requested:Null<Float>):Float {
		var value:Float = requested == null ? feed * 0.25 : requested;
		if (!Math.isFinite(value) || value <= 0.0 || value > feed)
			throw "CAM plunge feed must be positive and no greater than cutting feed";
		return value;
	}

	static function hasConcaveCorner(contour:CamContour):Bool {
		var vertices = contour.vertices;
		var orientation = contour.signedArea > 0.0 ? 1.0 : -1.0;
		for (i in 0...vertices.length) {
			var a = vertices[i], b = vertices[(i + 1) % vertices.length];
			var c = vertices[(i + 2) % vertices.length];
			var abx = b.x - a.x, aby = b.y - a.y;
			var bcx = c.x - b.x, bcy = c.y - b.y;
			var turn = orientation * (abx * bcy - aby * bcx) / (Math.sqrt(abx * abx + aby * aby) * Math.sqrt(bcx * bcx + bcy * bcy));
			if (turn < -1e-9)
				return true;
		}
		return false;
	}

	/** Expand hole centres into safe rapid, feed, and retract moves. */
	public function drill(holes:Array<Point3>, tool:Tool, depth:Float, retractZ:Float, feed:Float, ?featureRef:String, peckDepth:Float = 0):CamJob {
		if (holes == null || holes.length == 0 || tool == null || !Math.isFinite(depth) || !Math.isFinite(retractZ) || !Math.isFinite(feed) || feed <= 0.0)
			throw "CAM drill needs holes, tool, depth, retract and feed";
		for (hole in holes) {
			if (hole == null || !Math.isFinite(hole.x) || !Math.isFinite(hole.y) || !Math.isFinite(hole.z) || depth >= hole.z || retractZ < hole.z
				|| safeZ < retractZ)
				throw "CAM drill depth and retract must be below safe Z";
		}
		if (!Math.isFinite(peckDepth) || peckDepth < 0)
			throw "Invalid drill peck depth";
		for (hole in holes)
			checkReach(tool, hole.z - depth);
		var span = nextSpan(featureRef);
		selectTool(tool, span);
		for (hole in holes) {
			rapidToSafeXY(hole.x, hole.y, span);
			rapid(new Point3(hole.x, hole.y, retractZ), span);
			var prior = hole.z;
			var increment = peckDepth == 0 ? hole.z - depth : peckDepth;
			if ((hole.z - depth) / increment > 10000)
				throw "Drill exceeds peck count limit";
			while (prior > depth + 1e-12) {
				var next = Math.max(depth, prior - increment);
				rapid(new Point3(hole.x, hole.y, Math.min(retractZ, prior + approachClearance)), span);
				feedTo(new Point3(hole.x, hole.y, next), feed, span);
				rapid(new Point3(hole.x, hole.y, retractZ), span);
				prior = next;
			}
			rapid(new Point3(hole.x, hole.y, safeZ), span);
		}
		return this;
	}

	public function finish(?setup:Setup):ToolpathProgram {
		if (ops.length == 0)
			throw "CAM job has no operations";
		var result = ArcFitting.fit(ops, arcTolerance);
		var endSpan = Provenance.cam(operationNumber + 1);
		result.push(ToolpathOp.Spindle(Off, 0.0, endSpan));
		if (initialRetract || activeLength > 0)
			result.push(ToolpathOp.ToolLengthOffset(0, 0.0, endSpan));
		result.push(ToolpathOp.End(endSpan));
		var library = new ToolLibrary();
		for (tool in tools)
			library.set(tool);
		var placed = setup == null ? new Setup("1", new Point3(0.0, 0.0, 0.0)) : setup;
		if (setup != null)
			result.unshift(ToolpathOp.SetSetup(placed.id, Provenance.cam(0)));
		return new ToolpathProgram(result, library, [placed]);
	}

	function cutLoop(contour:CamContour, depth:Float, feed:Float, plungeFeed:Float, span:Provenance, prior:Float):Void {
		if (cutDirection != null && ((contour.signedArea > 0) != (cutDirection == CamCutDirection.Climb))) {
			var vertices = contour.vertices.copy();
			vertices.reverse();
			contour = new CamContour(vertices);
		}
		var first = contour.vertices[0];
		rapidToSafeXY(first.x, first.y, span);
		rapid(new Point3(first.x, first.y, Math.min(safeZ, prior + approachClearance)), span);
		feedTo(new Point3(first.x, first.y, depth), plungeFeed, span);
		for (i in 1...contour.vertices.length) {
			var next = contour.vertices[i];
			feedTo(new Point3(next.x, next.y, depth), feed, span);
		}
		feedTo(new Point3(first.x, first.y, depth), feed, span);
		rapid(new Point3(first.x, first.y, safeZ), span);
	}

	function cutOffset(contour:CamContour, radius:Float, depth:Float, feed:Float, plungeFeed:Float, span:Provenance, outside:Bool, prior:Float):Void {
		var path = offsetPath(contour, radius, depth, outside);
		if (path.length == 0)
			return;
		var first = toolpathkit.path.GeometryTools.pointAt(path[0], 0.0);
		rapidToSafeXY(first.x, first.y, span);
		rapid(new Point3(first.x, first.y, Math.min(safeZ, prior + approachClearance)), span);
		feedTo(first, plungeFeed, span);
		for (geometry in path)
			feedGeometry(geometry, feed, span);
		rapid(new Point3(first.x, first.y, safeZ), span);
	}

	function offsetPath(contour:CamContour, radius:Float, depth:Float, outside:Bool):Array<PathGeometry> {
		var path = contour.exactBoundary == null ? GeometryOffset.profile(contour.vertices, radius, depth,
			outside) : CamExactBoundary.offset(contour, radius, depth, outside);
		return orientPath(path, outside);
	}

	function orientPath(path:Array<PathGeometry>, outside:Bool):Array<PathGeometry> {
		if (cutDirection == null || path.length == 0)
			return path;
		var area = 0.0;
		for (g in path) {
			var previous = GeometryTools.pointAt(g, 0);
			for (i in 1...17) {
				var next = GeometryTools.pointAt(g, GeometryTools.length(g) * i / 16);
				area += previous.x * next.y - next.x * previous.y;
				previous = next;
			}
		}
		var ccw = (cutDirection == CamCutDirection.Climb) != outside;
		if ((area > 0) == ccw)
			return path;
		var result:Array<PathGeometry> = [];
		for (index in 0...path.length)
			switch path[path.length - index - 1] {
				case Line(a, b):
					result.push(Line(b, a));
				case Arc(center, radius, start, sweep):
					result.push(Arc(center, radius, start + sweep, -sweep));
				case _:
					throw "CAM ring direction needs planar line/arc geometry";
			}
		return result;
	}

	function cutReliefs(centers:Array<Point3>, depth:Float, feed:Float, span:Provenance):Void {
		for (center in centers) {
			rapidToSafeXY(center.x, center.y, span);
			rapid(new Point3(center.x, center.y, Math.min(safeZ, center.z + approachClearance)), span);
			feedTo(new Point3(center.x, center.y, depth), feed, span);
			rapid(new Point3(center.x, center.y, safeZ), span);
		}
	}

	function rapid(target:Point3, span:Provenance):Void {
		if (current.distanceTo(target) > 1e-12) {
			var kind = target.z > current.z + 1e-12 ? MoveKind.Retract : Math.abs(target.z - current.z) <= 1e-12 ? MoveKind.Link : MoveKind.Rapid;
			ops.push(ToolpathOp.Move(kind, PathGeometry.Line(current, target), 0.0, 0.0, span));
		}
		current = target;
	}

	function rapidToSafeXY(x:Float, y:Float, span:Provenance):Void {
		rapid(new Point3(current.x, current.y, safeZ), span);
		rapid(new Point3(x, y, safeZ), span);
	}

	function feedTo(target:Point3, speed:Float, span:Provenance):Void {
		if (current.distanceTo(target) > 1e-12) {
			var kind = MoveKind.Cut;
			if (target.z < current.z - 1e-12)
				kind = Math.abs(target.x - current.x) <= 1e-12
					&& Math.abs(target.y - current.y) <= 1e-12 ? MoveKind.Plunge : MoveKind.Ramp;
			ops.push(ToolpathOp.Move(kind, PathGeometry.Line(current, target), speed, kind == MoveKind.Cut ? blendTolerance : 0.0, span));
		}
		current = target;
	}

	function feedGeometry(geometry:PathGeometry, speed:Float, span:Provenance):Void {
		ops.push(ToolpathOp.Move(MoveKind.Cut, geometry, speed, blendTolerance, span));
		current = toolpathkit.path.GeometryTools.pointAt(geometry, toolpathkit.path.GeometryTools.length(geometry));
	}

	function selectTool(tool:Tool, span:Provenance):Void {
		var known = false;
		for (used in tools)
			if (used.number == tool.number) {
				if (used != tool)
					throw 'CAM job uses two different tools numbered ${tool.number}';
				known = true;
			}
		if (!known)
			tools.push(tool);
		if (selectedTool != tool.number) {
			// The first tool is loaded where the job starts, which must be clear: no
			// move is safe before the tool's length is known. Later changes retract
			// far enough that neither tool's tip comes below safe Z.
			if (selectedTool >= 0) {
				rapid(new Point3(current.x, current.y, safeZ + Math.max(0.0, tool.length - activeLength)), span);
				ops.push(ToolpathOp.Spindle(Off, 0.0, span));
			}
			ops.push(ToolpathOp.ToolChange(tool.number, span));
			// A measured tool is programmed at its tip through G43; an unmeasured
			// one at the gauge line. The spindle has not moved, so its programmed
			// Z shifts by the change in offset.
			if (tool.length > 0.0 || activeLength > 0.0) {
				ops.push(ToolpathOp.ToolLengthOffset(tool.length > 0.0 ? tool.number : 0, tool.length, span));
				current = new Point3(current.x, current.y, current.z + activeLength - tool.length);
				activeLength = tool.length;
			} else if (selectedTool < 0 && initialRetract) {
				ops.push(ToolpathOp.ToolLengthOffset(0, 0.0, span));
			}
			if (selectedTool < 0 && initialRetract) {
				var target = new Point3(current.x, current.y, safeZ);
				ops.push(ToolpathOp.Move(MoveKind.Retract, PathGeometry.Line(current, target), 0.0, 0.0, span));
				current = target;
			}
			ops.push(ToolpathOp.Spindle(Clockwise, requestedSpindleRpm, span));
			if (spindleSpinUpSeconds > 0)
				ops.push(ToolpathOp.Dwell(spindleSpinUpSeconds, span));
			selectedTool = tool.number;
			selectedSpindleRpm = requestedSpindleRpm;
		} else if (selectedSpindleRpm != requestedSpindleRpm) {
			ops.push(ToolpathOp.Spindle(Clockwise,requestedSpindleRpm,span));
			if (spindleSpinUpSeconds > 0) ops.push(ToolpathOp.Dwell(spindleSpinUpSeconds,span));
			selectedSpindleRpm = requestedSpindleRpm;
		}
	}

	static function depthLevels(surface:Float, depth:Float, stepDown:Float):Array<Float> {
		if (!Math.isFinite(stepDown) || stepDown <= 0.0)
			throw "CAM step-down must be positive";
		var count = Std.int(Math.ceil((surface - depth) / stepDown));
		if (count < 1 || count > 10000)
			throw "CAM cut needs 1..10000 depth levels";
		return [
			for (level in 1...(count + 1))
				Math.max(depth, surface - level * stepDown)
		];
	}

	static function faceRef(face:Face):Null<String>
		return face.index < 0 ? null : 'face:${face.index}';

	function nextSpan(?featureRef:String):Provenance
		return Provenance.cam(++operationNumber, featureRef);

	function checkReach(tool:Tool, depth:Float):Void {
		if (tool.fluteLength != null && depth > tool.fluteLength + 1e-10)
			throw "CAM depth exceeds flute length";
		if (tool.stickOut != null && depth + approachClearance > tool.stickOut + 1e-10)
			throw "CAM depth exceeds tool stick-out";
	}

	function require(contour:CamContour, tool:Tool, depth:Float, feed:Float):Void {
		if (contour == null || tool == null || tool.diameter <= 0.0 || !Math.isFinite(depth) || depth >= contour.z || !Math.isFinite(feed) || feed <= 0.0
			|| safeZ <= contour.z)
			throw "CAM cut needs a contour, cutter, depth, feed and safe Z";
		checkReach(tool, contour.z - depth);
	}
}
