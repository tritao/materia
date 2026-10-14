import haxe.io.Bytes;
import materia.drawing.DrawingProjection;
import materia.drawing.DrawingSvgExporter;
import materia.drawing.DrawingProjection.DrawingViewSpec;
import materia.project.SceneArtifact.SceneArtifactPart;
import materia.project.ThreadRenderData;
import materia.project.ThreadSpecData.ThreadHand;
import materia.project.ThreadSpecData.ThreadProfile;

class DrawingProjectionTests {
	static var assertions = 0;
	static function check(value:Bool, message:String):Void {
		assertions++;
		if (!value) throw message;
	}
	static function rejects(action:Void->Void, message:String):Void {
		var threw = false;
		try action() catch (_:Dynamic) threw = true;
		check(threw, message);
	}

	static function cube():SceneArtifactPart {
		var points = [[0.0,0.0,0.0],[1.0,0.0,0.0],[1.0,1.0,0.0],[0.0,1.0,0.0],
			[0.0,0.0,1.0],[1.0,0.0,1.0],[1.0,1.0,1.0],[0.0,1.0,1.0]];
		var vertices = Bytes.alloc(points.length * 24), normals = Bytes.alloc(points.length * 24);
		for (index in 0...points.length) for (axis in 0...3) vertices.setDouble(index * 24 + axis * 8, points[index][axis]);
		var triangles = [4,5,6,4,6,7, 0,2,1,0,3,2, 0,1,5,0,5,4,
			1,2,6,1,6,5, 0,4,7,0,7,3, 3,7,6,3,6,2];
		var indices = Bytes.alloc(triangles.length * 4);
		for (index in 0...triangles.length) indices.setInt32(index * 4, triangles[index]);
		var edgePoints = [
			[0.0,0.0,0.0, 1.0,0.0,0.0], [1.0,0.0,0.0, 1.0,1.0,0.0],
			[1.0,1.0,0.0, 0.0,1.0,0.0], [0.0,1.0,0.0, 0.0,0.0,0.0],
			[0.0,0.0,1.0, 1.0,0.0,1.0], [1.0,0.0,1.0, 1.0,1.0,1.0],
			[1.0,1.0,1.0, 0.0,1.0,1.0], [0.0,1.0,1.0, 0.0,0.0,1.0],
			[0.0,0.0,0.0, 0.0,0.0,1.0], [1.0,0.0,0.0, 1.0,0.0,1.0],
			[1.0,1.0,0.0, 1.0,1.0,1.0], [0.0,1.0,0.0, 0.0,1.0,1.0]
		];
		var edges = Bytes.alloc(edgePoints.length * 48);
		for (edge in 0...edgePoints.length) for (coordinate in 0...6)
			edges.setDouble(edge * 48 + coordinate * 8, edgePoints[edge][coordinate]);
		return {id:"cube", name:"Cube", red:0.5, green:0.5, blue:0.5,
			vertexCount:points.length, indexCount:triangles.length, vertices:vertices, normals:normals,
			indices:indices, edgeSegments:edges, edgeIds:Bytes.alloc(edgePoints.length * 4), faceRanges:[]};
	}

	static function narrowOccluder():SceneArtifactPart {
		var points = [[0.0,0.0,0.0],[1.0,0.0,0.0],[0.514,-0.1,1.0],[0.526,-0.1,1.0],[0.52,0.1,1.0]];
		var vertices = Bytes.alloc(points.length * 24), normals = Bytes.alloc(points.length * 24);
		for (index in 0...points.length) for (axis in 0...3) vertices.setDouble(index * 24 + axis * 8, points[index][axis]);
		var indices = Bytes.alloc(12);
		indices.setInt32(0, 2); indices.setInt32(4, 3); indices.setInt32(8, 4);
		var edges = Bytes.alloc(48);
		edges.setDouble(0, 0); edges.setDouble(8, 0); edges.setDouble(16, 0);
		edges.setDouble(24, 1); edges.setDouble(32, 0); edges.setDouble(40, 0);
		return {id:"narrow-occluder", name:"Narrow occluder", red:0.5, green:0.5, blue:0.5,
			vertexCount:points.length, indexCount:3, vertices:vertices, normals:normals, indices:indices,
			edgeSegments:edges, edgeIds:Bytes.alloc(4), faceRanges:[]};
	}

	public static function run():Int {
		assertions = 0;
		var part = cube();
		var front:DrawingViewSpec = {id:"front", direction:[0,0,1], up:[0,1,0]};
		var visible = DrawingProjection.project(part, 0.001, front);
		var visibleLines = [for (line in visible.lines) if (line.style == "visible") line];
		check(visibleLines.length == 4, "the front orthographic view retains its four visible boundary edges");
		check(visible.bounds.minX == 0 && visible.bounds.maxX == 1,
			"orthographic projection builds a stable millimetre-valued view frame");
		front.showHiddenLines = true;
		var hidden = DrawingProjection.project(part, 0.001, front);
		check([for (line in hidden.lines) if (line.style == "hidden") line].length == 4,
			"hidden-line mode classifies the rear face behind the front surface");
		var narrow = narrowOccluder();
		var narrowView:DrawingViewSpec = {id:"narrow-hidden", direction:[0,0,1], up:[0,1,0], showHiddenLines:true};
		var narrowDrawing = DrawingProjection.project(narrow, 0.001, narrowView);
		var narrowHidden = [for (line in narrowDrawing.lines) if (line.style == "hidden") line];
		check(narrowHidden.length == 1 && Math.abs(Math.abs(narrowHidden[0].x2 - narrowHidden[0].x1) - 0.006) < 1e-6,
			"exact hidden-line clipping retains a short occlusion that fixed edge sampling would miss");

		var thread:ThreadRenderData = {faceName:"bore:cyl.side", faceIndex:3,
			spec:{designation:"M6", majorDiameter:6, pitch:1, starts:1, hand:ThreadHand.RightHand,
				internal:true, toleranceClass:"6H", profile:ThreadProfile.Metric},
			origin:[0.5,0.5,1], xDirection:[1,0,0], direction:[0,0,1], length:8,
			ringAtStart:false, ringAtEnd:true};
		part.threads = [thread];
		var sectionView:DrawingViewSpec = {id:"section", direction:[1,0,0], up:[0,0,1], showHiddenLines:true,
			section:{origin:[0.5,0,0], normal:[1,0,0], keepPositive:true}};
		var section = DrawingProjection.project(part, 0.001, sectionView);
		check([for (line in section.lines) if (line.style == "section") line].length >= 4,
			"a section plane clips the retained half and adds its cut boundary");
		var hatches = [for (line in section.lines) if (line.style == "section-hatch") line];
		check(hatches.length > 0 && [for (line in visible.lines) if (line.style == "section-hatch") line].length == 0,
			"section views hatch cut material while ordinary orthographic views do not");
		for (line in hatches) check(Math.abs(line.x2 - line.x1) > 0 && Math.abs(line.y2 - line.y1) > 0 &&
			line.x1 >= -1e-8 && line.x1 <= 1 + 1e-8 && line.x2 >= -1e-8 && line.x2 <= 1 + 1e-8 &&
			line.y1 >= -1e-8 && line.y1 <= 1 + 1e-8 && line.y2 >= -1e-8 && line.y2 <= 1 + 1e-8,
			"section hatches remain inside the material boundary as oblique line segments");
		var sectionEdgeMaxX = -Math.POSITIVE_INFINITY;
		for (line in section.lines) sectionEdgeMaxX = Math.max(sectionEdgeMaxX, Math.max(line.x1, line.x2));
		check(section.callouts.length == 1 && section.callouts[0].x > sectionEdgeMaxX &&
			section.callouts[0].leaderX < section.callouts[0].x,
			'thread callouts occupy a clear annotation lane with a leader tied to the named face: count=${section.callouts.length}, ' +
			(section.callouts.length == 0 ? "none" : 'x=${section.callouts[0].x}, leader=${section.callouts[0].leaderX}, edge=${sectionEdgeMaxX}'));
		check(section.bounds.maxX - section.bounds.minX > 0.99 && section.bounds.maxY - section.bounds.minY > 0.99,
			"a face-on section projects a closed square boundary");

		var threaded = DrawingProjection.project(part, 0.001, {id:"thread-end", direction:[0,0,1], up:[0,1,0]});
		check([for (line in threaded.lines) if (line.faceName == thread.faceName && line.style == "thread-major") line].length == 64,
			"an end-on internal thread draws the minor diameter as a full major-weight circle");
		check([for (line in threaded.lines) if (line.faceName == thread.faceName && line.style == "thread-minor") line].length == 48,
			"an end-on internal thread draws the nominal major diameter as a three-quarter thin circle");
		var blindBottom = DrawingProjection.project(part, 0.001,
			{id:"blind-hole-bottom", direction:[0,0,-1], up:[0,1,0]});
		check([for (line in blindBottom.lines) if (line.faceName == thread.faceName) line].length == 0,
			"a blind thread has no end symbol when viewed from its closed end");
		var throughThread:ThreadRenderData = {faceName:thread.faceName, faceIndex:thread.faceIndex, spec:thread.spec,
			origin:thread.origin, xDirection:thread.xDirection, direction:thread.direction, length:thread.length,
			ringAtStart:true, ringAtEnd:true};
		part.threads = [throughThread];
		var throughTop = DrawingProjection.project(part, 0.001,
			{id:"through-hole-top", direction:[0,0,1], up:[0,1,0]});
		var throughBottom = DrawingProjection.project(part, 0.001,
			{id:"through-hole-bottom", direction:[0,0,-1], up:[0,1,0]});
		check([for (line in throughTop.lines) if (line.faceName == thread.faceName) line].length == 112 &&
			[for (line in throughBottom.lines) if (line.faceName == thread.faceName) line].length == 112,
			"a through thread has conventional end symbols from either open end");
		part.threads = [thread];
		check(threaded.callouts.length == 1 && threaded.callouts[0].text.indexOf("M6 × 1") >= 0 &&
			threaded.callouts[0].text.indexOf("6H") >= 0 && threaded.callouts[0].text.indexOf("8 mm deep") >= 0,
			"thread callouts derive size, pitch, tolerance and depth from the named thread data");
		var meterPart = cube();
		for (offset in 0...Std.int(meterPart.vertices.length / 8)) meterPart.vertices.setDouble(offset * 8,
			meterPart.vertices.getDouble(offset * 8) * 0.001);
		for (offset in 0...Std.int(meterPart.edgeSegments.length / 8)) meterPart.edgeSegments.setDouble(offset * 8,
			meterPart.edgeSegments.getDouble(offset * 8) * 0.001);
		var meterThread:ThreadRenderData = {faceName:thread.faceName, faceIndex:thread.faceIndex, spec:thread.spec,
			origin:[0.0005,0.0005,0.001], xDirection:[1,0,0], direction:[0,0,1], length:0.008,
			ringAtStart:thread.ringAtStart, ringAtEnd:thread.ringAtEnd};
		meterPart.threads = [meterThread];
		var metricView = DrawingProjection.project(meterPart, 1.0, {id:"thread-end",direction:[0,0,1],up:[0,1,0]});
		check(Math.abs(metricView.bounds.minX - threaded.bounds.minX) < 1e-8 &&
			Math.abs(metricView.bounds.maxY - threaded.bounds.maxY) < 1e-8 && metricView.callouts[0].text == threaded.callouts[0].text,
			"thread graphics and notes use part units while thread dimensions stay millimetric");
		var ballThread:ThreadRenderData = {faceName:"race:spherical.side", faceIndex:4,
			spec:{designation:"BS8", majorDiameter:8, pitch:3, starts:1, hand:ThreadHand.RightHand,
				internal:true, toleranceClass:"nominal", profile:ThreadProfile.Ball},
			origin:[0.5,0.5,1], xDirection:[1,0,0], direction:[0,0,1], length:8,
			ringAtStart:false, ringAtEnd:true};
		part.threads = [ballThread];
		var ballView = DrawingProjection.project(part, 0.001, {id:"ball-race",direction:[0,0,1],up:[0,1,0]});
		check(ballView.callouts.length == 0 && [for (line in ballView.lines) if (line.faceName == ballThread.faceName) line].length == 0,
			"ball-race data does not masquerade as a screw-thread drawing symbol");
		var externalThread:ThreadRenderData = {faceName:"shank:cyl.side", faceIndex:5,
			spec:{designation:"M6", majorDiameter:6, pitch:1, starts:1, hand:ThreadHand.RightHand,
				internal:false, toleranceClass:"6g", profile:ThreadProfile.Metric},
			origin:[0.5,0.5,1], xDirection:[0,1,0], direction:[1,0,0], length:8,
			ringAtStart:false, ringAtEnd:true};
		part.threads = [externalThread];
		var externalView = DrawingProjection.project(part, 0.001, {id:"shaft-side",direction:[0,0,1],up:[0,1,0]});
		var externalLines = [for (line in externalView.lines) if (line.faceName == externalThread.faceName) line];
		check(externalLines.length >= 5 && [for (line in externalLines) if (line.style == "thread-major") line].length >= 3 &&
			externalView.callouts[0].text.indexOf("8 mm long") >= 0 &&
			[for (line in externalView.lines) if (line.faceName == null && line.style == "visible") line].length > 0,
			"a partial external thread retains its named interval, major/minor sides and end limit");
		var externalEndView = DrawingProjection.project(part, 0.001, {id:"shaft-end",direction:[1,0,0],up:[0,1,0]});
		check([for (line in externalEndView.lines) if (line.faceName == externalThread.faceName && line.style == "thread-major") line].length == 64,
			"an end-on external thread draws the nominal major diameter as a full circle");
		check([for (line in externalEndView.lines) if (line.faceName == externalThread.faceName && line.style == "thread-minor") line].length == 48,
			"an end-on external thread draws the minor diameter as a three-quarter thin circle");
		part.threads = [thread];
		var svg = DrawingSvgExporter.svg(threaded);
		check(svg.indexOf("data-style=\"thread-major\"") >= 0 && svg.indexOf("data-face-name=\"bore:cyl.side\"") >= 0 &&
			svg.indexOf("data-style=\"callout-leader\"") >= 0 && svg.indexOf("M6 × 1") >= 0 &&
			svg.indexOf("width=\"210mm\"") >= 0 && svg.indexOf("Scale ") >= 0,
			"the drawing artifact exports line styles, named-face callout, physical page dimensions and view scale");
		var renamedThread:ThreadRenderData = {faceName:"bore:revised.side", faceIndex:thread.faceIndex, spec:thread.spec,
			origin:thread.origin, xDirection:thread.xDirection, direction:thread.direction, length:thread.length,
			ringAtStart:thread.ringAtStart, ringAtEnd:thread.ringAtEnd};
		part.threads = [renamedThread];
		var renamed = DrawingProjection.project(part, 0.001, {id:"renamed-hole",direction:[0,0,1],up:[0,1,0]});
		check(renamed.callouts.length == 1 && renamed.callouts[0].faceName == renamedThread.faceName &&
			[for (line in renamed.lines) if (line.faceName == renamedThread.faceName) line].length == 112,
			"projected symbols and callouts follow the edited named face from the same thread packet");
		rejects(function() DrawingProjection.project(part, 0.001, {id:"bad",direction:[0,0,0],up:[0,1,0]}),
			"drawing views reject a zero projection direction");
		return assertions;
	}
}
