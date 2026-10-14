import materia.drawing.DrawingProjection;
import materia.drawing.DrawingSvgExporter;
import materia.drawing.DrawingProjection.DrawingViewSpec;
import materia.project.SceneArtifact;
import sys.FileSystem;
import sys.io.File;

/** Exports drawing views from the same named-face CAD threads used by the editor. */
class ThreadDrawingPreview {
	public static function exports():Array<{name:String, svg:String}> {
		var artifact = SceneArtifact.decode(ThreadPreview.preview());
		var plate = artifact.parts[0], screw = artifact.parts[1];
		if (plate.threads == null || plate.threads.length != 2 || screw.threads == null || screw.threads.length != 1)
			throw "Drawing preview needs the two named taps and one external screw thread from CAD";
		var section:DrawingViewSpec = {id:"plate-section", direction:[0,-1,0], up:[0,0,1], showHiddenLines:true,
			section:{origin:[0,0,0], normal:[0,1,0], keepPositive:true}};
		var end:DrawingViewSpec = {id:"plate-end", direction:[0,0,1], up:[0,1,0], showHiddenLines:true};
		var shaft:DrawingViewSpec = {id:"screw-side", direction:[0,-1,0], up:[0,0,1], showHiddenLines:true};
		var sectionDrawing = DrawingProjection.project(plate, artifact.metresPerUnit, section);
		var endDrawing = DrawingProjection.project(plate, artifact.metresPerUnit, end);
		var shaftDrawing = DrawingProjection.project(screw, artifact.metresPerUnit, shaft);
		if ([for (line in sectionDrawing.lines) if (line.style == "section") line].length < 3 ||
			[for (line in sectionDrawing.lines) if (line.style == "section-hatch") line].length == 0 ||
			endDrawing.callouts.length != 2 || shaftDrawing.callouts.length != 1)
			throw 'Drawing preview lost section hatching, boundaries or named thread callouts: section=' +
				[for (line in sectionDrawing.lines) if (line.style == "section") line].length +
				' hatches=' + [for (line in sectionDrawing.lines) if (line.style == "section-hatch") line].length +
				', end=' + endDrawing.callouts.length + ', shaft=' + shaftDrawing.callouts.length;
		return [
			{name:"plate-section.svg", svg:DrawingSvgExporter.svg(sectionDrawing)},
			{name:"plate-end.svg", svg:DrawingSvgExporter.svg(endDrawing)},
			{name:"screw-side.svg", svg:DrawingSvgExporter.svg(shaftDrawing)}
		];
	}
}

function main():Void {
	var directory = Sys.getEnv("MATERIA_DRAWING_OUTPUT_DIR");
	if (directory == null) directory = "build/drawing";
	if (!FileSystem.exists(directory)) FileSystem.createDirectory(directory);
	for (drawing in ThreadDrawingPreview.exports()) {
		File.saveContent(directory + "/" + drawing.name, drawing.svg);
		Sys.println('Exported CAD drawing ${drawing.name} (${drawing.svg.length} characters)');
	}
}
