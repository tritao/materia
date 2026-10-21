import cnckit.CncCompiler;
import cnckit.CncController;
import cnckit.CncDiagnostic;
import haxe.Json;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.ToolpathOp;

typedef DiagnosticJson = {severity:String, code:String, line:Int, column:Int, message:String};
typedef MoveJson = {kind:String, line:Int, points:Array<Array<Float>>};

/**
 * The docs site's live runtime: real kits compiled to Wasm GC, called from widgets on the concept pages.
 *
 * Exports take scalars only, so a page sends text one character code at a time with `docs_char`, then calls
 * an operation; the operation prints one JSON line with the result, which the page's print hook receives.
 */
class DocsRuntime {
	static var input = new StringBuf();

	static function main() {}

	/** Appends one character code to the pending input. */
	@:expose("docs_char")
	static function char(code:Int):Void
		input.addChar(code);

	/**
	 * Compiles the pending input as LinuxCNC G-code with CncKit and prints the result: every diagnostic with
	 * its line and column, and every move as a polyline in millimetres, tagged with its kind and source line.
	 * Returns the number of errors.
	 */
	@:expose("docs_gcode")
	static function gcode():Int {
		var source = input.toString();
		input = new StringBuf();
		var result = CncCompiler.compileDetailed(source, new CncController());
		var errors = 0;
		var diagnostics:Array<DiagnosticJson> = [];
		for (diagnostic in result.diagnostics) {
			if (diagnostic.severity == Error)
				errors++;
			diagnostics.push({
				severity: diagnostic.severity == Error ? "error" : "warning",
				code: diagnostic.code,
				line: diagnostic.span.line,
				column: diagnostic.span.column,
				message: diagnostic.message
			});
		}
		var moves:Array<MoveJson> = [];
		for (op in result.program.ops)
			switch op {
				case Move(kind, geometry, _, _, provenance):
					moves.push({kind: Std.string(kind).toLowerCase(), line: provenance.line, points: polyline(geometry)});
				case _:
			}
		Sys.println(Json.stringify({diagnostics: diagnostics, moves: moves}));
		return errors;
	}

	/** A move as points in millimetres; arcs and helices are sampled every 5 degrees, via ToolpathKit's own geometry. */
	static function polyline(geometry:PathGeometry):Array<Array<Float>> {
		var sweep = switch geometry {
			case Line(_, _): 0.0;
			case Arc(_, _, _, sweepAngle) | Circular(_, _, _, sweepAngle, _, _): sweepAngle;
		};
		var steps = sweep == 0 ? 1 : Std.int(Math.max(2, Math.ceil(Math.abs(sweep) / (Math.PI / 36))));
		var length = GeometryTools.length(geometry);
		var points:Array<Array<Float>> = [];
		for (i in 0...steps + 1) {
			var p = GeometryTools.pointAt(geometry, length * i / steps);
			points.push([round(p.x * 1000), round(p.y * 1000), round(p.z * 1000)]);
		}
		return points;
	}

	static function round(value:Float):Float
		return Math.round(value * 1000) / 1000;
}
