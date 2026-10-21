import cnckit.CncCompiler;
import cnckit.CncController;
import cnckit.CncDiagnostic;
import haxe.Json;
import robotkit.model.ActuatorDrive.StepperDrive;
import robotkit.model.Actuator;
import robotkit.model.Joint;
import robotkit.model.JointCoupling;
import robotkit.model.JointLimits;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.SteadyLoads;
import robotkit.model.TorqueSpeedCurve;
import robotkit.model.Transmission;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.ToolpathOp;

typedef DiagnosticJson = {severity:String, code:String, line:Int, column:Int, message:String};
typedef MoveJson = {kind:String, line:Int, points:Array<Array<Float>>};
typedef BeltJson = {teeth:Int, velocity:Float, acceleration:Float};

/**
 * The docs site's live runtime: real kits (CncKit, ToolpathKit, RobotKit's robot model) compiled to Wasm GC, called from widgets on the concept pages.
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

	/**
	 * Derives a belt-driven axis's limits with RobotKit's `RobotModel.coupledLimits`, for every pulley from 12 to 60
	 * teeth (GT2, 2 mm pitch), and prints them: the carriage of `mass` kg rides a belt on a pulley driven directly by
	 * a stepper with `holding` N m up to `corner` rad/s, falling as 1 / speed beyond it. Returns the number of rows.
	 */
	@:expose("docs_belt")
	static function belt(mass:Float, holding:Float, corner:Float):Int {
		var rows:Array<BeltJson> = [];
		var teeth = 12;
		while (teeth <= 60) {
			var limits = beltAxis(teeth, mass, holding, corner).coupledLimits("carriage", new SteadyLoads());
			rows.push({teeth: teeth, velocity: limits.requireVelocity(), acceleration: limits.requireAcceleration()});
			teeth += 2;
		}
		Sys.println(Json.stringify({rows: rows}));
		return rows.length;
	}

	static function beltAxis(teeth:Int, mass:Float, holding:Float, corner:Float):RobotModel {
		var model = new RobotModel("belt-axis");
		var frame = model.addLink(new Link("frame"));
		var carriage = model.addLink(new Link("carriage"));
		var rotor = model.addLink(new Link("pulley"));
		carriage.mass = mass;
		rotor.mass = 0.05;
		rotor.centerOfMass = [0.0, 0.0, 0.0];
		rotor.inertiaTensor = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6];
		var slide = model.addJoint(new Joint("carriage", JointType.Prismatic, frame, carriage));
		// A horizontal axis: joints default to +Z, which would lift the carriage against gravity.
		slide.axis = [1.0, 0.0, 0.0];
		slide.limits = new JointLimits(0.0, 1.0);
		var shaft = model.addJoint(new Joint("pulley", JointType.Continuous, frame, rotor));
		shaft.limits = new JointLimits(-1e9, 1e9);
		// The pulley turns once per pitch length of belt: 2 pi rad per teeth x 2 mm.
		var coupling = new JointCoupling("belt", "carriage", "pulley", 2 * Math.PI / (teeth * 0.002), 0.0);
		coupling.efficiency = 0.97;
		model.addCoupling(coupling);
		var motor = new Actuator("motor", holding, corner * 4, Transmission.SimpleTransmission("pulley", 1.0, 0.0));
		motor.drive = new StepperDrive(200.0, 5e-6, holding,
			new TorqueSpeedCurve([0.0, corner, corner * 2, corner * 4], [holding, holding, holding / 2, holding / 4]));
		model.addActuator(motor);
		return model;
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
