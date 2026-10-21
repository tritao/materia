package machinekit.standard;

import cadkit.modeling.Part;
import cadkit.threads.ThreadSpec;
import cadkit.threads.Threads;
import cadkit.threads.ThreadedFace.ThreadStart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** ISO 4026 flat-point set screw, nominal dimensions, no helical collision solid.
 * Like the other fasteners its frame is at the drive end: the hex socket opens towards +z and the threaded body
 * runs to the flat point at z = -length. Seat it in a bore with `boreSeat`.
 * Length/supplier qualification remains a fabrication input. Socket dimensions:
 * https://www.accu.co.uk/flat-point-set-screws/4547-SSF-M6-5-A2
 * https://cdn.mscdirect.com/global/images/ProductDataSheet/pds_sku_254996_productbrochure_manual%20data%20sheet.pdf
 */
class SocketSetScrew extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public final thread:ThreadSpec;
	public final length:Float;
	public final socketSize:Float;
	public final socketDepth:Float;

	static function parameters():Array<ComponentParameter>
		return [P.length("diameter", 6), P.length("pitch", 1), P.length("length", 5)];

	public static function recipeType():ComponentType {
		if (recipe == null)
			recipe = new ComponentType("machinekit.standard.socket-set-screw", parameters(), v -> new SocketSetScrew(v), false, false, null, false);
		return recipe;
	}

	public static function forThread(spec:ThreadSpec, length:Float):SocketSetScrew {
		if (spec.internal || spec.profile != Metric || spec.hand != RightHand || spec.starts != 1)
			throw "ISO 4026 protection needs an external right-hand metric thread";
		return new SocketSetScrew(new ComponentValues().setNumber("diameter", spec.majorDiameter)
			.setNumber("pitch", spec.pitch).setNumber("length", length));
	}

	public function new(?input:ComponentValues) {
		var values = P.resolve(parameters(), input), diameter = values.number("diameter");
		thread = FastenerThreads.metric(diameter, values.number("pitch"));
		if (FastenerThreads.coarse(diameter).pitch != thread.pitch)
			throw "ISO 4026 protection requires its standard coarse pitch";
		length = values.number("length");
		// Preferred nominal lengths in the cited ISO 4026 manufacturer table.
		if ([2.0, 2.5, 3, 4, 5, 6, 8, 10, 12, 16, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80].indexOf(length) < 0)
			throw "Unsupported ISO 4026 nominal length";
		socketSize = switch Std.int(diameter) {
			case 3: 1.5; case 4: 2; case 5: 2.5; case 6: 3; case 8: 4;
			case 10: 5; case 12: 6; case 16: 8; case 20: 10;
			default: throw "Unsupported ISO 4026 protection diameter";
		};
		// A blind driving socket always leaves a solid resin-sealing end.
		socketDepth = Math.min(socketSize, length - thread.pitch);
		if (!Math.isFinite(length) || length <= thread.pitch || socketDepth <= 0)
			throw "Threaded plug needs a blind socket and a solid end";
		super('ISO4026-${thread.designation}x${Dimension.format(length)}', "Flat-point socket set screw; nominal supplier selection", "steel", false, values);
		addConnector("drive", Face, Solids.axial(0, 0, 0));
		addConnector("tip", Face, Solids.axial(0, 0, -length));
	}

	/**
	 * The screw's frame relative to a bore frame whose z = 0 is where the socket faces out and whose +z runs into
	 * the bore: half a turn about x, so the body fills the bore and the socket stays at the open end.
	 */
	public static function boreSeat():materia.assembly.AssemblyRecord.AssemblyFrame
		return F.fromRotationMatrix(0, 0, 0, [1, 0, 0, 0, -1, 0, 0, 0, -1]);

	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	/** Dimensioned from the standard and datasheets cited above. */
	override public function geometryFidelity():machinekit.component.GeometryFidelity
		return machinekit.component.GeometryFidelity.Datasheet;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		// Envelope is the free thread root after unscrewing; it is used only by
		// removal checks after the explicit fastener-release step.
		var radius = (detail == Envelope ? thread.externalMinorDiameter() : thread.majorDiameter) / 2;
		var shaft = Part.cylinderSpan(radius, -length, 0);
		var tagged:Part;
		try tagged = new Part(Threads.attach(shaft.shape, "cyl.side", thread, AxisMinimum, length))
		catch (error:Dynamic) {shaft.close(); throw error;}
		shaft.close();
		if (detail == Envelope)
			return tagged;
		return Solids.cut(tagged, [Part.prism(cadkit.modeling.Polygon.regular(6, socketSize), -socketDepth, thread.pitch)]);
	}
}
