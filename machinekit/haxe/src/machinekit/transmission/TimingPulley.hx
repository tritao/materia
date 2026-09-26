package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Timing belt pulley with a simplified tooth outline (shallow straight-flank grooves cut into a
 * near-pitch-diameter cylinder, not the true rounded-trapezoid GT2/HTD profile), extruded along
 * local +Z. CAD frame and connectors match `Sprocket`.
 * The pitch line lies in the belt's tension cords, outside the pulley, so the outside diameter is
 * the pitch diameter minus twice the pitch line differential (PLD). The PLD defaults by pitch for
 * the curvilinear HTD/GT2 families (0.254 mm up to 2.5 mm pitch, then 0.381 for 3 mm, 0.5715 for
 * 5 mm, 0.686 for 8 mm, 1.397 for 14 mm); pass it explicitly for other belts (T5: 0.5, XL: 0.254).
 */
class TimingPulley extends MachineComponent {
	public final pitch:Float;
	public final pitchLineDifferential:Float;
	public final teeth:Int;
	public final boreDiameter:Float;
	public final thickness:Float;
	public final pitchDiameter:Float;
	public final outsideDiameter:Float;
	public final grooveDiameter:Float;

	public function new(pitch:Float, teeth:Int, boreDiameter:Float, thickness:Float, ?pitchLineDifferential:Float) {
		if (!(pitch > 0)) throw "Timing pulley needs a positive belt pitch";
		if (teeth < 8) throw "Timing pulley needs at least 8 teeth";
		if (!(boreDiameter > 0)) throw "Timing pulley needs a positive bore diameter";
		if (!(thickness > 0)) throw "Timing pulley needs a positive thickness";
		var pld = pitchLineDifferential == null ? defaultPitchLineDifferential(pitch) : pitchLineDifferential;
		if (!(pld >= 0) || !(pld < 0.25 * pitch)) throw "Timing pulley pitch line differential must be between 0 and a quarter pitch";
		var pitchDia = pitch * teeth / Math.PI;
		var grooveDepth = 0.2 * pitch;
		var outsideDia = pitchDia - 2 * pld;
		var grooveDia = outsideDia - 2 * grooveDepth;
		if (!(grooveDia > boreDiameter))
			throw "Timing pulley groove diameter must clear the bore; use more teeth or a smaller bore";
		var pitchText = Dimension.format(pitch);
		super('PULLEY-P$pitchText-${teeth}T', 'Timing pulley, $pitchText mm pitch, ${teeth} teeth', "aluminium 6061");
		this.pitch = pitch;
		this.pitchLineDifferential = pld;
		this.teeth = teeth;
		this.boreDiameter = boreDiameter;
		this.thickness = thickness;
		pitchDiameter = pitchDia;
		outsideDiameter = outsideDia;
		grooveDiameter = grooveDia;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, thickness / 2));
		addConnector("back", Face, Solids.axial(0, 0, thickness));
	}

	/** Pitch line differential of the HTD/GT2 belt family with this pitch. */
	public static function defaultPitchLineDifferential(pitch:Float):Float
		return pitch <= 2.5 ? 0.254 : pitch <= 3.5 ? 0.381 : pitch <= 6 ? 0.5715 : pitch <= 10 ? 0.686 : 1.397;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.prism(profile(), 0, thickness);
		return Solids.cut(body, [Solids.cylinder(boreDiameter / 2, -0.1, thickness + 0.1)]);
	}

	function profile():Array<Vector> {
		var ra = outsideDiameter / 2, rf = grooveDiameter / 2;
		var angleStep = 2 * Math.PI / teeth;
		var halfLand = 0.3 * angleStep / 2, halfGroove = 0.15 * angleStep / 2;
		var points:Array<Vector> = [];
		for (i in 0...teeth) {
			var center = i * angleStep;
			points.push(new Vector(ra * Math.cos(center - halfLand), ra * Math.sin(center - halfLand)));
			points.push(new Vector(rf * Math.cos(center - halfGroove), rf * Math.sin(center - halfGroove)));
			points.push(new Vector(rf * Math.cos(center + halfGroove), rf * Math.sin(center + halfGroove)));
			points.push(new Vector(ra * Math.cos(center + halfLand), ra * Math.sin(center + halfLand)));
		}
		return points;
	}
}
