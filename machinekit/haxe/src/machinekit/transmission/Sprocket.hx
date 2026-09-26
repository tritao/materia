package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Roller chain sprocket with a simplified tooth outline (straight flanks between root and tip
 * radii, not the true ANSI B29.1 seating-curve profile), extruded along local +Z.
 * Diameters follow the standard relations: pitch diameter p / sin(pi/z), root (seating) diameter
 * pitch diameter minus the roller diameter, outside diameter p * (0.6 + cot(pi/z)). The roller
 * diameter defaults to 0.625 p (ANSI #40..#240 proportion); pass the chain's actual roller.
 * CAD frame: front face at z=0, back face at z=thickness, matching `DeepGrooveBearing`.
 * Connectors: `front`, `back` (faces) and `axis` (mid-thickness), all with +Y along +Z.
 */
class Sprocket extends MachineComponent {
	public final pitch:Float;
	public final rollerDiameter:Float;
	public final teeth:Int;
	public final boreDiameter:Float;
	public final thickness:Float;
	public final pitchDiameter:Float;
	public final outsideDiameter:Float;
	public final rootDiameter:Float;

	public function new(pitch:Float, teeth:Int, boreDiameter:Float, thickness:Float, ?rollerDiameter:Float) {
		if (!(pitch > 0)) throw "Sprocket needs a positive chain pitch";
		if (teeth < 8) throw "Sprocket needs at least 8 teeth";
		if (!(boreDiameter > 0)) throw "Sprocket needs a positive bore diameter";
		if (!(thickness > 0)) throw "Sprocket needs a positive thickness";
		var roller = rollerDiameter == null ? 0.625 * pitch : rollerDiameter;
		if (!(roller > 0) || !(roller < pitch)) throw "Sprocket roller diameter must be positive and less than the pitch";
		var pitchDia = pitch / Math.sin(Math.PI / teeth);
		var outsideDia = pitch * (0.6 + Math.cos(Math.PI / teeth) / Math.sin(Math.PI / teeth));
		var rootDia = pitchDia - roller;
		if (!(rootDia > boreDiameter))
			throw "Sprocket root diameter must clear the bore; use more teeth or a smaller bore";
		var pitchText = Dimension.format(pitch);
		super('SPROCKET-P$pitchText-${teeth}T', 'Sprocket, $pitchText mm pitch, ${teeth} teeth', "steel");
		this.pitch = pitch;
		this.rollerDiameter = roller;
		this.teeth = teeth;
		this.boreDiameter = boreDiameter;
		this.thickness = thickness;
		pitchDiameter = pitchDia;
		outsideDiameter = outsideDia;
		rootDiameter = rootDia;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, thickness / 2));
		addConnector("back", Face, Solids.axial(0, 0, thickness));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.prism(profile(), 0, thickness);
		return Solids.cut(body, [Solids.cylinder(boreDiameter / 2, -0.1, thickness + 0.1)]);
	}

	function profile():Array<Vector> {
		var rf = rootDiameter / 2, ra = outsideDiameter / 2;
		var angleStep = 2 * Math.PI / teeth;
		var halfRoot = 0.35 * angleStep / 2, halfTip = 0.2 * angleStep / 2;
		var points:Array<Vector> = [];
		for (i in 0...teeth) {
			var center = i * angleStep;
			points.push(new Vector(rf * Math.cos(center - halfRoot), rf * Math.sin(center - halfRoot)));
			points.push(new Vector(ra * Math.cos(center - halfTip), ra * Math.sin(center - halfTip)));
			points.push(new Vector(ra * Math.cos(center + halfTip), ra * Math.sin(center + halfTip)));
			points.push(new Vector(rf * Math.cos(center + halfRoot), rf * Math.sin(center + halfRoot)));
		}
		return points;
	}
}
