package machinekit.component;

import cadkit.modeling.Vector;
import cadkit.InertiaTensor;
import machinekit.units.Kilograms;
import machinekit.units.KgMm2;
import machinekit.units.Millimetres;

/** Origin of a component's mass and centre of mass estimate. */
enum MassSource {
	Computed(detail:ComponentDetail);
	Declared;
}

/** Mass in kg, centre in mm, and centroidal inertia in kg mm² in the component frame. */
class MassProperties {
	public final mass:Float;
	public var massKg(get, never):Kilograms;
	function get_massKg():Kilograms return new Kilograms(mass);
	public function centreMm():{x:Millimetres, y:Millimetres, z:Millimetres}
		return {x: new Millimetres(centreOfMass.x), y: new Millimetres(centreOfMass.y),
			z: new Millimetres(centreOfMass.z)};
	public function inertiaKgMm2():Null<{xx:KgMm2, xy:KgMm2, xz:KgMm2, yy:KgMm2, yz:KgMm2, zz:KgMm2}> {
		if (inertia == null) return null;
		return {xx: new KgMm2(inertia.xx), xy: new KgMm2(inertia.xy), xz: new KgMm2(inertia.xz),
			yy: new KgMm2(inertia.yy), yz: new KgMm2(inertia.yz), zz: new KgMm2(inertia.zz)};
	}
	public final centreOfMass:Vector;
	public final source:MassSource;
	/** Null when a declared mass has no declared inertia tensor. */
	public final inertia:Null<InertiaTensor>;

	public function new(mass:Float, centreOfMass:Vector, source:MassSource, ?inertia:InertiaTensor) {
		this.mass = mass;
		this.centreOfMass = centreOfMass;
		this.source = source;
		this.inertia = inertia;
	}
}
