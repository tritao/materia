package machinekit.standard;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.catalog.Catalog;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Rectangular key stock sized by DIN 6885-1. Square ends; rounded-end (form B) stock is not modelled.
 * CAD frame: width along X (centred), height along Y with the seat floor at y=0, length along Z.
 * Connector: `seat` (face) at the bottom, mid-length, with +Y along +Z as usual. A `SteppedShaft`
 * keyway connector shares this convention, so mating the two with a fixed joint seats the key flush.
 */
class ParallelKey extends MachineComponent {
	static var table:Null<Catalog<ParallelKeySpec>>;

	public final spec:ParallelKeySpec;
	public final length:Float;

	static function rows():Array<ParallelKeySpec>
		return [
			{maxShaft: 8, width: 2, height: 2, shaftDepth: 1.2, hubDepth: 1.0},
			{maxShaft: 10, width: 3, height: 3, shaftDepth: 1.8, hubDepth: 1.4},
			{maxShaft: 12, width: 4, height: 4, shaftDepth: 2.5, hubDepth: 1.8},
			{maxShaft: 17, width: 5, height: 5, shaftDepth: 3.0, hubDepth: 2.3},
			{maxShaft: 22, width: 6, height: 6, shaftDepth: 3.5, hubDepth: 2.8},
			{maxShaft: 30, width: 8, height: 7, shaftDepth: 4.0, hubDepth: 3.3},
			{maxShaft: 38, width: 10, height: 8, shaftDepth: 5.0, hubDepth: 3.3},
		];

	public static function catalog():Catalog<ParallelKeySpec> {
		if (table == null)
			table = new Catalog("parallel key", spec -> '${spec.width}x${spec.height}', rows());
		return table;
	}

	public static function metric(size:String, length:Float):ParallelKey
		return new ParallelKey(catalog().get(size), length);

	/** Key sized for the smallest DIN 6885-1 range that covers `shaftDiameter`. */
	public static function forShaft(shaftDiameter:Float, length:Float):ParallelKey {
		if (!(shaftDiameter > 0)) throw "Key needs a positive shaft diameter";
		for (designation in catalog().designations()) {
			var spec = catalog().get(designation);
			if (shaftDiameter <= spec.maxShaft) return new ParallelKey(spec, length);
		}
		throw 'No DIN 6885-1 key fits shaft diameter $shaftDiameter';
	}

	public function new(spec:ParallelKeySpec, length:Float) {
		if (!(length > 0) || !Math.isFinite(length)) throw "Key needs a positive length";
		var size = '${Dimension.format(spec.width)}x${Dimension.format(spec.height)}';
		super('DIN6885-${size}x${Dimension.format(length)}', 'Parallel key $size, ${Dimension.format(length)} mm long',
			"steel C45");
		this.spec = spec;
		this.length = length;
		addConnector("seat", Face, Solids.axial(0, 0, length / 2));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.prism([
			new Vector(-spec.width / 2, 0), new Vector(spec.width / 2, 0),
			new Vector(spec.width / 2, spec.height), new Vector(-spec.width / 2, spec.height),
		], 0, length);
}
