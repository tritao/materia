package machinekit.motion;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.Bom;
import machinekit.component.MachineComponent;
import machinekit.motion.LinearRailProfile.LinearRailProfileSpec;

/** Catalog-backed profile-rail guide assembly. This is an alternative to the round-rod
 * `LinearGuideSystem`: the rail is fixed and one or more blocks travel along its axis. The
 * profile and mounting data are explicit, while the generated rail and block solids remain
 * nominal envelopes suitable for layout and clearance checks. */
class LinearRailSystem {
	static var table:Null<Catalog<LinearRailProfileSpec>>;

	public final spec:LinearRailProfileSpec;
	public final rail:LinearRail;
	public final blocks:Array<LinearRailBlock>;
	public final railLength:Float;
	public final blockCount:Int;
	public final travelMin:Float;
	public final travelMax:Float;
	public final stroke:Float;

	static function rows():Array<LinearRailProfileSpec>
		return [{
			designation: "MGN12C", family: "HIWIN",
			railWidth: 12, railHeight: 8, blockWidth: 27, blockHeight: 13, blockLength: 34.7,
			blockHolePitchB: 20, blockHolePitchC: 15, blockMountScrew: "M3x8", railHolePitch: 25, railEndMargin: 10,
			railMountScrew: "M3x8", staticLoadCapacity: 2600
		}, {
			designation: "HGR15", family: "GENERIC-HG",
			railWidth: 15, railHeight: 15, blockWidth: 34, blockHeight: 28, blockLength: 61.4,
			blockHolePitchB: 26, blockHolePitchC: 26, blockMountScrew: "M4x10", railHolePitch: 60, railEndMargin: 20,
			railMountScrew: "M4x16", staticLoadCapacity: 10000
		}, {
			// SR1 reference envelopes and capacities, all unverified assumptions.
			designation: "HGR20", family: "GENERIC-HG",
			railWidth: 20, railHeight: 17.5, blockWidth: 44, blockHeight: 30, blockLength: 77.5,
			blockHolePitchB: 32, blockHolePitchC: 36, blockMountScrew: "M5x12", railHolePitch: 60, railEndMargin: 20,
			railMountScrew: "M5x20", staticLoadCapacity: 27000
		}, {
			designation: "HGR25", family: "GENERIC-HG",
			railWidth: 23, railHeight: 22, blockWidth: 48, blockHeight: 40, blockLength: 84,
			blockHolePitchB: 35, blockHolePitchC: 35, blockMountScrew: "M6x16", railHolePitch: 60, railEndMargin: 20,
			railMountScrew: "M6x25", staticLoadCapacity: 36000
		}, {
			// The long carriage of the MGN12 rail: same rail, a longer block for stiffness. Assumed envelope;
			// static capacity scaled from the assumed MGN12C value by the H/C block ratio (about 1.5).
			designation: "MGN12H", family: "HIWIN",
			railWidth: 12, railHeight: 8, blockWidth: 27, blockHeight: 13, blockLength: 45.4,
			blockHolePitchB: 20, blockHolePitchC: 15, blockMountScrew: "M3x8", railHolePitch: 25, railEndMargin: 10,
			railMountScrew: "M3x8", staticLoadCapacity: 3900
		}, {
			// The miniature MGN9 rail and its short carriage. Assumed envelope; static capacity scaled from
			// the assumed MGN12C value (about 0.55 of it).
			designation: "MGN9C", family: "HIWIN",
			railWidth: 9, railHeight: 6.5, blockWidth: 20, blockHeight: 10, blockLength: 28.9,
			blockHolePitchB: 10, blockHolePitchC: 15, blockMountScrew: "M3x8", railHolePitch: 20, railEndMargin: 7.5,
			railMountScrew: "M3x6", staticLoadCapacity: 1400
		}];

	/** HIWIN MGN/MGW linear guideway dimensions used by the MGN12C profile row. */
	public static function catalog():Catalog<LinearRailProfileSpec> {
		if (table == null)
			table = new Catalog("linear rail profile", spec -> spec.designation, rows(), spec -> (spec.designation == "HGR15" || spec.designation == "HGR20" || spec.designation == "HGR25") || spec.designation == "MGN12H" || spec.designation == "MGN9C" ? ({
				source: spec.designation == "HGR15" ? "Assumed HGR15/HGH15CA reference envelope and 10 kN static block capacity; not vendor verified"
					: "Assumed " + spec.designation + " reference envelope and static block capacity; not vendor verified",
				standard: null, standardEdition: null, dimensionKind: Nominal, conformance: GenericApproximation,
				verifiedFields: []
			}) : ({
				source: "Dimensions: https://www.hiwin.com/wp-content/uploads/HIWIN-Linear-Guideway-Catalog.pdf; assumed 2.6 kN static block capacity, not vendor verified",
				standard: null, standardEdition: null, dimensionKind: Nominal, conformance: NominalEnvelope,
				verifiedFields: ["railWidth", "railHeight", "blockWidth", "blockHeight", "blockLength",
					"blockMountScrew", "railHolePitch", "railEndMargin", "railMountScrew"]
			}));
		return table;
	}

	/** Construct a rail and one or more matching blocks from a catalog designation. */
	public static function forProfile(designation:String, railLength:Float, blockCount:Int = 1):LinearRailSystem
		return new LinearRailSystem(catalog().get(designation), railLength, blockCount);

	public function new(spec:LinearRailProfileSpec, railLength:Float, blockCount:Int = 1) {
		if (blockCount < 1) throw "Linear rail guide needs at least one block";
		if (!(railLength > spec.railEndMargin * 2 + spec.blockLength))
			throw "Linear rail length must leave room for a block and both end margins";
		this.spec = spec;
		this.railLength = railLength;
		this.blockCount = blockCount;
		travelMin = spec.railEndMargin + spec.blockLength / 2;
		travelMax = railLength - spec.railEndMargin - spec.blockLength / 2;
		stroke = travelMax - travelMin;
		if (!(stroke > 0)) throw "Linear rail guide needs a positive stroke";
		for (i in 0...blockCount)
			if (initialTravel(i) > travelMax + 1e-9) throw "Linear rail is too short for the requested blocks";
		rail = LinearRail.metric(spec.designation, railLength);
		blocks = [for (i in 0...blockCount) LinearRailBlock.metric(spec.designation)];
	}

	function initialTravel(index:Int):Float
		return travelMin + index * spec.blockLength;

	/** Build a kinematic guide model with one prismatic joint per block. Joint coordinates are
	 * measured from the rail's axis connector and constrained to the usable stroke. */
	public function assembly():AssemblyModel {
		var model = new AssemblyModel();
		rail.addTo(model, "rail");
		for (i in 0...blocks.length) {
			var id = 'block${i + 1}';
			blocks[i].addTo(model, id);
			model.mateOnAxis('$id-slide', "prismatic", "rail", "axis", id, "rail",
				{x: 0, y: 1, z: 0}, initialTravel(i),
				{lower: travelMin, upper: travelMax, velocity: null, effort: null});
		}
		return model;
	}

	/** Move one block to a distance along the rail. */
	public function setTravel(state:AssemblyState, travel:Float, blockIndex:Int = 0):Void {
		if (blockIndex < 0 || blockIndex >= blockCount) throw "Linear rail block index is out of range";
		state.setJoint('block${blockIndex + 1}-slide', travel);
	}

	public function bom():Bom {
		var result = new Bom();
		result.addComponent(rail);
		for (block in blocks) result.addComponent(block);
		return result;
	}

	/** Components and stable instance ids for geometry exporters. */
	public function components():Array<{id:String, component:MachineComponent}> {
		var result:Array<{id:String, component:MachineComponent}> = [{id: "rail", component: rail}];
		for (i in 0...blocks.length) result.push({id: 'block${i + 1}', component: blocks[i]});
		return result;
	}
}
