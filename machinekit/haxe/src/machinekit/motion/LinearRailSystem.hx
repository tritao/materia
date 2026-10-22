package machinekit.motion;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.Bom;
import machinekit.component.MachineComponent;
import machinekit.motion.LinearRailProfile.LinearRailProfileSpec;
import machinekit.motion.LinearRailProfile.LinearRailRating;

/** Catalog-backed profile-rail guide assembly. This is an alternative to the round-rod
 * `LinearGuideSystem`: the rail is fixed and one or more blocks travel along its axis. Rows are
 * keyed by carriage designation; each names its rail, the tables it comes from, and the load
 * ratings published for it. */
class LinearRailSystem {
	static var table:Null<Catalog<LinearRailProfileSpec>>;

	public static inline var HIWIN_MG = "https://damencnc.com/userdata/file/3455-3_HIWIN_MGN_and_MGW_Product_Catalog_2023.pdf";
	public static inline var HIWIN_2008 = "https://www.lm76.com/Hiwin%20Linear%20Guideway%20Catalog_G99TE13-0809_opt.pdf";
	public static inline var HIWIN_2024 = "https://www.hiwin.com/wp-content/uploads/HIWIN-Linear-Guideway-Catalog.pdf";

	static final HG_TABLE_FIELDS = ["railWidth", "railHeight", "blockWidth", "blockHeight", "blockLength", "blockHoleSpacingAlong",
		"blockHoleSpacingAcross", "blockMountThread", "railHolePitch", "railEndMargin", "railMountScrew", "railBore", "blockMassKg",
		"railMassKgPerMetre", "ratings"];

	public final spec:LinearRailProfileSpec;
	public final rail:LinearRail;
	public final blocks:Array<LinearRailBlock>;
	public final railLength:Float;
	public final blockCount:Int;
	public final travelMin:Float;
	public final travelMax:Float;
	public final stroke:Float;

	static function hiwinMg(c:Float, c0:Float):LinearRailRating
		return {source: "HIWIN MG catalogue 2023", reference: HIWIN_MG, dynamicLoad: c, staticLoad: c0, lifeBasisKm: 50, original: true};

	/** HIWIN's MGN miniature guides: dimensions, masses and ratings from the 2023 MG catalogue. */
	static function miniature():Array<LinearRailProfileSpec> {
		var mgnr12 = {through: 3.5, counterbore: 6.0, depth: 4.5}, mgnr9 = {through: 3.5, counterbore: 6.0, depth: 3.5};
		return [{
			designation: "MGN12C", rail: "MGNR12R", family: "HIWIN",
			railWidth: 12, railHeight: 8, blockWidth: 27, blockHeight: 13, blockLength: 34.7,
			blockHoleSpacingAlong: 15, blockHoleSpacingAcross: 20, blockMountThread: "M3", blockMountThreadDepth: 3.5,
			railHolePitch: 25, railEndMargin: 10, railMountScrew: "M3x8", railBore: mgnr12,
			blockMassKg: 0.034, railMassKgPerMetre: 0.65, ratings: [hiwinMg(2840, 3920)]
		}, {
			designation: "MGN12H", rail: "MGNR12R", family: "HIWIN",
			railWidth: 12, railHeight: 8, blockWidth: 27, blockHeight: 13, blockLength: 45.4,
			blockHoleSpacingAlong: 20, blockHoleSpacingAcross: 20, blockMountThread: "M3", blockMountThreadDepth: 3.5,
			railHolePitch: 25, railEndMargin: 10, railMountScrew: "M3x8", railBore: mgnr12,
			blockMassKg: 0.054, railMassKgPerMetre: 0.65, ratings: [hiwinMg(3720, 5880)]
		}, {
			designation: "MGN9C", rail: "MGNR9R", family: "HIWIN",
			railWidth: 9, railHeight: 6.5, blockWidth: 20, blockHeight: 10, blockLength: 28.9,
			blockHoleSpacingAlong: 10, blockHoleSpacingAcross: 15, blockMountThread: "M3", blockMountThreadDepth: 3,
			railHolePitch: 20, railEndMargin: 7.5, railMountScrew: "M3x6", railBore: mgnr9,
			blockMassKg: 0.016, railMassKgPerMetre: 0.38, ratings: [hiwinMg(1860, 2550)]
		}];
	}

	/**
	 * HIWIN's own figures where we have them, after the Vallder rating that sizing uses: Vallder (and the HG clones
	 * reviewed with it) reproduce the 2008 edition, which the 2024 edition raises by 30 to 50 %.
	 */
	static function hiwinRatings(designation:String):Array<LinearRailRating> {
		function edition(source:String, reference:String, c:Float, c0:Float):LinearRailRating
			return {source: source, reference: reference, dynamicLoad: c, staticLoad: c0, lifeBasisKm: 50, original: true};
		function both(c2008:Float, c02008:Float, c2024:Float, c02024:Float):Array<LinearRailRating>
			return [edition("HIWIN G99TE13-0809", HIWIN_2008, c2008, c02008), edition("HIWIN G99TE24-2410", HIWIN_2024, c2024, c02024)];
		return switch designation {
			case "HGH15CA", "HGW15CC": both(11380, 16970, 14700, 23470);
			case "HGH20CA": both(17750, 27760, 27100, 36680);
			case "HGH25CA", "HGW25CC": both(26480, 36490, 34900, 52820);
			case _: [];
		};
	}

	static function rows():Array<LinearRailProfileSpec> {
		var result = miniature();
		for (model in HgReferenceData.blocks()) {
			var spec = HgReferenceData.carriage(model);
			spec.ratings = spec.ratings.concat(hiwinRatings(model));
			if (model == "HGH25CA") spec.preloadTarget = "Z1 target, assumed; verify supplier preload and friction";
			result.push(spec);
		}
		return result;
	}

	/** Profile-rail carriages and their rails: HIWIN MGN miniature guides and the HG family. */
	public static function catalog():Catalog<LinearRailProfileSpec> {
		if (table == null)
			table = new Catalog("linear rail profile", spec -> spec.designation, rows(), spec -> StringTools.startsWith(spec.designation, "MGN") ? ({
				source: "HIWIN MGN/MGW catalogue: " + HIWIN_MG, standard: null, standardEdition: null, dimensionKind: Nominal,
				conformance: NominalEnvelope,
				// Checked against the catalogue for MGN12C only; the other miniature rows await the same review.
				verifiedFields: spec.designation == "MGN12C" ? ["railWidth", "railHeight", "blockWidth", "blockHeight", "blockLength",
					"blockMountThread", "railHolePitch", "railEndMargin", "railMountScrew"] : []
			}) : ({
				source: "Vallder HG tables (" + spec.ratings[0].reference + ", https://vallder.com/product/hgr-linear-guideways/)",
				sources: ["Pinned HIWIN reference CAD (tools/suppliers/hg-reference.json)"],
				standard: null, standardEdition: null, dimensionKind: Nominal, conformance: NominalEnvelope,
				// tools/suppliers/hg.py emits every one of these from the reviewed table rows.
				verifiedFields: HG_TABLE_FIELDS
			}));
		return table;
	}

	/** The rating sizing uses: the first one listed, the purchase source's. */
	public static function rating(spec:LinearRailProfileSpec):LinearRailRating
		return spec.ratings[0];

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
