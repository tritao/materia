package machinekit.motion;

import cadkit.InertiaTensor;
import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.catalog.PinnedCad;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentView;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.LinearRailProfile.LinearRailProfileSpec;
import materia.assembly.AssemblyFrames;

/**
 * A catalogue carriage block. Its drawing is chosen explicitly and saved with it: `nominal`, the catalogue outline
 * with end caps and tapped holes (`ProfileRailShapes`), or `supplier`, the pinned HIWIN CAD of HG carriages, loaded
 * on demand from the prepared reference directory. The drawing changes neither its part number nor its datasheet
 * mass. The four mounting holes and the rail axis are interface data from the catalogue. CAD frame: centred on
 * z=0, rail top at y=0, mounting face at blockHeight - railHeight.
 */
class LinearRailBlock extends MachineComponent {
	public static inline var NOMINAL = "nominal";
	public static inline var SUPPLIER = "supplier";

	public final spec:LinearRailProfileSpec;
	/** `NOMINAL` or `SUPPLIER`. */
	public final drawing:String;

	public static function metric(profile:String, drawing:String = NOMINAL):LinearRailBlock
		return new LinearRailBlock(LinearRailSystem.catalog().get(profile), drawing);

	public static function custom(spec:LinearRailProfileSpec):LinearRailBlock
		return new LinearRailBlock(spec, NOMINAL, true);

	public static function hasSupplierCad(spec:LinearRailProfileSpec):Bool
		return HgReferenceData.blocks().indexOf(spec.designation) >= 0;

	function new(spec:LinearRailProfileSpec, drawing:String, codeOnly:Bool = false) {
		if (spec.ratings.length == 0) throw 'Linear rail profile "${spec.designation}" needs a published load rating';
		for (rating in spec.ratings)
			if (!(rating.staticLoad > 0) || !(rating.dynamicLoad > 0) || !Math.isFinite(rating.staticLoad) || !Math.isFinite(rating.dynamicLoad))
				throw 'Linear rail profile "${spec.designation}" has an invalid ${rating.source} rating';
		if (!(spec.blockWidth > 0) || !(spec.blockHeight > 0) || !(spec.blockLength > 0) ||
			!(spec.blockHoleSpacingAlong > 0) || spec.blockHoleSpacingAlong > spec.blockLength ||
			!(spec.blockHoleSpacingAcross > 0) || spec.blockHoleSpacingAcross > spec.blockWidth ||
			spec.blockHeight <= spec.railHeight)
			throw 'Linear rail profile "${spec.designation}" has invalid block dimensions';
		if (drawing != NOMINAL && drawing != SUPPLIER) throw 'Unknown carriage drawing "$drawing"';
		if (drawing == SUPPLIER && !hasSupplierCad(spec)) throw 'No pinned supplier CAD for carriage ${spec.designation}';
		var customName = '${spec.family}-${spec.designation}-BW${Dimension.format(spec.blockWidth)}-BH${Dimension.format(spec.blockHeight)}' +
			'-BL${Dimension.format(spec.blockLength)}-A${Dimension.format(spec.blockHoleSpacingAlong)}' +
			'-X${Dimension.format(spec.blockHoleSpacingAcross)}-${spec.blockMountThread}';
		super(codeOnly ? customDesignation(customName) : '${spec.family}-${spec.designation}',
			'${spec.family} ${spec.designation} carriage block', "steel", codeOnly);
		this.spec = spec;
		this.drawing = drawing;
		addConnector("rail", Axis, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, 0));
		// The face a carriage bolts to: +Y (a connector's axis) up out of the block, +Z along the rail.
		var top = spec.blockHeight - spec.railHeight;
		addConnector("top", Mount, AssemblyFrames.translation(0, top, 0));
		var index = 1;
		for (x in [-spec.blockHoleSpacingAcross / 2, spec.blockHoleSpacingAcross / 2])
			for (z in [-spec.blockHoleSpacingAlong / 2, spec.blockHoleSpacingAlong / 2])
				addConnector('mount${index++}', Mount, AssemblyFrames.alongY(x, top, z, 0, 1, 0));
		// Datasheet mass, centred on the outline above the rail top.
		var kg = spec.blockMassKg, w = spec.blockWidth, l = spec.blockLength;
		declareMass(kg, new Vector(0, top / 2, 0),
			new InertiaTensor(kg * (top * top + l * l) / 12, 0, 0, kg * (w * w + l * l) / 12, 0, kg * (w * w + top * top) / 12));
	}

	/** Riding on its rail: mounting face (+Y) up and the travel (Z) running across the view. */
	override public function presentationView():Null<ComponentView>
		return ComponentView.lengthAlongZ();

	/** Full depth of the tapped mounting holes where the table gives it; HGW tables give only the thread. */
	public function mountThreadDepth():Null<Float>
		return spec.blockMountThreadDepth;

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		if (drawing == NOMINAL)
			return detail == Envelope ? outline() : ProfileRailShapes.block(spec);
		var lo = HgReferenceData.low(spec.designation), hi = HgReferenceData.high(spec.designation);
		if (detail == Envelope)
			return Part.prism([new Vector(lo[0], lo[1]), new Vector(hi[0], lo[1]), new Vector(hi[0], hi[1]), new Vector(lo[0], hi[1])], lo[2], hi[2]);
		return new Part(supplierCad(spec.designation));
	}

	/** The prepared HG reference directory: `MATERIA_HG_REFERENCE`, else build/suppliers/vallder/hg-reference in the repository. */
	public static function referenceDirectory():String {
		var configured = Sys.getEnv("MATERIA_HG_REFERENCE");
		if (configured != null && configured != "") return configured;
		var root = Sys.getEnv("MATERIA_ROOT");
		if (root == null || root == "") {
			root = Sys.getCwd();
			while (!sys.FileSystem.exists(root + "/tools/suppliers/hg-reference.json")) {
				var parent = haxe.io.Path.directory(root);
				if (parent == root || parent == "") throw "Set MATERIA_HG_REFERENCE or MATERIA_ROOT to find the HG reference CAD";
				root = parent;
			}
		}
		return root + "/build/suppliers/vallder/hg-reference";
	}

	/** Pinned HG model `model` (carriage or rail), checked against its review digest and frame. The caller owns it. */
	public static function supplierCad(model:String):Shape {
		var path = PinnedCad.require(referenceDirectory() + "/" + model + ".step", HgReferenceData.hash(model),
			"python3 tools/suppliers/hg.py --prepare build/suppliers/vallder/hg-reference");
		var shape = Shape.importStep(path), bounds = shape.bounds(), lo = HgReferenceData.low(model), hi = HgReferenceData.high(model);
		var min = bounds.get_min(), max = bounds.get_max();
		for (pair in [[min.get_x(), lo[0]], [min.get_y(), lo[1]], [min.get_z(), lo[2]], [max.get_x(), hi[0]], [max.get_y(), hi[1]], [max.get_z(), hi[2]]])
			if (Math.abs(pair[0] - pair[1]) > 0.02) {
				shape.close();
				throw 'Pinned $model CAD frame differs from its reviewed interface';
			}
		return shape;
	}

	function outline():Part {
		var top = spec.blockHeight - spec.railHeight;
		return Part.prism([
			new Vector(-spec.blockWidth / 2, 0), new Vector(spec.blockWidth / 2, 0),
			new Vector(spec.blockWidth / 2, top), new Vector(-spec.blockWidth / 2, top),
		], -spec.blockLength / 2, spec.blockLength / 2);
	}

	private static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null)
			recipeTypeCache = new ComponentType("machinekit.motion.linear-rail-block",
				[ComponentRecipeSupport.catalog("profile", LinearRailSystem.catalog(), "MGN12C"),
					ComponentRecipeSupport.choice("geometry", [NOMINAL, SUPPLIER], NOMINAL)],
				v -> new LinearRailBlock(LinearRailSystem.catalog().get(v.token("profile")), v.token("geometry")),
				true, false, ["geometry"]);
		return recipeTypeCache;
	}

	override public function componentType():Null<ComponentType> return codeOnly ? null : recipeType();

	override public function values():ComponentValues {
		return new ComponentValues().setToken("profile", this.spec.designation).setToken("geometry", drawing)
			.setToken("material", materialSpec());
	}

}
