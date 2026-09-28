package machinekit.motion;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentValue.*;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.Dimension;
import materia.project.MaterialLibrary;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.LinearRailProfile.LinearRailProfileSpec;
import materia.assembly.AssemblyFrames;

/** Matching carriage block for a profile rail. Geometry is a nominal exterior envelope; the
 * two mounting locations and rail axis are exact interface data from the profile catalog. CAD
 * frame: the block is centred on z=0, with its rail-facing side at y=0. */
class LinearRailBlock extends MachineComponent {
	public final spec:LinearRailProfileSpec;

	public static function metric(profile:String):LinearRailBlock
		return new LinearRailBlock(LinearRailSystem.catalog().get(profile));

	public static function custom(spec:LinearRailProfileSpec):LinearRailBlock
		return new LinearRailBlock(spec, true);

	private function new(spec:LinearRailProfileSpec, codeOnly:Bool = false) {
		if (!(spec.blockWidth > 0) || !(spec.blockHeight > 0) || !(spec.blockLength > 0) ||
			!(spec.blockHolePitchB > 0) || spec.blockHolePitchB > spec.blockLength ||
			!(spec.blockHolePitchC > 0) || spec.blockHolePitchC > spec.blockWidth ||
			spec.blockHeight <= spec.railHeight)
			throw 'Linear rail profile "${spec.designation}" has invalid block dimensions';
		var designation = '${spec.family}-${spec.designation}-BLOCK';
		var customName = '${spec.family}-${spec.designation}-BW${Dimension.format(spec.blockWidth)}-BH${Dimension.format(spec.blockHeight)}' +
			'-BL${Dimension.format(spec.blockLength)}-B${Dimension.format(spec.blockHolePitchB)}' +
			'-C${Dimension.format(spec.blockHolePitchC)}-M${spec.blockMountScrew}';
		super(codeOnly ? customDesignation(customName) : designation,
			'${spec.family} ${spec.designation} carriage block', "steel", codeOnly);
		this.spec = spec;
		addConnector("rail", Axis, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, 0));
		var index = 1;
		for (x in [-spec.blockHolePitchC / 2, spec.blockHolePitchC / 2])
			for (z in [-spec.blockHolePitchB / 2, spec.blockHolePitchB / 2])
				addConnector('mount${index++}', Mount,
					AssemblyFrames.alongY(x, spec.blockHeight - spec.railHeight, z, 0, 1, 0));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
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
			[ComponentRecipeSupport.catalog("profile", LinearRailSystem.catalog(), "MGN12C")],
			v -> new LinearRailBlock(LinearRailSystem.catalog().get(v.token("profile"))),
			true);
		return recipeTypeCache;
	}

	override public function componentType():Null<ComponentType> return codeOnly ? null : recipeType();

	override public function values():ComponentValues {
		return new ComponentValues().setToken("profile", this.spec.designation)
			.setToken("material", materialSpec());
	}

}
