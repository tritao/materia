package machinekit.component;

/**
 * How a component is best looked at on its own (catalogue thumbnails, part previews): which of its local
 * directions points up on screen, and which faces the viewer's side. A component declares one through
 * `MachineComponent.presentationView` when its shape alone does not say; otherwise `derive` chooses from its
 * bounds. Directions are unit vectors in the component's CAD frame and must be perpendicular.
 */
class ComponentView {
	/** Shown pointing up. */
	public final up:Array<Float>;

	/** Shown facing the viewer's side; the camera then looks at the part from a little to one side and above. */
	public final front:Array<Float>;

	public function new(up:Array<Float>, front:Array<Float>) {
		var u = unit(up), f = unit(front);
		if (Math.abs(u[0] * f[0] + u[1] * f[1] + u[2] * f[2]) > 1e-9)
			throw "A component view needs perpendicular up and front directions";
		this.up = u;
		this.front = f;
	}

	/** Standing as modelled: +Z up, -Y towards the viewer. */
	public static function upright():ComponentView
		return new ComponentView([0, 0, 1], [0, -1, 0]);

	/**
	 * Lying down for a part that runs along its local +Z with its working face towards +Y (rails, racks, screws,
	 * profiles, keys): +Y up and the length running across the view.
	 */
	public static function lengthAlongZ():ComponentView
		return new ComponentView([0, 1, 0], [1, 0, 0]);

	/** The view for a part with these CAD bounds: lying down when it is much longer along Z than across, else upright. */
	public static function derive(minimum:Array<Float>, maximum:Array<Float>):ComponentView {
		var x = maximum[0] - minimum[0], y = maximum[1] - minimum[1], z = maximum[2] - minimum[2];
		return z > 2.5 * Math.max(x, y) ? lengthAlongZ() : upright();
	}

	static function unit(v:Array<Float>):Array<Float> {
		var length = v.length == 3 ? Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) : 0;
		if (!(length > 0))
			throw "A component view direction needs three finite components";
		return [v[0] / length, v[1] / length, v[2] / length];
	}
}
