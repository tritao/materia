package gearkit;

/** The tooth form of a circular spline given as a sampled space (`SweptSpace`), for example the envelope of a swept
    flexspline tooth (TOOTH_DESIGN_PLAN.md TD-D3). It is an internal-gear form: `GearSpec(module, teeth, width, form, true)`.
    `tipRadius` is where the circular-spline tooth is cut off (its smallest radius); `rootRadius` is the nominal root
    circle. The space is sampled only up to its largest radius, so the root blend between there and `rootRadius` is an
    assumed radial wall, not a generated trochoid (see `SampledInternalProfile`). */
class SampledInternalForm implements ToothForm {
  public final space:SweptSpace;
  public final module:Float;
  public final tipRadius:Float;
  public final rootRadius:Float;

  public function new(space:SweptSpace, module:Float, tipRadius:Float, rootRadius:Float) {
    if (!Math.isFinite(module) || module <= 0) throw "sampled internal form needs a positive module";
    if (!(tipRadius >= space.minRadius() && tipRadius < space.maxRadius())) throw "tooth tip radius must lie inside the sampled space";
    if (!(rootRadius >= space.maxRadius())) throw "root radius must not be below the largest sampled radius";
    this.space = space;
    this.module = module;
    this.tipRadius = tipRadius;
    this.rootRadius = rootRadius;
  }

  public function name():String return "sampled-internal";

  function pitchRadius():Float return module * space.teeth / 2;

  /** Local pressure angle at the pitch radius, from the sampled flank. */
  public function pressureAngle():Float {
    var spec = new GearSpec(module, space.teeth, 1, this, true);
    var p = spec.profile();
    return Math.acos(p.normalLever(pitchRadius()) / pitchRadius());
  }

  public function addendum(m:Float):Float return pitchRadius() - tipRadius;
  public function dedendum(m:Float):Float return rootRadius - pitchRadius();

  public function pitchToothThickness(spec:GearSpec):Float return space.toothThickness(spec.pitchRadius());

  public function validate(spec:GearSpec):Void {
    if (!spec.internal) throw "a sampled circular-spline form is internal";
    if (spec.teeth != space.teeth) throw "tooth count differs from the sampled space";
    if (spec.module != module) throw "module differs from the sampled space";
    if (spec.profileShift != 0) throw "a sampled form carries its own tooth thickness; profile shift must be zero";
  }

  public function profile(spec:GearSpec):ToothProfile return new SampledInternalProfile(spec, this);
}
