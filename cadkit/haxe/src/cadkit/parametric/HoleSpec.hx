package cadkit.parametric;

import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * One round hole or recess cut into a part: a cylinder of `diameter` on the axis through (`x`, `y`) of `frame`, spanning
 * `from` to `to` along the frame's +Z, mm. The frame lies in the part's own frame with +Z into the material, so a tool that
 * starts outside the part has `from < 0` and one that leaves the far face runs past it.
 *
 * The same record is what a joint derives for its parts, what a `holes` document feature cuts, and what a hole table is
 * read from, so the drawing, the solid and the checks cannot disagree about a hole.
 */
@:wire typedef HoleSpec = {
  @:id(1) var frame:AssemblyFrame;
  @:id(2) var x:Float;
  @:id(3) var y:Float;
  /** As cut in the solid, mm: the tap drill for a tapped hole. */
  @:id(4) var diameter:Float;
  @:id(5) var from:Float;
  @:id(6) var to:Float;
  /** `tapped`, `clearance`, `counterbore`, `dowel` or `bore`; the thread names the screw. */
  @:id(7) @:optional var kind:String;
  @:id(8) @:optional var thread:String;
  /** Reamed fit (H7 for a dowel) and whether the hole exits the stock. */
  @:id(9) @:optional var fit:String;
  @:id(10) @:optional var through:Bool;
  /**
   * The hole's stable name (`J3.h2.tap`): the cylinder's faces are named under it, so a mate, a drawing or an inspection
   * point finds "the bore of hole 2 of joint J3" whatever else the part's geometry does.
   */
  @:id(11) @:optional var name:String;
  /** The finished size the hole is for (thread major diameter, dowel, bore), mm, when it differs from `diameter`. */
  @:id(12) @:optional var nominal:Float;
};
