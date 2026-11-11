package cadkit.parametric;

import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * One hole in a part: its mouth at (`x`, `y`) on the face `frame` (in the part's own frame, +Z into the material), `depth` mm
 * deep along +Z, cut at `diameter`. A `through` hole leaves the far face at `depth`; a `recess` (a counterbore for a screw head)
 * is opened from that far face back into the hole.
 *
 * It describes the hole, not the tool: how a cutter approaches and leaves the stock is `HolesFeature`'s business. The same
 * record is what a joint derives for its parts, what a `holes` document feature cuts and what a hole table is read from, so
 * the drawing, the solid and the checks cannot disagree about a hole.
 */
@:wire typedef HoleSpec = {
  @:id(1) var frame:AssemblyFrame;
  @:id(2) var x:Float;
  @:id(3) var y:Float;
  /** As cut in the solid, mm: the tap drill for a tapped hole. */
  @:id(4) var diameter:Float;
  @:id(5) var depth:Float;
  @:id(6) @:optional var through:Bool;
  /** `tapped`, `clearance`, `dowel` or `bore`; the thread names the screw. */
  @:id(7) @:optional var kind:String;
  @:id(8) @:optional var thread:String;
  /** Reamed fit (H7 for a dowel). */
  @:id(9) @:optional var fit:String;
  /**
   * The hole's stable name (`J3.h2.clearance`): its faces are named under it, so a mate, a drawing or an inspection point finds
   * "the bore of hole 2 of joint J3" whatever else the part's geometry does.
   */
  @:id(10) @:optional var name:String;
  /** The finished size the hole is for (thread major diameter, dowel, bore), mm, when it differs from `diameter`. */
  @:id(11) @:optional var nominal:Float;
  @:id(12) @:optional var recess:HoleRecess;
};

/** A counterbore opened from a through hole's far face: `diameter` wide, `depth` back into the hole, named on its own. */
@:wire typedef HoleRecess = {
  @:id(1) var diameter:Float;
  @:id(2) var depth:Float;
  @:id(3) @:optional var name:String;
};
