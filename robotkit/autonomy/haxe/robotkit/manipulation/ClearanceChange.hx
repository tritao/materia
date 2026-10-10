package robotkit.manipulation;

/**
 * A change to the cell during a program (COLLISION.md CL-D6, CL-D9),
 * declared with the program by whoever builds it: a grasp or a place, and
 * contact windows. Bodies are named as the clearance world names them.
 */
enum ClearanceChange {
  /** `body` rides on `link` from now on, where it is (a grasp). */
  Attach(body:String, link:String);
  /** `body` stays where it is, fixed in the cell (a place). */
  Detach(body:String);
  /**
   * The pair keeps at least `margin` (an approach or a retract), or may
   * touch when `margin` is null (the contact itself), until the window
   * closes.
   */
  OpenWindow(a:String, b:String, margin:Null<Float>);
  CloseWindow(a:String, b:String);
}
