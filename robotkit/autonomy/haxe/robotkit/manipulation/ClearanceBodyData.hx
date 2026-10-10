package robotkit.manipulation;

import robotkit.model.LinkId;

/** A convex collision body of the robot's cell: the hull of a part (x, y, z triples in metres, in the frame of the link that carries it). */
typedef ClearanceBodyData = {
  var name:String;
  var link:LinkId;
  var vertices:Array<Float>;
  /** Part of the tool: it may come closer to the work than the arm may, within the contact zone (see `CollisionClearance.violation`). */
  var tool:Bool;
}
