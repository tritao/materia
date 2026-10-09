package motionkit.robot;

/**
 * A joint that follows several axes, such as a CoreXY motor's pulley, which turns with x and with y. No axis
 * owns it: its trajectory is the sum of its terms, `ratio * leader + offset`, taken from the joints it follows
 * (axis joints or other derived joints), which are always earlier in the list.
 */
typedef DerivedJoint = {
  var joint:Int;
  var terms:Array<{leader:Int, ratio:Float, offset:Float}>;
}
