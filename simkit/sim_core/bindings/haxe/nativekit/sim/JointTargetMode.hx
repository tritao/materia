package nativekit.sim;

enum abstract JointTargetMode(Int) from Int to Int {
    var Position = 1;
    var Velocity = 2;
    var Effort = 3;
}
