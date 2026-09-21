package nativekit.sim;

enum abstract JointType(Int) from Int to Int {
    var Fixed = 1;
    var Revolute = 2;
    var Prismatic = 3;
}
