package nativekit.sim;

enum abstract MotionType(Int) from Int to Int {
    var Static = 0;
    var Kinematic = 1;
    var Dynamic = 2;
}
