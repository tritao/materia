package nativekit.sim;

enum abstract ShapeType(Int) from Int to Int {
    var Box = 1;
    var Sphere = 2;
    var Capsule = 3;
    var Plane = 4;
}
