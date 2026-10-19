package machinekit.motion;

/** Input/output shaft reduction, independent of its housing geometry. */
typedef ReducerSpec = {
    var ratio:Float;
    var efficiency:Float;
    var inputInertia:Float;
    var inertiaAssumed:Bool;
    var assumed:Bool;
    var diameter:Float;
    var length:Float;
    var bore:Float;
}
