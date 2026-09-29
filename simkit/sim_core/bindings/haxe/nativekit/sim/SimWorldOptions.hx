package nativekit.sim;

/** How a simulation world steps; unset fields keep the defaults. */
typedef SimWorldOptions = {
    ?timestep:Float,
    ?physicsSubsteps:Int,
    ?gravity:Array<Float>,
    /** NKSIM_INTEGRATOR_* and NKSIM_FRICTION_CONE_*; zero keeps the backend default. */
    ?integrator:Int,
    ?frictionCone:Int,
    /** Constraint solver iteration limits; zero keeps the backend default. */
    ?solverIterations:Int,
    ?lineSearchIterations:Int
};
