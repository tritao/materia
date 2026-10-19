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
    ?lineSearchIterations:Int,
    /** Friction/normal constraint impedance ratio; zero keeps the backend default (MuJoCo). */
    ?frictionImpedanceRatio:Float,
    /** Friction-only no-slip postprocessing iterations; zero disables it (MuJoCo). */
    ?noslipIterations:Int,
    /** Optional arena+stack budget in bytes (MuJoCo); zero keeps its default. */
    ?arenaMemoryBytes:Int,
    /** Constraint optimizer termination tolerance; zero keeps its defaults (MuJoCo). */
    ?solverTolerance:Float
};
