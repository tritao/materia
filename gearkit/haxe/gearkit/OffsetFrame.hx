package gearkit;

/** The frame in which a flexspline centre offset (`FlexOffset`) is fixed. */
enum OffsetFrame {
  /** Fixed in the circular-spline frame: the flexspline axis is displaced from the circular-spline axis. */
  Fixed;
  /** Rotates with the wave generator (the major axis): a change of the cam amplitude along one axis, or, with the
      centre distance as magnitude, the orbit of a rigid planet. */
  Carrier;
  /** Rotates with the flexspline's mean rotation: tooth-ring runout, eccentricity of the flexspline about its own axis. */
  Flexspline;
}
