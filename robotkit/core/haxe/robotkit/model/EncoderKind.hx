package robotkit.model;

/** How an encoder reports position: counts since it was powered up, or the position itself. */
enum abstract EncoderKind(String) from String to String {
  /** Quadrature counts from where the joint was when the encoder was powered up. */
  var Incremental = "incremental";
  /** The position itself, in counts from the joint's zero. */
  var Absolute = "absolute";
}
