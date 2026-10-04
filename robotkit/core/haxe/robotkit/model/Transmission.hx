package robotkit.model;

/**
 * Kinematic conversion from an actuator coordinate to a joint coordinate.
 * SimpleTransmission uses SI units: joint = offset + actuator / ratio.
 * A rotary actuator driving a prismatic joint therefore has ratio in rad/m;
 * a rotary-to-rotary ratio is dimensionless.
 */
enum Transmission {
  SimpleTransmission(jointId:JointId, ratio:Float, offset:Float);
}
