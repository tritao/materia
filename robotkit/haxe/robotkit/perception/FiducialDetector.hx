package robotkit.perception;

import robotkit.world.SensorFrame;

/** Converts an image-bearing camera frame into camera-relative fiducial poses. */
interface FiducialDetector {
  function detect(frame:SensorFrame):Array<FiducialMarkerObservation>;
}
