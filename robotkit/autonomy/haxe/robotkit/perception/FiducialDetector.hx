package robotkit.perception;



import robotkit.core.SensorFrame;

/** Converts an image-bearing camera frame into camera-relative fiducial poses. */
interface FiducialDetector {
  function detect(frame:SensorFrame):Array<FiducialMarkerObservation>;
}
