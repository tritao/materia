package robotkit.perception;



import robotkit.mobile.Pose2;
import robotkit.core.SensorFrame;

/**
 * Rewrites a sensor frame before perception reads it, given where its sensor stands in the
 * localization reference frame; `FrameAwarePerception` applies it to the frames it `applies` to.
 */
interface ScanFilter {
  function applies(frame:SensorFrame):Bool;
  function filter(frame:SensorFrame, referenceFromSensor:Pose2):SensorFrame;
}
