package processkit.screwdriving;
/** One consistent live/result frame from the tightening controller. */
interface ScrewdriverFeedback {
  function reading():ScrewdriverReading;
}
