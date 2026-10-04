package processkit;

import processkit.tool.WeldSensor.WeldReading;

/** What a welder reports: its latest `tool_weld` reading, however it was sensed. */
interface WelderFeedback {
  function reading():WeldReading;
}
