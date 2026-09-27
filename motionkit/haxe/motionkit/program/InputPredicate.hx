package motionkit.program;

import motionkit.event.EventValue;

/** Serializable condition used by a WaitInput barrier. */
enum InputPredicate {
  Equals(value:EventValue);
}
