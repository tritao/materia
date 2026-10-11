package processkit.skill;

import robotkit.skill.Skill;
import processkit.screwdriving.FasteningRunReport;

/** Measured execution and an explicitly skipped final pass expose the same reporting boundary. */
interface FasteningExecution extends Skill {
  function runReport():FasteningRunReport;
}
