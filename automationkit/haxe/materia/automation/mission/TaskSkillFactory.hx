package materia.automation.mission;

import materia.automation.facility.Facility;
import materia.automation.task.Task;
import robotkit.skill.Skill;
import robotkit.world.Robot;

/** Explicit application adapter from facility task data to configured RobotKit skills. */
interface TaskSkillFactory {
  function create(task:Task, robot:Robot, facility:Facility):Skill;
}
