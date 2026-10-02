package humankit.job;

import humankit.action.HumanAction;
import humankit.HumanLimb;

typedef HumanJobHold = {var action: HumanAction;
var objectId:String;
var grasp:Array<Float>;
var hand:HumanLimb;
}
