package humankit;

import humankit.action.HumanAction;

typedef HumanJobHold = {var action: HumanAction;
var objectId:String;
var grasp:Array<Float>;
var hand:HumanLimb;
}
