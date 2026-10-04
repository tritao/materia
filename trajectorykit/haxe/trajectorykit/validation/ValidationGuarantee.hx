package trajectorykit.validation;

import haxe.Int64;

/** What one validation check establishes for a particular plan. */
enum ValidationGuarantee {
  Proven;
  Sampled(resolutionNs:Int64);
  Unchecked;
  Failed;
}
