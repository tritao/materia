package machinekit.hmi;

import machinekit.catalog.CatalogMetadata;
import machinekit.component.ComponentValues;

/** One purchasable operator control: its designation, the recipe inputs that describe it, and where they came from. */
typedef ControlRow = {
  var designation:String;
  /** The recipe inputs this part sets; the rest keep the recipe's defaults. */
  var values:ComponentValues;
  var metadata:CatalogMetadata;
}
