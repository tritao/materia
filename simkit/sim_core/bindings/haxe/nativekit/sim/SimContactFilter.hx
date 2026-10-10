package nativekit.sim;

/** Selects how a shape participates in simulation contacts. */
enum abstract SimContactFilter(Int) from Int to Int {
    /** Contacts follow the body's collision layer and mask. */
    var Layers = 0;
    /** Contacts occur only through explicitly declared body pairs. */
    var PairsOnly = 1;
    /** Contacts use explicit pairs and include every environment body. */
    var PairsAndEnvironment = 2;
}
