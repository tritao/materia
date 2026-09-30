/** How a facility scenario's job learns what the worker stands at and picks up. */
enum abstract FacilityDescription(Int) {
    /** The facility describes its own surfaces and slot items, and the job reads them. */
    var Modelled = 0;
    /** The caller hands the surfaces, part and place point to the job. */
    var Explicit = 1;
    /** Nothing is described: the worker stands where its shoulder reaches and grips plainly. */
    var Undescribed = 2;
}
