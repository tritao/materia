package gearkit;

/** Where a point lies on one tooth, by its radius: the root blend (below the flank, inside the root
    join), the working flank (the involute or arc between the root join and the tip-blend start), or
    the tip (the tip blend and the land). See `ToothProfile.regionAt`. */
enum ToothRegion {
  Root;
  Flank;
  Tip;
}
