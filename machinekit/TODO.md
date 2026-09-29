# MachineKit follow-ups

- Evaluate whether code-only components should get a separate saved geometry record. The current generated codec reports them as unsavable.
- Review old MachineKit side records after the version 2 rollout and remove the compatibility reader only when older documents are no longer supported.
- Make the records inside the mechanical description's `ReadOnlyArray` fields read-only too. The arrays now reject mutation at compile time, but a caller can still edit a field on an occurrence or joint in its detached copy.
