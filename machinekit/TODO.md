# MachineKit follow-ups

- Evaluate whether code-only components should get a separate saved geometry record. The current generated codec reports them as unsavable.
- Review old MachineKit side records after the version 2 rollout and remove the compatibility reader only when older documents are no longer supported.
- Make the mechanical half of `MachineAssemblyDescription` a compile-time read-only view. `describe()` currently returns a deep, detached copy, and the side arrays use `ReadOnlyArray`, but callers can still mutate the copied `AssemblyDefinition` fields.
- Replace the generic placeholder geometry for nested assembly document instances with an evaluated nested assembly shape. Their placement and connectors are persisted today.
- Add a damaged-document fixture that preserves an owned joint while removing its endpoint. CadKit's normal delete API prevents that state, and `toDefinition()` now raises a typed `AssemblyDocumentDiagnostic` if an imported document contains it.
