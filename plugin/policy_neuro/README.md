# policy_neuro

StepIt plugin for running neural network-based policy.

Provided interfaces:

- `stepit::neuro_policy::FieldSource`

Provided factories:

- `stepit::Policy`: `neuro`
- `stepit::neuro_policy::FieldSource`:
    - `action_history`
    - `action_filter`
    - `action_reorder`
    - `action_scaling`
    - `actor`
    - `cmd_height`
    - `cmd_pitch`
    - `cmd_roll`
    - `cmd_vel`
    - `estimator`
    - `field_composite`
    - `field_scaling`
    - `heightmap`
    - `joint_reorder`
    - `roll_pitch`
    - `proprioceptor`

## Mechanisms

### FieldSource

Classes derived from `FieldSource` require to declare their input dependencies (`requirements`) and output fields
(`provisions`) by registering named fields through a global FieldManager. At runtime, FieldManager assigns a unique ID
and size to each field, and maintains a registry of source factories. Then the field sources sequentially produce or
process data segments identified by FieldId.

### NeuroPolicy

`NeuroPolicy` orchestrates a set of FieldSource instances into a neural‐network policy pipeline. It:

1. Loads YAML configuration and registers the main `action` field.
2. Reads user‐specified field sources and ensures an actor source is present.
3. Automatically resolves and orders sources based on declared requirements, detecting circular dependencies.
4. Calls `init()` on each source before control loop start.
5. In each `act()` step, sequentially invokes `update()` and `postUpdate()`, assembles the action vector and
   passed it to the `Agent`.
