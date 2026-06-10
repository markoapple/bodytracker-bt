# Codex Operating Rule: No Bureaucrat Logic

This repo must keep useful data alive. Do not add gates that turn degraded, partial, fallback, draft, or stale evidence into silence. If the system computes something usable, runtime consumers must be allowed to use it with the right confidence, scope, and status.

## Bureaucrat Logic definition

Bureaucrat Logic is code that accepts or computes a feature, then quietly neuters it through a responsible-sounding flag, hidden override, broad validation rule, stale marker, or post-solve veto. It usually looks reasonable in isolation and breaks the feature in the real data path.

## Patterns to delete, not replicate

1. Precautionary kill switch: fallback or predicted output is computed, then a downstream source/status gate drops it entirely.
2. Nanny-state override: UI/config/user input is accepted, then runtime clamps, rewrites, or ignores it without saying so.
3. Conservative overwrite: a draft, partial state, failed retry, or weak frame erases a previously working accepted state.
4. Proof-of-life requirement: a subsystem must constantly prove it is fresh/ready before output continues, even when last-known-good data can still be emitted.
5. Purity spiral: functional input is rejected because it does not match the preferred architecture or ideal solve shape.
6. Guilt by association: one bad field/sample/tracker marks the whole object/subsystem invalid.
7. Reconsideration loop: a solver produces a result, then a stricter post-check discards it instead of scoring confidence.
8. Phantom authority: a source string, legacy field, external subsystem, or hardcoded constant vetoes the current valid data.
9. Overly strict state machine: progress requires every condition to be perfect instead of using the core sufficient conditions.
10. Telemetry trap: a warning/debug/error label for a recoverable condition becomes a hard runtime failure.

## Required behavior

- Treat `valid`, `ready`, `accepted`, `stale`, `source_known`, `backend_owned`, `degraded`, and `reason` as scoped facts, not universal execution authority.
- Split capability from quality. Example: `usable_for_osc`, `usable_for_projective_depth`, `usable_for_scalar_metric_assist`, `numbers_finite`, `source_label_known`, `confidence`, `age_ms`.
- Preserve last-known-good runtime state unless the user explicitly clears it or the numbers are impossible/non-finite.
- Let degraded data reach output with lower confidence instead of killing the feature.
- Let draft evidence remain draft; do not let it overwrite active calibration unless promoted intentionally.
- Let warnings stay warnings. Do not make cleanup/logging/network/debug failures kill tracking.
- If the UI exposes a setting, runtime must honor it. If runtime cannot honor it, narrow the UI/config contract instead of silently changing the value.

## Review method before each patch

Trace the user-facing claim through the actual runtime path:

config defaults -> config structs -> parsing/validation -> UI controls -> UI save/load payload -> backend command -> runtime branch -> solver/consumer -> telemetry/debug -> tests/docs

A setting existing is not proof. A UI toggle existing is not proof. A fallback state existing is not proof. The proof is that the data reaches the consumer that matters.

When changing behavior, ask one question: does this patch keep useful imperfect data alive, or did it create another ceremonial veto?
