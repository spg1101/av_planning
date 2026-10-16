## Task

Plan ID: P_._   <!-- e.g. P1.4 -->

## What changed

<!-- 2-4 bullets -->

## Done when

<!-- copy the task's "done when" test and show it passing: test name, metric, screenshot -->

## Metrics (fill whichever apply)

- Localization RMSE: ___ m on bag ___
- Planning cycle time: ___ ms (p95)
- Scenario / lap result: ___

## Checklist

- [ ] Algorithm code lives in `planning_core` and has unit tests
- [ ] `colcon test` passes locally
- [ ] No hard-coded topic names, frames or tuning values in node code
- [ ] No new message types (those go to `av_interfaces`)
- [ ] Docs updated if behavior or interfaces changed
