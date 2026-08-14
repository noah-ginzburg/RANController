# Docs

These are the task guides for this workspace. Each one is written to answer a
question you'd actually show up with, rather than to describe a package.

## Index

| Guide | Answers |
|---|---|
| [`simulation.md`](simulation.md) | How do I run this without hardware? |
| [`real-flight.md`](real-flight.md) | How do I fly an actual drone, safely? |
| [`hardware-drone.md`](hardware-drone.md) | Radio channel/address, firmware, batteries |
| [`vicon-setup.md`](vicon-setup.md) | Mocap subjects, markers, network |
| [`troubleshooting.md`](troubleshooting.md) | Symptom → cause → fix |
| [`analysis.md`](analysis.md) | Bags, CSV export, plotting |

## Adding a guide

Copy [`_TEMPLATE.md`](_TEMPLATE.md), write the guide, and add a row to the table
above. Try to keep one topic per file, and split a file once it grows past what
you'd comfortably read in one sitting.

## Conventions

These conventions exist for a reason: docs that ignore them go stale quietly,
and then mislead someone at 2am with a drone in the air.

- **Anchor claims to `file:line`.** Prefer ``the interlock is `launch_requested`
  (`controller_server.py:529`)`` over a paragraph that paraphrases the code. It
  lets readers verify what you wrote, and it makes stale docs easy to spot.
- **Document *why*, and link to *what*.** The code already explains what it
  does. What docs can add is the reasoning that isn't in the code — usually the
  failure that motivated the design in the first place. Why-docs also age far
  more slowly than what-docs.
- **Show what good looks like.** Paste the expected output. Someone needs to be
  able to tell a working scan from a broken one without already knowing the
  answer.
- **Date any hardware finding.** Something like `2026-07-17: confirmed flashed
  with 2026.04` is honest about being a point-in-time observation. Undated
  hardware claims eventually turn into lies.
- **Mark gaps with `TODO`** instead of guessing. A hole you know about is much
  better than a confident wrong answer.
