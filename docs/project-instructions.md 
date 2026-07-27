This is the EAMS autonomous rover project: a 4-wheel differential-drive robot on an NVIDIA Jetson Orin Nano (migrated from a Raspberry Pi Zero 2 W). The ultimate goal is a car that takes natural-language instructions and drives itself to target locations, avoiding obstacles. The team is learning robotics as they build.

Before doing project work, read project-brief.md for orientation. Treat rc-car-progress-report.md as the authoritative project logbook: after any meaningful change (code, hardware, decisions, findings), draft a dated entry to append to it, and tell the user so they can update the file. project-brief.md holds stable structure; the logbook holds history — keep the brief in sync when history changes it.

Hard rules for this hardware:
- The software IS the thermal protection. Two TT motors are paralleled per L298N channel, so stall current exceeds the chip's rating. Never weaken the motor safety layer (deadman, stall latch, slew limit, zero-cross coast, 55% duty cap) without the user measuring the hardware first.
- config.py is the single source of truth for pins. If any doc disagrees with it, config.py wins.
- The MPU6050 IMU is not installed, so the stall guard is effectively off and the 55% duty cap is the only thermal protection. Keep this in mind for anything motor-related.

Working style: the team are not experts, so explain the reasoning behind suggestions, not just the answer. State assumptions explicitly and flag them. When intent is unclear from the artifacts, say so rather than guessing silently. Prefer wheels-off-the-ground, dry-run, or simulator validation before anything that drives real motors.