# Notes for reviewers

This was a cool assignment — I was genuinely excited to work on it. I was surprised how smoothly the stack ran through several layers of virtualization (WSL → Docker → Gazebo / ArduRover SITL / ROS 2).

There is a lot I would still tweak: cleaner speed near folds, less hand-tuned unstick, a tidier split between tracking and recovery. I tried to follow KISS(Keep It Simple, Stupid) while still scoring well. At some point you have to stop and hand it over, so this is that point.

## Deliverables

- Controller in this repo
- How it works: [REPORT.md](REPORT.md)
- Screen recording of Gazebo + RViz on `2-complicated.path`: [complicated-path-run.mp4](https://drive.google.com/file/d/1QJLcPvn3pPn7JSlc_7JFIZcqCurqgMwI/view?usp=sharing)

## Extra changes (revertible)

Neither commit is part of the controller. Revert if it causes issues.

**WSL2 Docker GPU** (`26cda51`) — Gazebo GPU from Docker-in-WSL. Ubuntu Docker is unchanged unless something breaks.

```bash
git revert 26cda51 && ./docker/build.sh && docker rm -f ardurover-navigation && ./docker/run.sh
```

**Scorer premature-stop** (`bad34c0`) — scoring formula is unchanged. The original scorer could finish early on `2-complicated.path` (self-crossing near the end). Revert, then `./scripts/build.sh` in the container.

```bash
git revert bad34c0
```
