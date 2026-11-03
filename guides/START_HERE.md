# Robocurve workspace guide

To start from a fresh clone of this fork's guide branch:

```sh
git clone --branch dev https://github.com/Ardal-ON/benchmark-tasks.git
cd benchmark-tasks
```

A normal clone follows the fork's default branch, so explicitly select `dev` to see these guides. Keep `dev` as the guide worktree; create each submission in a separate worktree from the latest upstream `main` using [the clean task-branch instructions](agent_workflow.md#create-a-clean-task-branch).

This branch contains reusable guidance for task development. The YAM Arena package and robot assets are not vendored here; obtain them from their source repository or the configured Brev environment. For a new task, begin with [Agent workflow](agent_workflow.md). It covers obtaining the benchmark fork, designing and validating a task, running the Brev simulation, preparing media, and submitting when authorized.

| Resource | Purpose |
| --- | --- |
| [Agent workflow](agent_workflow.md) | Practical end-to-end workflow and lessons from the conveyor task |
| [Submission notes](submission.md) | Short submission checklist; the benchmark repository's current `AGENTS.md` is authoritative |
| [Simulation walkthrough](simulation_walkthrough.md) | Supplied Brev and Inspect Robots walkthrough |
| [YAM Arena reference](README.md) | Supplied package README; source, assets and tools live in the linked package repository |
| [Task example](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/examples/stack_cubes.py) | Minimal scene, scoring and scripted-demo example |
| [Tests](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730/tests) and [scripts](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730/scripts) | Offline and Isaac Sim validation examples |
| [Benchmark repository](gitrepo.md) | Upstream submission repository |

This is the fork’s guide-only `dev` branch. Keep it separate from task submissions. A task belongs in one folder under `tasks/` on a new branch based on the latest upstream `main`, not on `dev` or a previous task branch. Inspect existing worktrees, remotes, branch status and existing work first. Preserve tasks already present in upstream `main`; do not add another task to an existing task branch or PR.

Keep team credentials in a private file outside the checkout and set `ROBOCURVE_CREDENTIALS_FILE` to its path. Treat it as secret: never print its contents or commit it. Dream credentials may already be installed on the VM. Do not assume that credentials, an instance, account names, or exact paths from a previous task are still current.
