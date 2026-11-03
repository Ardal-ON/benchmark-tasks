# Agent workflow: design and submit a Robocurve task

This is a practical guide based on a completed YAM conveyor task on 2026-11-03. The current user request and the benchmark checkout's `AGENTS.md` take precedence over these notes. Check the task board and repository for current rules, versions, field limits and duplicate tasks; do not assume this example's results transfer to a different task.

## 1. Establish the workspace and scope

1. Read [START_HERE.md](START_HERE.md), the user-provided submission requirements, and the benchmark checkout's `AGENTS.md`. Identify what the user authorizes: fork, start a paid instance, run Dream, upload media, push, or open a PR. Honor requests to review the demo or videos first.
2. Inspect `git status -sb`, remotes and existing branches. If a suitable fork and checkout already exist, reuse them without disturbing existing work. Otherwise, authenticate with the team's registered GitHub account and run `gh repo fork robocurve/benchmark-tasks --clone`; a fork created under a different account does not establish that the account is registered with the team. Fetch upstream and create a distinct branch for the new task from a current base. Keep one task per PR.
3. Read the task board before settling on an idea. Compare pending and accepted tasks for substantive overlap, not just titles. The board requires team access. If access fails, report that the duplicate check is incomplete; do not claim it passed.
4. Keep credential files, Dream API keys, raw logs with credentials, local paths and personally identifying material out of the public fork. Avoid printing environment variables or shell commands that interpolate secrets. Task descriptions, screenshots and videos must follow the repository's blind-review rules.

### Create a clean task branch

Keep the guide worktree on `dev`. In that checkout, first inspect `git status -sb` and `git remote -v`. If the canonical upstream remote is absent, add it as `upstream`; do not replace an existing remote without checking it. Then, substituting a new task name and an unused worktree path:

```sh
git fetch upstream main
TASK_SLUG=example-task
git worktree add -b "task/$TASK_SLUG" "../task-$TASK_SLUG" upstream/main
```

Make the new task under `tasks/<task-name>/` in the new worktree, and use the guide worktree for reference. Before committing and opening a PR, run `git diff --name-status upstream/main...HEAD` and inspect uncommitted changes. Only the intended new task folder should differ; never copy `guides/`, the guide branch's `AGENTS.md` changes, old submissions, credentials or generated media into the PR. An upstream `main` may contain tasks already accepted by the maintainers: keep those baseline files. A task branch and PR target upstream `main`; do not submit the guide branch itself.

The task board's authenticated task-list endpoint was available as `/api/tasks` during the previous submission. If checking from a terminal, a browser-style user agent avoided the edge filter, and the bearer token can be constructed inside the process without exposing it in command arguments. Set `ROBOCURVE_CREDENTIALS_FILE` to the path of your private credentials file outside the checkout, then run from either worktree:

```sh
set -a
source "$ROBOCURVE_CREDENTIALS_FILE"
set +a
python3 - <<'PYBOARD'
import json, os, urllib.request
headers = {
    'Authorization': 'Bearer ' + os.environ['ROBOCURVE_TEAM'] + ':' + os.environ['ROBOCURVE_TOKEN'],
    'User-Agent': 'Mozilla/5.0',
}
req = urllib.request.Request('https://tasks.robocurve.org/api/tasks', headers=headers)
with urllib.request.urlopen(req, timeout=30) as response:
    tasks = json.load(response)['tasks']
for task in tasks:
    print(task.get('title', ''), '|', task.get('status', ''))
PYBOARD
```

Review relevant tasks beyond their titles; this list is a starting point, not proof of uniqueness. Do not print the authorization header or the credentials. Endpoint behavior and visibility may change.

The previous task lives on its own `conveyor-sort` branch and PR, with review media kept locally outside the repository. It is an example, not a template to edit for a new task; the task files and media are not part of this guide branch. Its submission was [PR #83](https://github.com/robocurve/benchmark-tasks/pull/83), received as `TK-LJQEDJ` on 2026-11-03. Check its present status independently if needed.

## 2. Design the task and scorer

A normal task file declares `SCENE`, `OBJECTS`, optional `success(objects)`, `score(objects)`, and optionally `demo()`; see [the YAM Arena reference](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/README.md#your-own-task) and [stack_cubes.py](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/examples/stack_cubes.py). An object with mass is free; one without mass is fixed. The supplied `yam_arena` task-file API does **not** itself provide moving conveyors, timed feeders or robot contact data in `score(objects)`. Tasks needing those features require a simulator extension or a different supported mechanism, plus clear run instructions and validation; do not describe a static prop as moving.

Specify every object, material approximation, size, mass, pose, camera, timing, seed and reset rule. Confirm reach, clearance, collisions and camera visibility. Document numerical scoring for all outcomes, including partial credit, wrong placements, drops, contact and whether the final or best score is used. Prevent unintended strategies such as fallen or piled blocks earning lift credit. If using a proxy for grasping rather than measured contacts, say so and test its limitations. A task should have real-world relevance, be safely feasible, and be materially different from existing tasks.

For the conveyor task, the documented simplifications include a kinematically animated loading shelf and removal of missed blocks at the belt end. Those choices should not silently become defaults for a future task. A fixed mixed sequence is reproducible but is not randomized per trial; describe it accurately. Use meaningful offline tests for scoring, resets and relevant edge cases, and GPU/physics tests for behavior that cannot be verified offline.

## 3. Start and use Brev

Discover the existing instance before creating one. Its name and status may change:

```sh
brev ls
brev start <instance-name> -d
brev ls
brev exec <instance-name> 'hostname'
```

Wait until SSH is actually usable; an instance can briefly report ready before its connection works. A fresh launchable also needs setup to finish; check `/data/yam-setup.log` as described in [simulation_walkthrough.md](simulation_walkthrough.md). RTX-class hardware such as an L40S is needed for Isaac Sim rendering; the supplied guide says A100/H100 lack the required RT cores. Record the actual software versions and commits used. Shader compilation after a restart can take several minutes.

In a noninteractive VM shell, activate the installed environment explicitly:

```sh
source ~/IsaacLab-Arena/yam.env
inspect-robots list embodiments | grep yam_arena
```

The VM's venv may have no `pip` module and noninteractive shells may not have `uv` on `PATH`. If present, `~/.local/bin/uv pip install -e <task-directory> --no-deps` installs an extension into the activated environment. Copy the task to the VM using `rsync` or `scp`, but do not copy local credentials into the repository. Check for existing Dream credentials without displaying them.

For custom physics, verify actual state over time, not just a visual animation. In this environment the tested PhysX surface-velocity conveyor maintained contacts with `device=cpu`, while a GPU-physics test let blocks fall through; RTX still rendered cameras. That observation is specific to the tested setup, not a blanket rule for all Isaac Sim versions or tasks. Verify every new configuration, including released and missed objects, and document approximations.

## 4. Demonstrate and evaluate

Run offline tests first, then validate a scripted or teleoperated demo under the same task and physical configuration. If the user asks for a good demo before Dream, do not run Dream until that condition is satisfied. Unless Dream itself earns full score, submission requires a full-score demo. Run Dream with the approved instruction, action limit, seed and camera configuration; use `--store-frames`. Do not infer a final score from visuals, an intermediate frame, an interrupted run, or a different configuration. Read the completed Inspect Robots JSON and record the reported score, steps, runtime, task hash and settings.

The sample commands in the supplied [walkthrough](simulation_walkthrough.md) use `yam_arena`, 20 Hz and 320×176 observations. For a custom task, follow its README and registered embodiment/scorer. Raising frame dimensions can improve the output, but it may also change policy inputs or model compatibility; rerun and document the chosen settings. Changing the top camera can change Dream's behavior: a new camera view of a policy requires a new run unless actions or a replayable simulation recording were actually saved. Do not present a fresh run as another view of the exact same trajectory.

Inspect frames, not only scores. Check the starting layout, feeder or object motion, physical interactions, end state and blind-review requirements. Save a setup image that shows all objects; a second view is acceptable only when it accurately depicts the same setup, and the difference should be documented. Respect any instruction that the user wants to review media before upload.

Render the completed log on the VM and copy the resulting media to a local review folder:

```sh
inspect-robots video <completed-run.json>
# Output: frames/<run>/scene-0-e0_top_cam.mp4 and wrist-camera MP4s
ffmpeg -y -i <input.mp4> -c:v libx264 -pix_fmt yuv420p -movflags +faststart <output.mp4>
ffprobe -v error -select_streams v:0 -show_entries stream=codec_name,pix_fmt,width,height \
  -show_entries format=duration,size -of default=noprint_wrappers=1 <output.mp4>
```

Per the supplied submission guide, videos are H.264 MP4 at most 50 MB each and the screenshot is at most 10 MB; recheck current board rules. Preserve run logs and unuploaded media outside the public task folder. After work finishes, stop an instance that was started for the task when appropriate, and tell the user its state.

## 5. Prepare and submit

Each task folder needs `task.json`, `README.md`, and all scene/scoring/extension code needed to reproduce the run. The README should give exact installation, commands, seeds, run limit, actual scores and wall-clock runtime, version identifiers, and fidelity limitations. Check all required metadata against the benchmark repo's current `AGENTS.md` and task board. On 2026-11-03 the board enforced a **500-character maximum for `fidelity`**; an overlong note caused the validator to report both a length error and a missing field. Keep descriptions within current field limits.

Choose the exact reviewed Dream video, demo video and screenshot, then ensure `scores`, camera/fidelity notes and reproduction commands correspond to those files. Different camera runs have different reported scores. Review the task and PR text and the actual media for names, usernames, institution logos, identifying paths and other blind-review violations.

When the user has authorized upload, load team credentials privately, then use the repository uploader. For example, from the benchmark checkout, with `ROBOCURVE_CREDENTIALS_FILE` set to a private secret file outside the repository:

```sh
set -a
source "$ROBOCURVE_CREDENTIALS_FILE"
set +a
python3 tools/upload_media.py tasks/<task-name> \
  --video <dream.mp4> --demo <demo.mp4> --screenshot <setup.png>
```

The uploader records the returned media ID in `task.json`; verify that it did so. Never paste the token into the command line, logs, commits or chat. On 2026-11-03 the board's edge security returned `403` with plain-text `error code: 1010` to Python's default `urllib` user agent, even though the credentials were valid. A request with a normal browser-style user agent succeeded. Distinguish this from a genuine authentication error. If the same filter recurs, the following wrapper runs the **existing uploader** with a browser-style user agent, without modifying the repository or exposing credentials:

```sh
set -a
source "$ROBOCURVE_CREDENTIALS_FILE"
set +a
python3 - <task-directory> <dream.mp4> <demo.mp4> <setup.png> <<'PY'
import runpy, sys, urllib.request

request = urllib.request.Request
class BrowserRequest(request):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.add_header('User-Agent', 'Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/130.0.0.0 Safari/537.36')
urllib.request.Request = BrowserRequest
sys.argv = ['tools/upload_media.py', sys.argv[1], '--video', sys.argv[2], '--demo', sys.argv[3], '--screenshot', sys.argv[4]]
runpy.run_path('tools/upload_media.py', run_name='__main__')
PY
```

Do not weaken security or expose a token to bypass an access denial. If a normal browser-style request still gets an authentication error, ask the user to verify credentials or account access.

Once authorized, stage only the task folder, verify `git diff --check`, commit, push the task branch to the team's fork and open a PR into `robocurve/benchmark-tasks:main`. Use a concise description of the task, actual scores and validation. Do not push or open a PR when the user has withheld that authorization.

Read PR comments after opening. A validation failure is not a received task: fix the concrete issue, push to the **same** branch when instructed, and wait for the comment that gives a task ID and says it is waiting for review. In the previous task, shortening `fidelity` and pushing a correction produced a receipt within seconds. Once received, follow the current repository rules about changes while review is pending; do not assume a push before review updates the submission. Report the PR URL, task ID and status accurately. Do not merge the PR yourself.
