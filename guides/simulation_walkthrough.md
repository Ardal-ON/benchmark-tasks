# Simulation walkthrough

> For the complete workspace-to-submission process, see [START_HERE.md](START_HERE.md) and [agent_workflow.md](agent_workflow.md). This is the supplied walkthrough; verify current paths, versions, credentials and instance status before running it.
From a fresh Brev instance to a scored Dream-1 run, a full-score demo and your own task.

Also available as a PDF. Copy commands from this page rather than the PDF: PDFs can break long commands.

Start a GPU instance
Start the Brev launchable from this link. Before deploying, make sure your instance has:

an L40S GPU (or another RTX GPU)
about 200 GB of storage
secure port 6080 exposed, for the remote desktop
Wait for the instance to be ready
Wait until VM Mode shows Built and the script shows Completed. Then open the notebook, or connect from your own terminal:

Brev instance page: VM Mode sbuilt, script completed, and the Open Notebook button
brev login
brev shell <your-instance-name>

Wait for setup to finish
The launchable installs Isaac Lab-Arena, the YAM embodiment, Dream-1, the scripted demo runner, a browser desktop and a trial-reports page, and warms up Isaac Sim. Watch its log until it ends with === [yam-setup] done, then press Ctrl-C and open a new shell:

tail -f /data/yam-setup.log

Check the YAM is installed
inspect-robots list embodiments | grep yam_arena

Save your Dream API key
Use the key from your account at platform.dreamlabs.ai:

mkdir -p ~/.dreamlabs && echo 'DREAM_API_KEY=dl_live_...' > ~/.dreamlabs/credentials && chmod 600 ~/.dreamlabs/credentials

Let Dream-1 operate the arms
The station scene is a replica of Dream Labs’ calibrated YAM station, scored automatically. --max-steps 2400 is the default run length: 2 minutes at 20 steps a second. Your own task may need more or fewer: choose what it reasonably needs, and enter it as your submission’s Run limit.

inspect-robots run --embodiment yam_arena --policy dream \
  -E scene=station -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "put apple into the bowl" --scorer success_at_end \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/dream

Run the scripted demo
Other station tasks include put orange into the bowl and put banana on the plate. Add --epochs 3 for repeated trials; -E object_jitter=0 keeps every object exactly where the scene puts it. The scripted demo reaches full score:

inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E scene=station -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "put apple into the bowl" --scorer success_at_end \
  --max-steps 600 --store-frames --log-dir ~/yam-eval/demo

Make videos
One MP4 per camera, written next to each run’s frames in ~/yam-eval/dream/frames/<run>/:

for log in ~/yam-eval/dream/*.json; do inspect-robots video "$log"; done

Save a setup screenshot
The first frame of the top camera, for your submission (replace <run> with the run’s folder):

ffmpeg -y -i ~/yam-eval/dream/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 ~/yam-eval/setup.png

Browse the trial reports
On the instance:

inspect-robots view ~/yam-eval/dream --serve --force --port 8301

Then, in a second terminal on your laptop, forward the port and open http://127.0.0.1:8301:

brev port-forward <instance-name> -p 8301:8301

Watch runs live (optional)
Get the desktop password on the instance:

cat ~/yam-desktop-password

Open the instance’s port 6080 secure link from the Brev console, or forward the port on your laptop and open http://127.0.0.1:6080/vnc.html. Then enter the password.

brev port-forward <instance-name> -p 6080:6080

Add -E headless=false to a run, and the Isaac Sim window opens on that desktop. The first windowed run on a new instance shows a black window for about 4 minutes while Isaac Sim warms up:

inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E scene=station -E object_jitter=0 -E headless=false -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "put apple into the bowl" --scorer success_at_end \
  --max-steps 600 --store-frames --log-dir ~/yam-eval/live

Build your own task
Simple tasks can be one Python file: copy the example, change its objects, success(), score() and demo(), then run it the same way (see “Your own task” in the repository’s README). Dynamic mechanisms or additional simulator state may require an extension; document and validate it with the task.

cp ~/yam-isaac-lab-arena/examples/stack_cubes.py ~/my_task.py

inspect-robots run --embodiment yam_arena --policy dream \
  -E task=~/my_task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "put the red cube on the blue block" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/my-task-dream

inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=~/my_task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "put the red cube on the blue block" --scorer task_score \
  --max-steps 600 --store-frames --log-dir ~/yam-eval/my-task-demo
