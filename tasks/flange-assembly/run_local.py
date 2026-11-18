"""Register this source tree for one Inspect CLI process, without installation."""
from inspect_robots.registry import register
from flange_assembly.embodiment import make_embodiment
from flange_assembly.policy import make_policy
from inspect_robots.cli import main
register('embodiment','yam_flange')(make_embodiment)
register('policy','yam_flange_scripted')(make_policy)
if __name__=='__main__':raise SystemExit(main())
