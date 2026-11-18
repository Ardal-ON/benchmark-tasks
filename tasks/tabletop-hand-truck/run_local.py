from inspect_robots.registry import register
from cart_task.embodiment import make_embodiment
from cart_task.policy import make_policy
from inspect_robots.cli import main
register('embodiment','yam_tabletop_cart')(make_embodiment)
register('policy','cart_scripted')(make_policy)
if __name__=='__main__':raise SystemExit(main())
