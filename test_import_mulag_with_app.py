import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Test importing Mulag asset after Isaac Sim starts.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import rrlab_assets.mulag as m

print("[OK] Imported mulag.py from:")
print(m.__file__)

simulation_app.close()
