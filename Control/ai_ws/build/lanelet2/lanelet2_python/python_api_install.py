#!/usr/bin/env python
# this file is taken and converted from a bash script from catkin to correctly install python modules
import os
import sys
import subprocess

python_interpreter = "/usr/bin/python2"
if not python_interpreter:
    python_interpreter = "python"

if "DESTDIR" in os.environ:
    if not os.path.isabs(os.environ["DESTDIR"]):
        print("DESTDIR argument must be absolute...\notherwise python's distutils will bork things.")
        sys.exit(1)
    destdir_arg = ["--root={}".format(os.environ["DESTDIR"])]
else:
    destdir_arg = []


def print_and_run(cmd, env):
    print(" ".join(cmd))
    subprocess.check_call(cmd, env=env)

os.chdir("/home/labdog/ai_ws/build/lanelet2/lanelet2_python")
install_dir = "$DESTDIR/home/labdog/ai_ws/install/lib/python2.7/dist-packages"
print("Creating {}".format(install_dir))
try:
    os.makedirs(install_dir)
except OSError:
    pass

env = os.environ.copy()
env["PYTHONPATH"] = "/home/labdog/ai_ws/install/lib/python2.7/dist-packages:/home/labdog/ai_ws/build/lib/python2.7/dist-packages:{}".format(env.get("PYTHONPATH", ""))
env["CATKIN_BINARY_DIR"] = "/home/labdog/ai_ws/build"
setuptools_arg = "--install-layout=deb"
setuptools_arg = [setuptools_arg] if setuptools_arg else []
cmd = [python_interpreter, "/home/labdog/ai_ws/build/lanelet2/lanelet2_python/setup.py", "build", "--build-base", "/home/labdog/ai_ws/build/lanelet2/lanelet2_python", "install"] + destdir_arg + \
      setuptools_arg + ["--prefix=/home/labdog/ai_ws/install", "--install-scripts=/home/labdog/ai_ws/install/bin"]

print_and_run(cmd, env)
