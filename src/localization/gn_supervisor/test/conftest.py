"""pytest src/localization/gn_supervisor/test: the modules under test are in ../scripts (run by path, not a package). None of these tests needs ROS 2."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
