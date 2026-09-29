import os
import subprocess
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]

class CompatibilityTests(unittest.TestCase):
    def test_bash_syntax(self):
        for script in ('detect_ros_distro.sh', 'setup_workspace.sh', 'run_three_modes.sh', 'profile_mapper.sh'):
            subprocess.run(['bash', '-n', str(ROOT/'scripts'/script)], check=True)

    def test_distribution_matrix_and_shared_build(self):
        doc = (ROOT/'docs'/'distribution_compatibility.md').read_text()
        self.assertIn('Ubuntu 22.04', doc)
        self.assertIn('Ubuntu 24.04', doc)
        self.assertIn('`jazzy`', doc)
        self.assertIn('`main`', doc)
        script = (ROOT/'scripts'/'run_three_modes.sh').read_text()
        self.assertIn('humble|jazzy', script)
        cmake = (ROOT/'CMakeLists.txt').read_text()
        self.assertIn('option(BUILD_CUDA', cmake)

if __name__ == '__main__': unittest.main()
