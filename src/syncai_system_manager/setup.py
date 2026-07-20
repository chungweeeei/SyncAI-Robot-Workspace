import compileall
import os
import shutil
from glob import glob

from setuptools import find_packages, setup
from setuptools.command.install import install

package_name = 'syncai_system_manager'


class InstallNoSource(install):
    """Install compiled bytecode only.

    After the normal install, compile the package's ``.py`` files to
    ``.pyc`` in the legacy (sourceless) layout and remove the ``.py``
    sources from the install space, so deployment ships no source code.

    Guarded for ``--symlink-install``: if the installed modules are
    symlinks back to the source tree, stripping is skipped so developer
    builds never lose their sources.
    """

    def run(self):
        install.run(self)
        target = os.path.join(self.install_lib, package_name)
        if not os.path.isdir(target):
            return
        py_files = glob(os.path.join(target, '**', '*.py'), recursive=True)
        if any(os.path.islink(p) for p in py_files):
            return
        compileall.compile_dir(target, legacy=True, quiet=1)
        for py in py_files:
            os.remove(py)
        for root, dirs, _ in os.walk(target):
            for d in dirs:
                if d == '__pycache__':
                    shutil.rmtree(os.path.join(root, d), ignore_errors=True)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    cmdclass={'install': InstallNoSource},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='chungweeeei@gmail.com',
    description='System manager node coordinating SyncAI robot subsystems',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'system_manager_node = syncai_system_manager.main:main',
        ],
    },
)
