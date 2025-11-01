"""
Setup script for FCND Motion Planning project.
This allows the project to be installed as a package for proper module imports.
"""

from setuptools import setup, find_packages

setup(
    name="fcnd-motion-planning",
    version="0.1.0",
    description="FCND Motion Planning - Autonomous drone path planning",
    author="FCND Team",
    python_requires=">=3.6",
    packages=find_packages(include=['visualize', 'tests']),
    py_modules=[
        'planning_utils',
        'motion_planning',
        'backyard_flyer_solution',
    ],
    install_requires=[
        'numpy',
        'matplotlib',
        'udacidrone',
    ],
    extras_require={
        'dev': ['pytest', 'ipython', 'jupyter'],
    },
    package_data={
        '': ['*.csv'],
    },
    include_package_data=True,
)
