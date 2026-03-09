from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="drone-project",
    version="0.1.0",
    author="harsh-pandhe",
    author_email="harshpandhehome@gmail.com",
    description="Autonomous drone control and monitoring system with MAVLink interface",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/harsh-pandhe/drone-project",
    packages=find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Topic :: Scientific/Engineering :: Robotics",
    ],
    python_requires=">=3.9",
    install_requires=[
        "pymavlink>=2.4.0",
        "mavsdk>=1.4.0",
        "flask>=3.0.0",
        "flask-socketio>=5.0.0",
        "pyserial>=3.5",
    ],
    extras_require={
        "dev": [
            "pytest>=7.0",
            "pytest-cov>=4.0",
            "black>=23.0",
            "flake8>=5.0",
        ],
        "ros": [
            "rclpy>=0.13.0",
            "nav-msgs>=0.13.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "drone-server=src.ascend_server:main",
        ],
    },
)
