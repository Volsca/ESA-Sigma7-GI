from setuptools import find_packages, setup

package_name = "s7spaceship"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Emre",
    maintainer_email="emre.artar@ext.esa.int",
    description="Sigma 7 communication package for force and pose information",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "StateSubscriber = s7spaceship.StateSub:main",
            "ForcePublisher = s7spaceship.ForcePub:main",
            "InterfaceMsgSubscriber = s7spaceship.IntMsgSub:main",
            "InstructionPublisher = s7spaceship.InstructionPub:main",
            "Spaceship = s7spaceship.Spaceship:main",
        ],
    },
)
