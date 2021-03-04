from setuptools import setup, find_packages

setup_requires = [
    ]

install_requires = [
	'pyserial>=3.5',
	'numpy',
	'colorama'
	'matplotlib'
    ]

dependency_links = [
    ]
desc = """\
Python package for controlling CoDrone
"""

setup(
    name='CoDrone',
    version='1.2.6',
    description='Python package for CoDrone',
    url='https://github.com/RobolinkInc/CoDrone-python.git',
    author='Robolink',
    author_email='info@robolink.com',
    packages=["CoDrone"],
    keywords=['robolink','drone','codrone'],
    include_package_data=True,
    install_requires=install_requires,
    setup_requires=setup_requires,
    dependency_links=dependency_links,
    python_requires='>=3',
    )
