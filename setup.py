from setuptools import setup

with open("README.md", "r", encoding="utf-8") as fh:
  long_description = fh.read()

setup(
  name='pioneer_sdk',
  packages=['pioneer_sdk', 'pioneer_sdk.mavsub', 'pioneer_sdk.tools'],
  include_package_data=True,
  version='0.5.3',
  license='MIT',
  description='Programming tools for programming geoscan pioneer drone',
  long_description=long_description,
  long_description_content_type="text/markdown",
  author='geoscan',
  author_email='info@geoscan.aero',
  url='https://github.com/geoscan/pioneer_sdk',
  keywords=['mavlink', 'pioneer', 'geoscan'],
  setup_requires=['wheel'],
  install_requires=[
          'pymavlink==2.4.37',
          'pyserial==3.5',
          'future==0.18.3',
          'numpy',
          'opencv-contrib-python==4.7.0.72'
      ],
  classifiers=[
    "Programming Language :: Python :: 3",
    "License :: OSI Approved :: MIT License",
    "Operating System :: OS Independent",
  ],
  python_requires='>=3.7',
)
