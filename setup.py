from setuptools import setup
setup(
  name='pioneer_sdk',
  packages=['pioneer_sdk'],
  version='0.2.4',
  license='MIT',
  description='Programming tools for programming geoscan pioneer drone',
  author='geoscan',
  author_email='info@geoscan.aero',
  url='https://github.com/geoscan/pioneer_sdk',
  keywords=['mavlink', 'pioneer', 'geoscan'],
  setup_requires=['wheel'],
  install_requires=[
          'pymavlink',
      ],
  classifiers=[
    "Programming Language :: Python :: 3",
    "License :: OSI Approved :: MIT License",
    "Operating System :: OS Independent",
  ],
  python_requires='>=3.6',
)
