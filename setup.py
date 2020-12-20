from distutils.core import setup
setup(
  name='Pioneer_sdk',
  packages=['Pioneer_sdk'],
  version='0.1',
  license='MIT',
  description='Programming tools for programming geoscan pioneer drone',
  author='geoscan',
  author_email='info@geoscan.aero',
  url='',
  download_url='',
  keywords=['p', 'i', 'o', 's', 'd', 'k'],
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
