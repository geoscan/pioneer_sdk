# Geoscan `pioneer_sdk`

Geoscan PioneerSDK is a collection of tools for interfacing with Geoscan Pioneer UAVs.
It provides a succinct and convenient API to drones' functions such as autonomous flight, acquiring frames from a drone's camera, and more.
Please feel free to exlore `examples/` directory for stepping stones and inspiration.

## Installation

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install pioneer_sdk.

```bash
python3 -m pip install pioneer_sdk or
python3 -m pip install git+https://github.com/geoscan/pioneer_sdk.git@master
```
## Usage

```python
from pioneer_sdk import Pioneer
pioneer_mini = Pioneer()
if __name__ == '__main__':

    ...
```

## Documentation

You can read information about package in the documentation page
[readthedocs](https://pioneer-doc.readthedocs.io/ru/master/programming/python/python_main.html)

## Additional information

You can find the examples and the camera calibration script in the homonymous folders.
Unlike "bare" `pioneer_sdk`, they require a broader set of dependencies.
To satisfy the requirements, run:
[pip](https://pip.pypa.io/en/stable/):
```bash
python3 -m pip install -r requirements.txt
```
