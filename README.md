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
[readthedocs](https://pioneer-doc.readthedocs.io/ru/master/)

## Additional information

You can find the examples and the camera calibration script in the homonymous folders.
To satisfy requiremets for them you can use requirements.txt file and install them by 
[pip](https://pip.pypa.io/en/stable/): 
```bash
python3 -m pip install -r requirements.txt
```
