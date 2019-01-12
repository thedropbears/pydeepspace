# pydeepspace
[![Build Status](https://travis-ci.org/thedropbears/pydeepspace.svg?branch=master)](https://travis-ci.org/thedropbears/pydeepspace)
[![codecov](https://codecov.io/gh/thedropbears/pydeepspace/branch/master/graph/badge.svg)](https://codecov.io/gh/thedropbears/pydeepspace)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/ambv/black)

The Drop Bears' robot code for _FIRST_ Destination: Deep Space (FRC 2019).

## Install dependencies
```bash
pip3 install -r requirements.txt
```

## Style
This codebase adheres to the code style enforced by the black autoformatter:
```bash
black .
```

This is enforced by CI.

See [PEP 8](https://www.python.org/dev/peps/pep-0008/) on naming conventions.

Docstrings should follow Google style.
See also [PEP 257](https://www.python.org/dev/peps/pep-0257/).

## Checks
Ensure code passes pyflakes, a static analysis checker:
```bash
pyflakes .
```

This is enforced by CI.

## Run
### Simulation (desktop)
```bash
./robot.py sim
```

### Tests
```bash
./robot.py test
```

If you wish to collect coverage data, use this instead:
```bash
./robot.py coverage test
```

### Deploy to roboRIO
```bash
./robot.py deploy
```

This codebase is set up to deploy to 4774's robot. (Use the `--robot` option to override this.)
