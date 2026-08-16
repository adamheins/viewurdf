![viewurdf](https://github.com/adamheins/viewurdf/blob/main/viewurdf.png "viewurdf")

A dead simple URDF viewer that justs works.

## Skeleton Mode

One of the main goals of `viewurdf` is to provide a useful visualization *even
when the URDF's mesh files are missing.* This is done using *skeleton mode*,
which removes all existing visual and collision components from the URDF and
replaces them with simple primitives.

## Install

It is recommended to use [uv](https://docs.astral.sh/uv/) or
[pipx](https://pipx.pypa.io/latest/index.html) to automatically install the
tool into an isolated environment.
```bash
# preferably:
uv tool install viewurdf
# or
pipx install viewurdf

# if you must:
pip install viewurdf
```

## Usage

```
# basic usage: starts a viser server, view in your browser
viewurdf <file.urdf>

# skeleton mode: if you don't have all the assets referenced in the URDF
viewurdf -s <file.urdf>
```

## License

MIT
