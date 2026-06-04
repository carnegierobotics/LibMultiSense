# LibMultiSense Library-Consumption Test

A minimal CMake/Python project that stands in for a downstream consumer of an already-installed `LibMultiSense`. CI builds and installs the library then configures this project against the install prefix to verify that:

- the installed public headers compile,
- `find_package(MultiSense)` resolves and propagates its transitive dependencies (`MultiSenseWire`, optionally `nlohmann_json` and `OpenCV`) through the installed `MultiSenseConfig.cmake`,
- the installed shared library and its transitive DLLs/`so`s load at runtime,
- the installed Python wheel imports and exercises the native extension (see `consumption_test.py`).

## Maintenance

This test library must be maintained in parallel with `LibMultiSense` development to ensure that consumers can use the library as anticipated.

### Keep `vcpkg.json` in sync with the repository root

The Windows fixture installs the consumer's transitive dependencies via vcpkg manifest mode using the `vcpkg.json` manifest in this directory. Two fields must track the repository-root `vcpkg.json` and `vcpkg-configuration.json` whenever those files change:

1. `builtin-baseline` must equal the `default-registry.baseline` SHA in the root `vcpkg-configuration.json`.
1. The dependencies (including enabled features) must match or be a super set of the root `vcpkg.json`.

### Exercising a new propagated dependency

If `LibMultiSense` adds a new transitive dependency to the installed `MultiSenseConfig.cmake` (a new `find_dependency(...)` call):

1. Add a representative call to that dependency in `main.cc`, gated by an appropriate `CONSUME_*` macro defined from `CMakeLists.txt`.
1. Add the dependency to `vcpkg.json` here so the Windows fixture installs it, again matching the root manifest's features and baseline.
1. Add the corresponding system package install step to `.github/workflows/build-ubuntu.yml` and `.github/workflows/build-macos.yml`.
