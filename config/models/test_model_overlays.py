"""Checks on config/models/<model>/*.yaml, the per-model parameter files.

A launch only checks that a model's file exists and parses (it refuses to
start otherwise). What the file *holds* is checked here, because none of it
fails at runtime: rclcpp ignores a key no node declares, and a key a model
leaves out is inherited from the factory file -- the G23's -- without a word.

  * every key in a model file is a key of the factory file it lays over, under
    the same node key: a misspelt key would otherwise change nothing;
  * the factory model has all six files (robot01 boots on them);
  * a model other than the factory model sets every key model_keys.yaml
    requires of a file, once that file exists;
  * the groups model_keys.yaml lists as `equal` hold one value on the merged
    result (factory + model file) of every model.

A model with a file missing is not a failure here: that is the launch's
refusal, and how a model that is still being measured (m20 today) stays
unrunnable without turning CI red.

PyYAML + pytest only, so it runs on the host:  python3 -m pytest config/models
"""

from pathlib import Path

import pytest
import yaml

MODELS_DIR = Path(__file__).resolve().parent
WORKSPACE = MODELS_DIR.parent.parent

with open(MODELS_DIR / "model_keys.yaml", encoding="utf-8") as _f:
    SCHEMA = yaml.safe_load(_f)

FACTORY_MODEL = SCHEMA["factory_model"]
FILES = SCHEMA["files"]
MODELS = sorted(p.name for p in MODELS_DIR.iterdir() if p.is_dir() and not p.name.startswith((".", "_")))


def flatten(node, prefix=""):
    """{a: {b: 1}} -> {"a.b": 1}; a list or a scalar is a leaf."""
    out = {}
    for key, value in (node or {}).items():
        path = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(value, dict) and value:
            out.update(flatten(value, path))
        elif not isinstance(value, dict):
            out[path] = value
    return out


def load_params(path: Path):
    """A params file as {node key: {dotted path: value}}."""
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    assert isinstance(data, dict), f"{path}: top level is not a mapping"
    params = {}
    for node, section in data.items():
        assert isinstance(section, dict) and "ros__parameters" in section, (
            f"{path}: '{node}' has no ros__parameters block"
        )
        params[node] = flatten(section["ros__parameters"])
    return params


def factory_params(file_name: str):
    return load_params(WORKSPACE / FILES[file_name]["factory"])


def model_files():
    """(model, file name) for every model file that exists."""
    return [
        (model, name)
        for model in MODELS
        for name in FILES
        if (MODELS_DIR / model / name).is_file()
    ]


def test_models_found():
    assert FACTORY_MODEL in MODELS, f"no config/models/{FACTORY_MODEL}/"


@pytest.mark.parametrize("file_name", sorted(FILES))
def test_factory_file_exists(file_name):
    assert (WORKSPACE / FILES[file_name]["factory"]).is_file()


@pytest.mark.parametrize("file_name", sorted(FILES))
def test_factory_model_has_every_file(file_name):
    # Every launch refuses a model without its file, and the factory model is
    # the one the robots in the field boot as.
    assert (MODELS_DIR / FACTORY_MODEL / file_name).is_file()


@pytest.mark.parametrize("model", MODELS)
def test_no_unknown_file(model):
    # A file under another name is one no launch loads.
    unknown = sorted(
        p.name for p in (MODELS_DIR / model).glob("*.yaml") if p.name not in FILES
    )
    assert not unknown, f"config/models/{model}/: no launch loads {unknown}"


@pytest.mark.parametrize("model,file_name", model_files())
def test_keys_exist_in_factory(model, file_name):
    factory = factory_params(file_name)
    overlay = load_params(MODELS_DIR / model / file_name)
    for node, keys in overlay.items():
        assert node in factory, (
            f"{model}/{file_name}: node key '{node}' is not in "
            f"{FILES[file_name]['factory']} ({sorted(factory)})"
        )
        unknown = sorted(set(keys) - set(factory[node]))
        assert not unknown, (
            f"{model}/{file_name}: {node} sets keys the factory file does not have "
            f"(a misspelt key is ignored by the node): {unknown}"
        )


@pytest.mark.parametrize("file_name", sorted(FILES))
def test_required_keys_are_factory_keys(file_name):
    # The list itself: a required key that was renamed in the factory file
    # could never be satisfied, or worse, would be satisfied by a dead key.
    factory = factory_params(file_name)
    for node, keys in FILES[file_name]["required"].items():
        assert node in factory, f"model_keys.yaml: {file_name} has no node '{node}'"
        missing = sorted(set(keys) - set(factory[node]))
        assert not missing, f"model_keys.yaml: {file_name} {node} requires unknown {missing}"


@pytest.mark.parametrize(
    "model,file_name", [mf for mf in model_files() if mf[0] != FACTORY_MODEL]
)
def test_required_keys_set(model, file_name):
    overlay = load_params(MODELS_DIR / model / file_name)
    for node, keys in FILES[file_name]["required"].items():
        missing = sorted(set(keys) - set(overlay.get(node, {})))
        assert not missing, (
            f"{model}/{file_name}: {node} leaves these at the {FACTORY_MODEL}'s "
            f"values: {missing}"
        )


def merged_value(model, file_name, node, key):
    value = factory_params(file_name)[node][key]
    path = MODELS_DIR / model / file_name
    if path.is_file():
        value = load_params(path).get(node, {}).get(key, value)
    return value


@pytest.mark.parametrize("model", MODELS)
@pytest.mark.parametrize("group", SCHEMA["equal"], ids=lambda g: g["name"])
def test_equal_groups(model, group):
    values = {
        f"{file_name} {node} {key}": merged_value(model, file_name, node, key)
        for file_name, node, key in group["keys"]
    }
    assert len(set(map(repr, values.values()))) == 1, (
        f"{model}: '{group['name']}' must be one value, got {values}"
    )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
