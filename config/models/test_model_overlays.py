"""Checks on <package>/params/models/<model>.yaml, the per-model parameter files.

Each lives in its package, beside the factory file it lays over; this test
stays in config/models/ with model_keys.yaml because some of its rules span
packages (one lidar height in five of them).

A launch only checks that a model's file exists and parses (it refuses to
start otherwise). What the file *holds* is checked here, because none of it
fails at runtime: rclcpp ignores a key no node declares, and a key a model
leaves out is inherited from the factory file -- the G23's -- without a word.

  * every key in a model file is a key of the factory file it lays over, under
    the same node key: a misspelt key would otherwise change nothing;
  * the factory model has a file in all six packages (robot01 boots on them);
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


def model_path(model: str, name: str) -> Path:
    """<package>/params/models/<model>.yaml for the package entry `name`."""
    return WORKSPACE / FILES[name]["models"] / f"{model}.yaml"


# Every model any package has a file for: a model is named by its files, there
# is no directory of models to list any more.
MODELS = sorted(
    {p.stem for name in FILES for p in (WORKSPACE / FILES[name]["models"]).glob("*.yaml")}
)


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
    """(model, package entry) for every model file that exists."""
    return [(model, name) for model in MODELS for name in FILES if model_path(model, name).is_file()]


def test_models_found():
    assert FACTORY_MODEL in MODELS, f"no package has params/models/{FACTORY_MODEL}.yaml"


@pytest.mark.parametrize("file_name", sorted(FILES))
def test_factory_file_exists(file_name):
    assert (WORKSPACE / FILES[file_name]["factory"]).is_file()


@pytest.mark.parametrize("file_name", sorted(FILES))
def test_factory_model_has_every_file(file_name):
    # Every launch refuses a model without its file, and the factory model is
    # the one the robots in the field boot as.
    assert model_path(FACTORY_MODEL, file_name).is_file()


@pytest.mark.parametrize("file_name", sorted(FILES))
def test_models_dir_holds_only_models(file_name):
    # A launch loads <model>.yaml and nothing else from this directory, so
    # anything else in it (a .yml, a subdirectory, a stray copy) is a file no
    # launch reads.
    stray = sorted(
        p.name
        for p in (WORKSPACE / FILES[file_name]["models"]).iterdir()
        if not (p.is_file() and p.suffix == ".yaml")
    )
    assert not stray, f"{FILES[file_name]['models']}: no launch loads {stray}"


def model_nodes(file_name):
    """Nodes whose block lives in the model files only (model_keys.yaml)."""
    return set(FILES[file_name].get("model_nodes") or [])


@pytest.mark.parametrize("file_name", sorted(FILES))
def test_model_nodes_not_in_factory(file_name):
    # A model node in the factory file too would be checked against two
    # references, and its factory block would reach every model unannounced.
    both = sorted(model_nodes(file_name) & set(factory_params(file_name)))
    assert not both, (
        f"{FILES[file_name]['factory']}: {both} are model_nodes in model_keys.yaml; "
        "their block belongs in the model files only"
    )


def model_keys(file_name):
    """{node: keys} that live in the model files only (model_keys.yaml)."""
    return {
        node: set(keys)
        for node, keys in (FILES[file_name].get("model_keys") or {}).items()
    }


@pytest.mark.parametrize("file_name", sorted(FILES))
def test_model_keys_not_in_factory(file_name):
    # A model key left in the factory file too would be a silent fallback for
    # a model that forgets it -- the thing model_keys exists to rule out.
    factory = factory_params(file_name)
    for node, keys in model_keys(file_name).items():
        assert node in factory, f"model_keys.yaml: {file_name} has no node '{node}'"
        both = sorted(keys & set(factory[node]))
        assert not both, (
            f"{FILES[file_name]['factory']}: {node} {both} are model_keys in "
            "model_keys.yaml; they belong in the model files only"
        )


@pytest.mark.parametrize("model,file_name", model_files())
def test_model_keys_set(model, file_name):
    # Every model, the factory model included: there is no factory value under
    # a model key, so one left out is the C++ default.
    overlay = load_params(model_path(model, file_name))
    for node, keys in model_keys(file_name).items():
        missing = sorted(keys - set(overlay.get(node, {})))
        assert not missing, (
            f"{model_path(model, file_name).relative_to(WORKSPACE)}: {node} leaves "
            f"model keys unset (the C++ default would run): {missing}"
        )


@pytest.mark.parametrize("model,file_name", model_files())
def test_keys_exist_in_factory(model, file_name):
    # The reference for a model node is the factory model's own file, since
    # the factory file has no block for it; the factory model's file is then
    # the reference for itself, and its keys are not checked (a misspelt key
    # there shows as a node that misbehaves on the G23, where it is tested).
    path = model_path(model, file_name).relative_to(WORKSPACE)
    factory = factory_params(file_name)
    reference = load_params(model_path(FACTORY_MODEL, file_name))
    overlay = load_params(model_path(model, file_name))
    for node, keys in overlay.items():
        if node in model_nodes(file_name):
            if model == FACTORY_MODEL:
                continue
            assert node in reference, (
                f"{path}: model node '{node}' is not in {FACTORY_MODEL}'s file"
            )
            unknown = sorted(set(keys) - set(reference[node]))
            assert not unknown, (
                f"{path}: {node} sets keys {FACTORY_MODEL}'s file does not have "
                f"(a misspelt key is ignored by the node): {unknown}"
            )
            continue
        assert node in factory, (
            f"{path}: node key '{node}' is not in "
            f"{FILES[file_name]['factory']} ({sorted(factory)})"
        )
        allowed = set(factory[node]) | model_keys(file_name).get(node, set())
        unknown = sorted(set(keys) - allowed)
        assert not unknown, (
            f"{path}: {node} sets keys the factory file does not have "
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
    overlay = load_params(model_path(model, file_name))
    for node, keys in FILES[file_name]["required"].items():
        missing = sorted(set(keys) - set(overlay.get(node, {})))
        assert not missing, (
            f"{model_path(model, file_name).relative_to(WORKSPACE)}: {node} leaves these at the {FACTORY_MODEL}'s "
            f"values: {missing}"
        )


def merged_value(model, file_name, node, key):
    # A model key has no factory value; every model file sets it (above)
    value = factory_params(file_name).get(node, {}).get(key)
    path = model_path(model, file_name)
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
