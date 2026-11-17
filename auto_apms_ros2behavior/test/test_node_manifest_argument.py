# Copyright 2026 Robin Müller
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The --node-manifest argument of `ros2 behavior run` and `ros2 behavior send` is a node manifest resource. The
behavior functions expect its NodeManifest, which they dump into the build request."""

from argparse import Namespace

import pytest

from auto_apms_ros2behavior.verb import run, send


class FakeNodeManifest:
    def dump(self):
        return "manifest"


class FakeNodeManifestResource:
    # Stands in for NodeManifestResource, which needs the resource index
    def __init__(self):
        self.node_manifest = FakeNodeManifest()


def common_args(**overrides):
    args = dict(
        behavior=None,
        build_request="<root/>",
        build_handler="auto_apms_behavior_tree::TreeFromStringBuildHandler",
        entry_point="Tree",
        node_manifest=FakeNodeManifestResource(),
        blackboard=[],
        tick_rate=None,
        groot2_port=None,
        state_change_logger=None,
        logging=None,
    )
    args.update(overrides)
    return Namespace(**args)


@pytest.fixture
def captured(monkeypatch):
    calls = {}

    def capture(**kwargs):
        calls.update(kwargs)
        return 0

    monkeypatch.setattr(run, "sync_run_generic_behavior_locally", capture)
    monkeypatch.setattr(send, "sync_run_generic_behavior_with_executor", capture)
    return calls


def test_run_passes_the_manifest_of_the_resource(captured):
    run.RunVerb().main(args=common_args(namespace=None, node_name=None))
    assert isinstance(captured["node_manifest"], FakeNodeManifest)
    assert captured["node_manifest"].dump() == "manifest"


def test_send_passes_the_manifest_of_the_resource(captured):
    verb = send.SendVerb()
    verb.action_executor_mapping = {}
    verb.main(args=common_args(action_name="/executor/start", executor="", clear_blackboard=False))
    assert isinstance(captured["node_manifest"], FakeNodeManifest)


def test_run_without_manifest_passes_none(captured):
    run.RunVerb().main(args=common_args(node_manifest=None, namespace=None, node_name=None))
    assert captured["node_manifest"] is None
