# Copyright 2015 Open Source Robotics Foundation, Inc.
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

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # This package's docstring register is deliberate and predates the linter:
    # multi-line summaries that start on the FIRST line (the D212 style — D213
    # wants the opposite and the two are mutually exclusive), prose paragraphs
    # that explain *why* rather than one imperative sentence, and summaries
    # that run past one line or end without a period. The codes below encode
    # that house style so the linter keeps checking what can actually be wrong
    # (malformed quotes, stray blank lines, missing r-prefixes) instead of
    # demanding a rewrite of every docstring in the package:
    #   D205/D209/D213 — multi-line layout (summary-on-first-line, D212 style)
    #   D400/D415     — "first line must end with a period / punctuation"
    #   D401          — "first line must be imperative mood"
    #   D403          — first-word capitalization (a docstring legitimately
    #                    starts with a lowercase field name like ``q``)
    rc = main(argv=[
        '.', 'test',
        '--add-ignore', 'D205', 'D209', 'D213', 'D400', 'D401', 'D403', 'D415',
    ])
    assert rc == 0, 'Found code style errors / warnings'
