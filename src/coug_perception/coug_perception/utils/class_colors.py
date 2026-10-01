# Copyright 2026 BYU FROST Lab
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

_CLASS_COLORS = {
    "red": (1.0, 0.0, 0.0),
    "green": (0.0, 0.8, 0.0),
    "black": (0.1, 0.1, 0.1),
    "yellow": (1.0, 0.9, 0.0),
    "white": (1.0, 1.0, 1.0),
    "orange": (1.0, 0.5, 0.0),
    "blue": (0.0, 0.3, 1.0),
}
_DEFAULT_COLOR = (1.0, 1.0, 1.0)


def class_color(class_name: str) -> tuple[float, float, float]:
    return _CLASS_COLORS.get(class_name.rsplit("_", 1)[-1], _DEFAULT_COLOR)
