# License setup

This project is released under **GPL-3.0-or-later**, with some files under
**LGPL-3.0-or-later** (noted per-file).

## 1. Add the full license text

GitHub recognizes the license automatically if you add the canonical text as a
file named `LICENSE` in the repo root. Fetch it directly from the FSF:

```bash
curl -L https://www.gnu.org/licenses/gpl-3.0.txt -o LICENSE
```

If any files are LGPL, also include the LGPL text:

```bash
curl -L https://www.gnu.org/licenses/lgpl-3.0.txt -o COPYING.LESSER
```

(The LGPL is an additional permission layered on top of the GPL; the GPL text
in `LICENSE` is still required alongside it.)

## 2. Per-file header

Put this near the top of each GPL source file (adjust the one-line description):

```
# pan_tilt_node.py — STS3215 pan/tilt ROS2 node
# Copyright (C) 2024-2026 Edward L. Taychert, AA2MZ
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
```

For any file you intend to be LGPL instead, change the second paragraph to
reference the **GNU Lesser General Public License** and add, after the GPL
paragraph:

```
# You should also have received a copy of the GNU Lesser General Public
# License along with this program.  If not, see <https://www.gnu.org/licenses/>.
```

## 3. SPDX shorthand (optional but tidy)

Modern tooling also accepts a single SPDX line per file, which you can use in
addition to or instead of the full header:

```
# SPDX-License-Identifier: GPL-3.0-or-later
```

or for LGPL files:

```
# SPDX-License-Identifier: LGPL-3.0-or-later
```
