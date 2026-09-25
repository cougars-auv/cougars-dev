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

# shellcheck shell=bash
pull_all() {
  while IFS= read -r git_dir; do
    repo="$(dirname "${git_dir}")"
    branch="$(git -C "${repo}" branch --show-current)"
    [[ -n ${branch} ]] || continue
    git -C "${repo}" pull "$1" "${branch}"
  done < <(find ~/cougars-dev -name .git -prune)
}

compose() {
  docker compose -f ~/cougars-dev/ops/docker-compose.yaml "$@"
}

pull() { pull_all origin; }
pull-base() { pull_all base; }
vcs-import() { ~/cougars-dev/ops/import.sh; }
docker-pull() { compose pull; }
build() { compose run --rm builder; }
restart() { compose up -d --force-recreate; }
logs() { docker logs -f cougars-runtime-ct; }
