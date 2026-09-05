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
[[ -f ~/.bashrc ]] && source ~/.bashrc

pull_all() {
  while IFS= read -r git_dir; do
    repo="$(dirname "${git_dir}")"
    branch="$(git -C "${repo}" branch --show-current)"
    [[ -n ${branch} ]] || continue
    git -C "${repo}" pull "$1" "${branch}"
  done < <(find ~/cougars-dev -name .git -prune)
}

alias pull="pull_all origin"
alias pull-base="pull_all base"
alias vcs-import="~/cougars-dev/import.sh"
alias docker-pull="docker compose -f ~/cougars-dev/docker-compose.yaml pull"
alias build="docker compose -f ~/cougars-dev/docker-compose.yaml run --rm builder"
alias restart="sudo systemctl restart cougars.service && sudo journalctl -u cougars.service -f"
alias logs="docker logs -f cougars-runtime-ct"
