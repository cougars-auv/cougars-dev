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
