# Sourced by build_image.sh (and nav_live.sh): which commit is this checkout?
#   checkout_revision REPO   prints the 40-hex commit of REPO: git when REPO is itself a repository, else the REVISION file that `git archive` fills in
#                            (.gitattributes export-subst); prints nothing when neither knows (a plain copy made by hand).
#   checkout_dirty REPO      succeeds when REPO is a repository with uncommitted changes.
# A directory that merely sits inside another repository (a copy under a repository's working tree) is not a repository: the toplevel must be REPO itself.
checkout_revision() {
  local repo sha top
  repo=$(cd "$1" && pwd -P)
  top=$(git -C "$repo" rev-parse --show-toplevel 2>/dev/null || true)
  sha=
  [[ -n $top && $(cd "$top" && pwd -P) == "$repo" ]] && sha=$(git -C "$repo" rev-parse HEAD 2>/dev/null || true)
  [[ $sha =~ ^[0-9a-f]{40}$ || ! -r $repo/REVISION ]] || sha=$(tr -d '[:space:]' < "$repo/REVISION")
  [[ $sha =~ ^[0-9a-f]{40}$ ]] && echo "$sha"
  return 0
}
checkout_dirty() {
  local repo top
  repo=$(cd "$1" && pwd -P)
  top=$(git -C "$repo" rev-parse --show-toplevel 2>/dev/null || true)
  [[ -n $top && $(cd "$top" && pwd -P) == "$repo" && -n $(git -C "$repo" status --porcelain 2>/dev/null) ]]
}
