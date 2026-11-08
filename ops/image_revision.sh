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
  # shortcut: only what the image is built from counts (untracked runs/ or out/ must not mark every build dirty); a plain copy (no .git) edited by hand after git archive is not seen, when to upgrade: hash the context
  [[ -n $top && $(cd "$top" && pwd -P) == "$repo" && -n $(git -C "$repo" status --porcelain -- docker deps src 2>/dev/null) ]]
}

# image_freshness IMAGE REPO: is the image built from what this checkout holds? Replaces a guard per feature ("an older image runs without /scan and without an error").
#   0 current; 1 stale (the reason on stdout); 2 cannot tell (the reason on stdout).
# Only what the image is built from, plus the host launcher, counts: a commit that touches docs, tools/ or the other ops/ scripts leaves the image current. With git history
# the label's commit is compared with the checkout's working tree (uncommitted edits count); without it (a copy made by git archive) the two commits must be equal.
image_freshness() {
  local image=$1 repo=$2 label sha cur top
  label=$(docker image inspect -f '{{index .Config.Labels "org.opencontainers.image.revision"}}' "$image" 2>/dev/null || true)
  sha=${label%-dirty}
  [[ $label == *-dirty ]] && { echo "$image was built from a tree with uncommitted edits ($label): nothing says what it holds; rebuild from a clean tree, or STALE_IMAGE_OK=1"; return 2; }
  [[ $sha =~ ^[0-9a-f]{40}$ ]] || { echo "$image carries no revision label (built by hand, or before ops/build_image.sh labelled images)"; return 2; }
  cur=$(checkout_revision "$repo")
  [[ -n $cur ]] || { echo "this checkout does not know its commit (no .git and no filled REVISION)"; return 2; }
  top=$(git -C "$repo" rev-parse --show-toplevel 2>/dev/null || true)
  if [[ -n $top && $(cd "$top" && pwd -P) == "$(cd "$repo" && pwd -P)" ]] && git -C "$repo" cat-file -e "$sha^{commit}" 2>/dev/null; then
    git -C "$repo" diff --quiet "$sha" -- src deps docker ops/nav_live.sh && return 0
    echo "$image was built from ${sha:0:7}; src/, deps/, docker/ or ops/nav_live.sh differ in this checkout (HEAD ${cur:0:7})"; return 1
  fi
  [[ $sha == "$cur" ]] && return 0
  echo "$image was built from ${sha:0:7}, this checkout is ${cur:0:7}, and there is no history here to tell whether the difference matters"; return 1
}
