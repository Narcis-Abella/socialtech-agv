#!/bin/bash
# Send an image built here to other boards, so nothing is built on an 8 GB Nano: docker save | ssh | docker load, then move the alias there.
#   ops/ship_image.sh robot jetson@socialtech-nano [jetson@socialtech-nx ...]      env: SSH_KEY (identity file)
# Skips the transfer when the board already has socialtech:<target>-<sha>. Rollback on a board: docker tag socialtech:<target>-<old sha> socialtech:<target>
set -euo pipefail
TARGET=${1:?usage: ship_image.sh robot|mapper|sim HOST [HOST...]}; shift
(($#)) || { echo "usage: ship_image.sh robot|mapper|sim HOST [HOST...]" >&2; exit 2; }
[[ $TARGET =~ ^(robot|mapper|sim)$ ]] || { echo "unknown target '$TARGET' (robot|mapper|sim)" >&2; exit 2; }
rev=$(docker image inspect -f '{{index .Config.Labels "org.opencontainers.image.revision"}}' "socialtech:$TARGET")
[[ -n $rev ]] || { echo "socialtech:$TARGET has no revision label: build it with ops/build_image.sh" >&2; exit 1; }
sha=${rev%-dirty}; tag=$TARGET-${sha:0:7}; [[ $rev != *-dirty ]] || tag+=-dirty
ssh_opts=(-o ConnectTimeout=10 ${SSH_KEY:+-i "$SSH_KEY"})
id=$(docker image inspect -f '{{.Id}}' "socialtech:$tag"); arch=$(docker image inspect -f '{{.Architecture}}' "socialtech:$tag")
for host in "$@"; do
  harch=$(ssh "${ssh_opts[@]}" "$host" "docker info -f '{{.Architecture}}'" | sed 's/x86_64/amd64/; s/aarch64/arm64/')
  [[ $harch == "$arch" ]] || { echo "$host: it runs $harch and socialtech:$tag is $arch (built on a laptop? build on a board): not sent" >&2; exit 1; }
  # same NAME is not same IMAGE (a -dirty tag is rebuilt under one name): the content id decides
  if [[ $(ssh "${ssh_opts[@]}" "$host" "docker image inspect -f '{{.Id}}' socialtech:$tag 2>/dev/null") == "$id" ]]; then echo "$host: socialtech:$tag is already there"
  else echo "$host: sending socialtech:$tag"; docker save "socialtech:$tag" | gzip | ssh "${ssh_opts[@]}" "$host" "gunzip | docker load"; fi
  ssh "${ssh_opts[@]}" "$host" "docker tag socialtech:$tag socialtech:$TARGET"
  echo "$host: socialtech:$TARGET -> $tag"
done
