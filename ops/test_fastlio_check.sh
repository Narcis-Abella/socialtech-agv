#!/bin/bash
# Run: bash ops/test_fastlio_check.sh
set -u
here=$(cd "$(dirname "$0")" && pwd)
source "$here/fastlio_check.sh"
d=$(mktemp -d); trap 'rm -f "$d"/*.yaml; rmdir "$d"' EXIT
fail=0
expect() {   # expect ok|bad NAME FILE key=value ...
  local want=$1 name=$2 file=$3; shift 3
  if fastlio_check "$file" "$@" 2>/dev/null; then got=ok; else got=bad; fi
  [[ $got == "$want" ]] || { echo "FAIL: $name -> $got, expected $want"; fail=1; }
}
cat > "$d/good.yaml" <<'EOF'
mapping:
    gyr_cov: 0.01
    acc_cov: 1.0
    filter_size_map: 0.25
    point_filter_num: 2
    blind: 0.1
EOF
expect ok "all keys exact" "$d/good.yaml" gyr_cov=0.01 acc_cov=1.0 filter_size_map=0.25 point_filter_num=2 blind=0.1
expect bad "a different value" "$d/good.yaml" filter_size_map=0.5
expect bad "a prefix of the value is not the value" "$d/good.yaml" filter_size_map=0.2
sed 's/filter_size_map: 0.25/filter_size_map: 0.255/' "$d/good.yaml" > "$d/longer.yaml"
expect bad "0.255 must not pass for 0.25" "$d/longer.yaml" filter_size_map=0.25
grep -v filter_size_map "$d/good.yaml" > "$d/missing.yaml"
expect bad "a key the upstream file lost" "$d/missing.yaml" filter_size_map=0.25
{ cat "$d/good.yaml"; echo "    filter_size_map: 0.25"; } > "$d/dup.yaml"
expect bad "a duplicated key" "$d/dup.yaml" filter_size_map=0.25
sed 's/^    filter_size_map: 0.25/    # filter_size_map: 0.25/' "$d/good.yaml" > "$d/commented.yaml"
expect bad "only a commented line" "$d/commented.yaml" filter_size_map=0.25
((fail == 0)) && echo "ok"
exit $fail
