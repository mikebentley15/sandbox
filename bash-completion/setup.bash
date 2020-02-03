SCRIPT_DIR="$(readlink -f "$(dirname "${BASH_SOURCE[0]}")")"

function _pathmunge() {
  local varname="$1"
  local toadd="$2"
  if [ -d "$toadd" ]; then
    case "${!varname}" in
      *$toadd*)
        ;;
      *)
        export ${varname}="${!varname}:$toadd"
        ;;
    esac
  fi
}

_pathmunge PATH "$SCRIPT_DIR/bin"
unset -f _pathmunge

# include completions
for file in "$SCRIPT_DIR/completions"/*; do
  source "$file"
done

unset file
unset SCRIPT_DIR
