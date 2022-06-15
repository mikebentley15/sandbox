#!/usr/bin/env bash

python -m venv venv
ln -s venv/bin/activate
source activate
python -m pip install --upgrade pip
pip install --requirement env.txt
doit tabcompletion > .doit.bash_completion
sed -i \
  -e '/unset -f deactivate/a\' \
  -e '        ### section start by setup-venv.sh ###\' \
  -e '        complete -r doit\' \
  -e '        unset -f _doit\' \
  -e '        ### section start by setup-venv.sh ###' \
  $(readlink -f activate)
echo                                          >> activate
echo "### section start by setup-venv.sh ###" >> activate
echo "if [ -f .doit.bash_completion ]; then"  >> activate
echo "    source .doit.bash_completion"       >> activate
echo "fi"                                     >> activate
echo "### section end by setup-venv.sh ###"   >> activate

