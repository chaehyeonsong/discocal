#!/usr/bin/env bash
# -*- coding: UTF-8 -*-
#
# author        : JV-conseil
# credits       : JV-conseil
# copyright     : Copyright (c) 2019-2024 JV-conseil
#                 All rights reserved
#====================================================

# shellcheck source=/dev/null
. ".bash/incl/all.sh"

_jvcl_::pipenv_install() {
  find . -type f -name "Pipfile*" -print -delete
  pipenv --rm
  pipenv --clear # clear cache
  pipenv install --verbose -r ./requirements.txt
  pipenv run pip freeze -r ./requirements.txt | grep -E "## The following requirements were added by pip freeze:" -B 100
  echo
}

if _jvcl_::brew_install_formula "pipenv"; then
  _jvcl_::pipenv_install || printf "ERROR"
fi
