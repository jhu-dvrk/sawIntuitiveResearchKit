#!/bin/sh
dict=rename-to-crtk.dict
sed_input=rename-to-crtk-sed-input.tmp
sed -e 's/^\(.*\) = \(.*\)$/s\/\\(\\b\\|\\.\\)\1\\(\\b\\|)\\)\/\\1\2\\2\/g/' $dict > $sed_input
find . -name "*.cpp" -print0 | xargs -0 sed -f $sed_input -i
find . -name "*.h" -print0 | xargs -0 sed -f $sed_input -i
