#!/bin/bash
cd modules/ChibiOS
# this uses git am to attempt to apply it, and then agains with --skip right after so we make the process non-fatal, ie if the patch *can* be applied, it is, if not, its assumed already done.
ls ../*.patch | xargs -n1 -i__  echo git am  __ \|\| git am --skip  | bash
