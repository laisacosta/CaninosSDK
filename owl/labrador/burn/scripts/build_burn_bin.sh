#!/bin/sh

P=`pwd`
MAKER_DIR=${P}/../../../tools/fw_maker

cd $MAKER_DIR

rm -rf $MAKER_DIR/Output

./maker_install.run && cd $P

python -O $MAKER_DIR/Output/PyMaker.pyo -c $1 --forceunextract -o $2 --mf 1

rm -rf $MAKER_DIR/Output
