#!/bin/bash
# Check if $PIXI_PROJECT_ROOT/.pixi/envs/default/include/python3.12/CPyCppyy exists
if [ ! -d "$PIXI_PROJECT_ROOT/.pixi/envs/default/include/python3.12/CPyCppyy" ]; then
    echo "CPyCppyy not found in $PIXI_PROJECT_ROOT/.pixi/envs/default/include/python3.12/CPyCppyy"
    # Check if $PIXI_PROJECT_ROOT/.pixi/envs/default/include/python3.12/cpycppyy exists
    if [ ! -d "$PIXI_PROJECT_ROOT/.pixi/envs/default/include/python3.12/cpycppyy" ]; then
        echo "cpycppyy not found in $PIXI_PROJECT_ROOT/.pixi/envs/default/include/python3.12/cpycppyy"
        exit 1
    else
        # Create a symlink from cpycppyy to CPyCppyy
        echo "Found cpycppyy in $PIXI_PROJECT_ROOT/.pixi/envs/default/include/python3.12/cpycppyy, creating symlink to CPyCppyy"
        ln -s $PIXI_PROJECT_ROOT/.pixi/envs/default/include/python3.12/cpycppyy $PIXI_PROJECT_ROOT/.pixi/envs/default/include/python3.12/CPyCppyy
    fi
fi
