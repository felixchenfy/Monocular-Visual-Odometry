#!/bin/bash
# ./bin/run_vo config/default.yaml

cd build && make -j4
cd ..
# ./bin/main config/default.yaml
bin/run_vo_v1 config/default.yaml