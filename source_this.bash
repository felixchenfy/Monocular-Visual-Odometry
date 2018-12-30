#!/bin/bash
# ./bin/run_vo config/default.yaml

cd build && make
cd ..
./bin/main config/default.yaml
