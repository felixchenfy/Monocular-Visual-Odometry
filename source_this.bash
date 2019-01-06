#!/bin/bash
# ./bin/run_vo config/default.yaml

mymake
bin/run_vo_v1 config/default.yaml
bin/test_epipolor_geometry image0001.jpg image0002.jpg
bin/test_epipolor_geometry image0012.jpg image0013.jpg