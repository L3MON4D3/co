#!/bin/bash
cd captures
for f in *; do dot -Tsvg "$f" -O ; done
