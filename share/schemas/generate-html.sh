#!/bin/sh

# quick hack, this should be a Makefile or proper script with option to set output directory

generate-schema-doc --deprecated-from-description dvrk-console.schema.json v2.0/dvrk-console.html
generate-schema-doc cisst-component-manager.schema.json --deprecated-from-description v2.0/cisst-component-manager.html
