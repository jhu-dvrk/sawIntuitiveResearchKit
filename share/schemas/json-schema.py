#!/usr/bin/env python

# simple program to load a JSON schema and file to validate.  this is
# a bit better than the jsonschema program provided with jsonschema
# since it will first check the JSON syntax of both files.  we also
# use jsmin to remove comments from configuration file.

import sys
import argparse
import jsmin
import json
import jsonschema

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('-f', '--file', type=str, required=True,
                    help = 'json file to load')
parser.add_argument('-s', '--schema', type=str, required=True,
                    help = 'json schema used to validate file')
args = parser.parse_args(sys.argv[1:]) # skip argv[0], script name

# we let the following calls to "load" and "validate" throw
# exceptions, users have to read the trace back to understand the
# issues
with open(args.file) as f:
  fileNoComments = jsmin.jsmin(f.read())
  file = json.loads(fileNoComments)

with open(args.schema) as s:
  schema = json.load(s)

jsonschema.validate(instance = file, schema = schema)

print ('All good')
