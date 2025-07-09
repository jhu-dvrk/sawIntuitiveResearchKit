#!/usr/bin/env python3

# simple program to load a JSON schema and file to validate.  this is
# a bit better than the jsonschema program provided with jsonschema
# since it will first check the JSON syntax of both files.  we also
# use jsmin to remove comments from configuration file.

import sys
import argparse
import jsmin
import json
import jsonschema
import referencing
import pathlib

# parse arguments
parser = argparse.ArgumentParser(description = 'json-schema')

parser.add_argument('-s', '--schema', type = str, required = True,
                    help = 'json schema used to validate file')

parser.add_argument('files', metavar = 'N', type = str, nargs = '+',
                    help = 'a JSON file to validate')

args = parser.parse_args(sys.argv[1:]) # skip argv[0], script name

schema_path = pathlib.Path(args.schema).resolve()
with open(schema_path) as s:
  schema = json.load(s)

schema_dir = schema_path.parent
print('Using schemas from: {}'.format(schema_dir))

def retrieve_callback(filename: str):
  path = schema_dir / filename
  contents = json.loads(path.read_text())
  return referencing.Resource.from_contents(contents)

registry = referencing.Registry(retrieve=retrieve_callback)
validator = jsonschema.Draft7Validator(schema, registry=registry)

# we let the following calls to "load" and "validate" throw
# exceptions, users have to read the trace back to understand the
# issues

# finally loop over files provided
for json_file in args.files:
  print('Validating file: {}'.format(json_file))
  with open(json_file) as f:
    # remove comments
    file_no_comments = jsmin.jsmin(f.read())
    try:
      file = json.loads(file_no_comments)
    except Exception as err:
      print("Unexpected {}".format(err))
      print(file_no_comments)
      raise
    validator.validate(file)

print ('All good')
