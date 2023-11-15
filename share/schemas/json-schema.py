#!/usr/bin/env python3

# simple program to load a JSON schema and file to validate.  this is
# a bit better than the jsonschema program provided with jsonschema
# since it will first check the JSON syntax of both files.  we also
# use jsmin to remove comments from configuration file.

# resolving references should be simple but I can't figure it out.  So
# I implemented something hackish based on
# https://stackoverflow.com/questions/42159346/jsonschema-refresolver-to-resolve-multiple-refs-in-python

import os
import sys
import argparse
import jsmin
import json
import jsonschema

# parse arguments
parser = argparse.ArgumentParser(description = 'json-schema')

parser.add_argument('-s', '--schema', type = str, required = True,
                    help = 'json schema used to validate file')

parser.add_argument('files', metavar = 'N', type = str, nargs = '+',
                    help = 'a JSON file to validate')

args = parser.parse_args(sys.argv[1:]) # skip argv[0], script name

# we let the following calls to "load" and "validate" throw
# exceptions, users have to read the trace back to understand the
# issues
with open(args.schema) as s:
  schema = json.load(s)

# get the directory for the schema
tmp = open(args.schema)
current_schema_directory = os.path.dirname(os.path.realpath(tmp.name))
print('Using schemas from: {}'.format(current_schema_directory))

# the following is to be able to locate all possible $ref in schema
schema_store = {}

# find all files in directory
all_files = os.listdir(current_schema_directory)
for that_file in all_files:
  that_file_full_path = os.path.join(current_schema_directory, that_file)
  extension = '.schema.json'
  if that_file_full_path[-len(extension):] == extension:
    print('Found possible schema file: {}'.format(that_file))
    with open(that_file_full_path, 'r') as schema_file_descriptor:
      that_schema = json.load(schema_file_descriptor)
      if '$id' in that_schema:
        print(' - $id: {}'.format(that_schema['$id']))
        schema_store[that_schema['$id']] = that_schema

resolver = jsonschema.RefResolver.from_schema(schema, store = schema_store)
validator = jsonschema.Draft7Validator(schema, resolver = resolver)

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
