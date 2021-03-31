#!/usr/bin/env python3

import os
import shutil
import sys
import argparse
import json
from json_schema_for_humans.generate import generate_from_schema, GenerationConfiguration

# parse arguments
parser = argparse.ArgumentParser(description = 'generate-html')

parser.add_argument('-d', '--directory', type = str, required = True,
                    help = 'directory containing the .schema.json files')
parser.add_argument('-v', '--version', type = str, required = True,
                    help = 'name of subdirectory to dump the generated files (e.g. v2.0)')

args = parser.parse_args(sys.argv[1:]) # skip argv[0], script name

print('Using schemas from: {}'.format(args.directory))

# the following is to be able to locate all possible $ref in schema
schema_store = {}
schema_files = []

# find all files in directory
all_files = os.listdir(args.directory)
for that_file in all_files:
  that_file_full_path = os.path.join(args.directory, that_file)
  extension = '.schema.json'
  if that_file_full_path[-len(extension):] == extension:
    print('Found possible schema file: {}'.format(that_file))
    with open(that_file_full_path, 'r') as schema_file_descriptor:
      that_schema = json.load(schema_file_descriptor)
      if '$id' in that_schema:
        print(' - $id: {}'.format(that_schema['$id']))
        schema_store[that_schema['$id']] = that_schema
        schema_files.append(that_file_full_path)
      else:
        print('WARNING: ignoring file {}, $id is missing'.format(that_file))

# create the output directory
if not os.path.exists(args.version):
  os.mkdir(args.version)

# now iterate through the same files to generate the html documentation
config = GenerationConfiguration(copy_css = True, expand_buttons = True)
for schema_file in schema_files:
  html = generate_from_schema(schema_file = schema_file, loaded_schemas = schema_store, config = config)
  # file name with version and without .schema.json
  html_file = os.path.normpath('{}/{}.html'.format(args.version, schema_file)).replace('.schema.json', '')
  print('Generating {}'.format(html_file))
  with open(html_file, 'w') as file_descriptor:
    file_descriptor.write(html)
  # also copy the original schema file
  shutil.copy(schema_file, args.version)
