# Introduction

JSON schemas for dVRK configuration files.  We also provide a small python script to check configuration files against schemas.  The custom script adds a couple of features compared to the default `jsonschema` command:
  * Allow comments in the configuration file.  Internally comments are stripped using `jsmin`.  The C++ dVRK code uses JsonCpp which supports comments.
  * Check the syntax of both JSON files (configuration file and schema) using `json.load`.  Without this, incorrect JSON files are rejected without any useful error message.

Ideally, the C++ code should use the schemas to check the configuration files.  Unfortunately, the C++ library we use to parse JSON files doesn't support JSON schemas (yet).  So for now, the best approach is to test your configuration files as a separate step.

To allow testing offline, we provide two python scripts that will load all the schemas in this directory and maintain a dictionary of `$ref` and schemas used to check a configuration file or generate the html documentation.

# Check JSON dVRK configuration files

* Installation:
    ```sh
  pip3 install jsonschema referencing jsmin
  ```

* Usage:
  Assuming your working directory is the *dVRK* `share/schemas`:
    ```sh
  ./json-schema.py -s dvrk-console.schema.json <files-to-test>
  ```
  To test console files in multiple directories, for example all JHU console files:
    ```sh
  ./json-schema.py -s dvrk-console.schema.json ../console/console-* ../jhu-daVinci/console-* ../jhu-dVRK/console-*
  ```
  The output is the Python stack if any error is found.  You will need to read it all to figure out the issue.  If there are no issue found, the scripts outputs **All good**.

  To test arm files, make sure you use the right schema.  Examples below are for arms at JHU:
    ```sh
  ./json-schema.py -s dvrk-mtm.schema.json ../*/MTML-*.json ../*/MTMR-*.json
  ./json-schema.py -s dvrk-psm.schema.json ../*/PSM1-*.json ../*/PSM2-*.json ../*/PSM3-*.json
  ./json-schema.py -s dvrk-ecm.schema.json ../*/ECM-*.json
  ```

  To test the tool index files:
    ```sh
    ./json-schema.py -s dvrk-tool-list.schema.json ../tool/index.json ../jhu-dVRK/custom-tool/custom-tool-index.json
  ```

# Generate html documentation from schemas

This section is for the dVRK maintainers.  Most users should use the online documentation: https://dvrk.readthedocs.io/

* Installation:
    ```sh
  pip3 install json-schema-for-humans
  ```

* Usage:
    ```sh
  ./generate-html.py -d . -v v2.1
  ```

`-d` is for the directory containing all the `.schema.json` files.   `-v` is for a subdirectory for all the generated files.  If you are a dVRK maintainer, make sure you also upload the generated documentation to `https://dvrk.lcsr.jhu.edu/documentation/schemas/`.
