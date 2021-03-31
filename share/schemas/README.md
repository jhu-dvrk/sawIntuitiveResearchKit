# Introduction

JSON schemas for dVRK configuration files.  We also provide a small python script to check configuration files against schemas.  The custom script adds a couple of features compared to the default `jsonschema` command:
  * Allow comments in the configuration file.  Internally comments are stripped using `jsmin`.  The C++ dVRK code uses JsonCpp which supports comments.
  * Check the syntax of both JSON files (configuration file and schema) using `json.load`.  Without this, incorrect JSON files are rejected without any useful error message.

Ideally, the C++ code should use the schemas to check the configuration files.  Unfortunately, the C++ library we use to parse JSON files doesn't support JSON schemas (yet).  So for now, the best approach is to test your configuration files as a separate step.

To allow testing offline, we provide two python scripts that will load all the schemas in this directory and maintain a dictionary of `$ref` and schemas used to check a configuration file or generate the html documentation.

# Check JSON dVRK configuration files

* Installation:
    ```sh
  pip3 jsonschema jsmin
  ```

* Usage:
  Assuming your working directory is the *dVRK* `share/schemas`:
    ```sh
  ./json-schema.py -s console.schema.json <files-to-test>
  ```
  The output is the Python stack if any error is found.  You will need to read it all to figure out the issue.  If there are no issue found, the scripts outputs **All good**.


# Generate html documentation from schemas

* Installation:
    ```sh
  pip3 install json-schema-for-humans
  ```

* Usage:
    ```sh
  ./generate-html.py -d . -v v2.0
  ```

`-d` is for the directory containing all the `.schema.json` files.   `-v` is for a subdirectory for all the generated files.
