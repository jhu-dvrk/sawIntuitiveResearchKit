"""
Parser for reading variables from .cal files written in Matlab syntax

Given a .cal file, the output is a Python dictionary whose keys are the
names of the variables from the Matlab file, and whose values are
floats, strings, or NumPy arrays of floats. Cell arrays and matrices
will become NumPy arrays, 1x1 vectors will become floats or strings.
Other types are not supported, and .cal file must only contain variable
declarations in terms of raw numbers - that is constants, computations,
functions, etc. are not allowed.

The one exception to the raw-numbers-only-rule is the matrix/vector
variables may include indices in their declaration, and those indices
may make use of variables injected via the 'context' parameter, which
should also a dictionary. The types of indexing allowed is currently
very limited: vectors may have indices from 1:N and matrices may have
indices of the form (ROW,1:N).

Examples of valid declarations, assuming that
    context = {
        "S": 5,
        "L": 2,
    }

FileType     = ['MTM_CAL']
FileVersion  = [ 302 ]
valuesA      = [ 603 204 122 ]
valuesB(1:3) = [ 603 204 122 ]
valuesC(1:L) = [ 603 204 ]
A(1,1:S)     = [ 1 2 3 4 5 ]
A(2,1:S)     = [ 1 4 9 16 25 ]

"""

import numpy as np

# If x is variable, replace with value from context
def parseInContext(x, context):
    if x in context:
        return context[x]

    return int(x)


# CAL keys can either just be variable name,
# or include MATLAB-style array-subscripting.
# e.g. DATA(1:MST_MOT_DOFS)
def parseCALKey(key, context):
    key = key.strip()  # remove leading/trailing whitespace

    # No indices specified
    if key.find("(") == -1:
        return (key, (0, 0, 0))

    keyName = key[0 : key.find("(")]
    indices = key[key.find("(") + 1 : -1]

    # Vector indices, but not matrix
    if key.find(",") == -1:
        indices = indices.split(":")
        assert len(indices) == 2, "Malformed vector index range"
        startIndex = parseInContext(indices[0], context)
        endIndex = parseInContext(indices[1], context)
        assert startIndex == 1, "Vector index range must start at 1"
        assert endIndex >= startIndex, "Malformed vector index range end"
        return (keyName, (0, startIndex - 1, endIndex - 1))

    # Matrix indices
    indices = indices.split(",")
    assert len(indices) == 2, "Malformed matrix index in cal file"

    row = parseInContext(indices[0], context)
    assert row >= 1, "Invalid matrix row index"
    colIndices = indices[1].split(":")
    assert len(colIndices) == 2, "Malformed matrix column index in cal file"
    startIndex = parseInContext(colIndices[0], context)
    endIndex = parseInContext(colIndices[1], context)
    assert startIndex == 1, "Matrix column index range must start at 1"
    assert endIndex >= startIndex, "Malformed natrix column index range end"

    return (
        keyName,
        (row - 1, startIndex - 1, endIndex - 1),
    )


# Parses a single value
def parseCALValue(value):
    value = value.strip()  # remove leading/trailing whitespace

    # CAL values are formatted as lists, i.e. '[ value ]',
    # so we check to make square it has square brackets
    if value[0] != "[" or value[-1] != "]":
        raise SyntaxError

    # remove square brackets
    value = value[1:-1].strip()

    # list delimiter is space
    raw_values = []
    processed_values = []
    # if list elements are strings, make sure we aren't splitting
    # on spaces inside of strings
    if value.find("'") != -1:
        raw_values = value.split("' '")
    else:
        raw_values = value.split(" ")

    for v in raw_values:
        # string value, leave as exact contents
        if v[0] == "'" and v[-1] == "'":
            processed_values.append(v[1:-1])
        # float value
        elif v.find('.') != -1:
            processed_values.append(float(v))
        # otherwise, assume it is an integer
        else:
            processed_values.append(int(v))

    # return as value rather than list if only one element
    if len(processed_values) == 1:
        return processed_values[0]
    else:
        return processed_values


# Parses CAL file specified by fileName
# When possible, variables in CAL file are replaced with values from context
# Returns data as dictionary whose keys are variable names
def parseCALFile(fileName, context):
    raw_values = {}
    with open(fileName, "r") as cal:
        for line in cal:
            line = line.strip()  # remove leading and trailing whitespace
            if len(line) == 0 or line[0] == "%":
                # skip comments and blank lines
                continue

            assignmentIdx = line.find("=")
            if assignmentIdx < 1 or assignmentIdx >= len(line):
                raise SyntaxError

            key, indices = parseCALKey(line[0:assignmentIdx], context)
            value = parseCALValue(line[assignmentIdx + 1 :])

            if isinstance(value, list):
                assert (indices[2]-indices[1]+1) == len(value), "Incompatible shapes: " + str(indices[2]-indices[1]+1) + ", " + str(value)

            if key in raw_values:
                raw_values[key].append((indices, value))
            else:
                raw_values[key] = [(indices, value)]

    data = {}
    for key in raw_values:
        values = raw_values[key]
        if len(values) == 1:
            data[key] = values[0][1]
            continue

        max_row = max(values, key=lambda x: x[0][0])[0][0]
        max_column = max(values, key=lambda x: x[0][2])[0][2]
        value = np.zeros((max_row+1, max_column+1))

        for x in values:
            indices, raw_value = x
            row, start, end = indices
            value[row, start:end+1] = raw_value

        data[key] = value

    return data
