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
A(2,:)       = [ 1 4 9 16 25 ]

"""

import numpy as np
from numpy.core.fromnumeric import var
from numpy.lib.shape_base import column_stack


class Context:
    def __init__(self, variables: dict):
        self.variables = variables

    # If string is Matlab variable name, replace with literal value from context
    # Otherwise, parse as string/float/int literal
    def parse(self, string: str):
        # known variable
        if string in self.variables:
            return self.variables[string]
        # string literal
        elif len(string) >= 2 and string[0] == "'" and string[-1] == "'":
            return string[1:-1]
        # float literal
        elif string.find(".") != -1:
            return float(string)
        # otherwise, assume it is an integer
        else:
            return int(string)


class Indices:
    # Start and end indices of range, inclusive
    def __init__(self, start: int, end: int):
        self.start = start
        self.end = end

    @staticmethod
    def _parseIndex(index: int, context: Context):
        if index == "":
            return None

        try:
            index = context.parse(index)
        except ValueError:
            raise SyntaxError("Malformed index in cal file")

        if not isinstance(index, int):
            raise SyntaxError("Indices must be integers")

        return index

    @staticmethod
    def parse(string: str, context: Context):
        if string.find(":") == -1:
            index = Indices._parseIndex(string, context)
            return Indices(index - 1, index - 1)

        indices = string.split(":")
        assert len(indices) == 2, "Malformed index range"

        start = Indices._parseIndex(indices[0], context) or 1
        end = Indices._parseIndex(indices[1], context)
        assert start == 1, "Vector index range must start at 1"
        assert end is None or start <= end, "Malformed vector index range end"

        return Indices(start - 1, None if end is None else end - 1)
        
    def slice(self):
        return slice(self.start, None if self.end is None else self.end + 1)

    def valid(self):
        return self.start >= 0 and self.end >= self.start

    def size(self):
        if self.end is None:
            return None

        return self.end + 1 - self.start


class AssignmentNode:
    def __init__(self, parent):
        self.parent = parent

    @staticmethod
    def _parseIndices(segment: str, context: Context):
        segment = segment[1:-1]
        indices = segment.split(",")
        indices = [Indices.parse(x, context) for x in indices]
        
        if len(indices) == 1:
            return Indices(0, 0), indices[0]
        
        return indices

    @staticmethod
    def _createNode(parent, isKey: bool, pathSegment: str, context: Context):
        if isKey:
            return KeyAssignment(parent, pathSegment)
        else:
            rows, columns = AssignmentNode._parseIndices(pathSegment, context)
            return IndexAssignment(parent, rows, columns)

    @staticmethod
    def parse(path: str, context: Context):
        node = None

        index = 0
        isKeyAssignment = True
        while index < len(path):
            if path[index] == "." or path[index] == "(":
                node = AssignmentNode._createNode(node, isKeyAssignment, path[:index], context)

                if path[index] == ".":
                    isKeyAssignment = True
                    path = path[index+1:]
                    index = 0
                else:
                    isKeyAssignment = False
                    path = path[index:]
                    index = 1
            else:
                index += 1

        node = AssignmentNode._createNode(node, isKeyAssignment, path.strip(), context)
        return node

    def getFrom(self, variables: dict):
        raise NotImplementedError("Please implement getFrom(variables)")

    def assignTo(self, variables: dict, value: any):
        raise NotImplementedError("Please implement assignTo(variables, value)")


class KeyAssignment(AssignmentNode):
    def __init__(self, parent: AssignmentNode, key: str):
        super().__init__(parent)
        self.key = key

    def getFrom(self, variables: dict):
        data = self.parent.getFrom(variables) if self.parent is not None else variables
        if data is None:
            return None

        try:
            return data[self.key]
        except KeyError:
            return None

    def assignTo(self, variables: dict, value: any):
        if self.parent is None:
            variables[self.key] = value
        else:
            data = self.parent.getFrom(variables) or {}
            data[self.key] = value
            self.parent.assignTo(variables, data)


class IndexAssignment(AssignmentNode):
    def __init__(self, parent: AssignmentNode, rows: Indices, columns: Indices):
        assert parent is not None, "IndexAssignment cannot be root of an AssignmentPath"
        assert isinstance(parent, KeyAssignment), "IndexAssignment must be a child of a KeyAssignment"
    
        super().__init__(parent)
        self.rows = rows
        self.columns = columns

    def getFrom(self, variables: dict):
        data = self.parent.getFrom(variables)
        if data is None:
            return None

        try:
            if self.rows.size() == 1 and self.columns.size() == 1:
                return data[self.rows.start, self.columns.start]

            return data[self.rows.slice(), self.columns.slice()]
        except IndexError:
            return None

    def assignTo(self, variables: dict, value: np.array):
        data = self.parent.getFrom(variables)
        if data is None:
            data = np.array([[]])

        self._realizeIndices(value)
        self._validateIndices(value)

        data = self._padToFit(data, value)
        data[self.rows.slice(), self.columns.slice()] = value
        self.parent.assignTo(variables, data)

    def _arrayDimensions(self, array: np.array):
        size = np.shape(array)
        if len(size) == 0:
            return 1, 1
        elif len(size) == 1:
            return 1, size[0]
        else:
            return size

    def _realizeIndices(self, value: np.array):
        rows, columns = self._arrayDimensions(value)
        
        if self.rows.end is None:
            self.rows = Indices(0, rows - 1)

        if self.columns.end is None:
            self.columns = Indices(0, columns - 1)

    def _validateIndices(self, value: np.array):
        rows, columns = self._arrayDimensions(value)

        assert self.rows.valid(), "Range assignment invalid row indices"
        assert self.columns.valid(), "Range assignment invalid column indices"
        
        assert self.rows.size() == rows, "Range assignment row size mismatch"
        assert self.columns.size() == columns, "Range assignment column size mismatch"

    # Zero-pad matrix if necessary to accommodate new assignment
    def _padToFit(self, data: np.array, value: np.array):
        rows, columns = np.shape(data)
        padRows = max(0, self.rows.end + 1 - rows)
        padColumns = max(0, self.columns.end + 1 - columns)
        
        # Zero-pad numeric arrays, None-pad object arrays
        numeric_array = isinstance(value, np.ndarray) and value.dtype != np.dtype(object)
        if numeric_array:
            return np.pad(data, ((0, padRows), (0, padColumns)))
        else:
            data = data.astype(object)
            return np.pad(data, ((0, padRows), (0, padColumns)), constant_values=None)


class ArrayParser:
    def __init__(self, line: str, context: Context):
        self.line = line.strip()
        self.context = context
        self.position = 0

    def parse(self):
        self.position += 1
        values = []

        while self.position < len(self.line) and not self.line[self.position] == "]":
            if self.line[self.position] == "[":
                subarray = self.parse()
                self.position += 1
                values.append(subarray)
            elif self._atStringDelimiter():
                string = self._parseString()
                values.append(string)
            elif self._atNumberStart():
                number = self._parseNumber()
                values.append(number)
            else:
                self.position += 1

        return values

    def _atStringDelimiter(self):
        char = self.line[self.position]
        return char == "'" or char == '"'

    def _atNumberStart(self):
        char = self.line[self.position]
        return char == "-" or char.isdigit()

    def _atNumber(self):
        char = self.line[self.position]
        return char == "." or char.isdigit() or char == "e" or char == "-"

    def _parseString(self):
        stringStart = self.position
        self.position += 1

        while not self._atStringDelimiter():
            self.position += 1

        self.position += 1
        return self.context.parse(self.line[stringStart:self.position])

    def _parseNumber(self):
        numberStart = self.position
        self.position += 1

        while self._atNumber():
            self.position += 1

        return self.context.parse(self.line[numberStart:self.position])


class CalParser:
    def __init__(self, variables):
        self.context = Context(variables)

    # All r-values in a .cal file are arrays
    def _parseValue(self, value):
        parser = ArrayParser(value, self.context)
        values = parser.parse()
        return np.array(values)

    # Remove unnecessary matrix dimensions, and convert NumPy types
    # into their standard Python equivalents
    def _simplifyValues(self, variables):
        processed = {}

        # Recursively reduce along primary dimension so that 1x1 matrices
        # become length 1 arrays and then scalars, and 1x5 matrices become
        # length 5 arrays, etc.
        def simplifyArray(value: np.ndarray):
            shape = np.shape(value)
            if len(shape) == 0:
                return value
            elif len(shape) == 1:
                return value.item(0) if shape[0] == 1 else value.tolist()
            elif len(shape) == 2:
                return simplifyArray(value[0]) if shape[0] == 1 else value.tolist()

        for variable_name in variables:
            value = variables[variable_name]

            if isinstance(value, dict):
                processed[variable_name] = self._simplifyValues(value)
            elif isinstance(value, np.ndarray):
                processed[variable_name] = simplifyArray(value)
            else:
                raise ValueError("Raw variables can only be dicts or Numpy arrays")

        return processed

    # Parses CAL file specified by fileName
    # When possible, variables in CAL file are replaced with values from context
    # Returns data as dictionary whose keys are variable names
    def parseFile(self, fileName):
        # map from Matlab variable name to value
        data = {}

        # first, read each assignment from CAL file and add to data
        with open(fileName, "r") as cal:
            for line in cal:
                line = line.strip()  # remove leading and trailing whitespace
                if len(line) == 0 or line[0] == "%":
                    # skip comments and blank lines
                    continue

                assignmentIdx = line.find("=")
                if assignmentIdx < 1 or assignmentIdx >= len(line):
                    raise SyntaxError(
                        "Invalid syntax in CAL file: all statements must be variable assignments."
                    )

                assignmentPath = AssignmentNode.parse(line[0:assignmentIdx], self.context)
                value = self._parseValue(line[assignmentIdx + 1:])
                assignmentPath.assignTo(data, value)

        data = self._simplifyValues(data)
        return data
