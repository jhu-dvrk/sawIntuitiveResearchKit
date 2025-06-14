#!/bin/sh

print_usage ()
{
    echo "-d|--dir to specify directory, -s|--substitution-file to specify substitution dictionary"
}


if [ "$#" -ne 4 ]
then
    echo "need 4 arguments, $# provided"
    print_usage
    exit 1
fi

while [ $# -gt 0 ]
do
    key="$1"

    case $key in
	-d|--dir)
	    DIRECTORY="$2"
	    shift
	    shift
	    ;;
	-s|--substitution-file)
	    DICTIONARY="$2"
	    shift
	    shift
	    ;;
	*)    # unknown option
	    echo "Unknown option $1"
	    print_usage
	    exit 1
	    ;;
    esac
done

echo "DIRECTORY  = ${DIRECTORY}"
echo "DICTIONARY = ${DICTIONARY}"

if ! [ -d "${DIRECTORY}" ]; then
    echo "directory ${DIRECTORY} doesn't seem to exist"
    exit 1
fi

if ! [ -e "${DICTIONARY}" ]; then
    echo "file ${DICTIONARY} doesn't seem to exist"
    exit 1
fi

SED_INPUT="${DICTIONARY}-sed-input-generated.tmp"
echo "SED_INPUT  = ${SED_INPUT}"

sed -e 's/^\(.*\) = \(.*\)$/s\/\\(\\b\\|\\.\\)\1\\(\\b\\|)\\)\/\\1\2\\2\/g/' ${DICTIONARY} > ${SED_INPUT}

suffices="cpp c h py m json xml"

for suffix in ${suffices}; do
    echo "processing $suffix files"
    find ${DIRECTORY} -name "*.${suffix}" -print0 | xargs -0 sed -f ${SED_INPUT} -i
done
