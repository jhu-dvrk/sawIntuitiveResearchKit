/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2021-02-07

  (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/mtsToolList.h>

#include <cisstCommon/cmnPath.h>

bool mtsToolList::Load(const cmnPath & path,
                       const std::string & indexFile)
{
    std::string fullFilename;

    // try to locate the file assuming absolute path
    if (cmnPath::Exists(indexFile)) {
        fullFilename = indexFile;
    } else {
        // else use path provided
        fullFilename = path.Find(indexFile);
        // still not found, try to add suffix to search again
        if (fullFilename == "") {
            CMN_LOG_CLASS_INIT_ERROR << "ToolList::Load: failed to locate tool index file for \""
                                     << indexFile << "\"" << std::endl;
            return false;
        }
    }

    // load index of supported tools
    try {
        std::ifstream jsonStream;
        Json::Value jsonConfig;
        Json::Reader jsonReader;

        jsonStream.open(fullFilename.c_str());
        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "ToolList::Load: failed to parse configuration file \""
                                     << fullFilename << "\"\n"
                                     << jsonReader.getFormattedErrorMessages();
            return false;
        }

        CMN_LOG_CLASS_INIT_VERBOSE << "ToolList::Load: using file \"" << fullFilename << "\"" << std::endl
                                   << "----> content of configuration file: " << std::endl
                                   << jsonConfig << std::endl
                                   << "<----" << std::endl;

        const Json::Value instruments = jsonConfig["instruments"];
        for (unsigned int index = 0; index < instruments.size(); ++index) {
            mtsIntuitiveResearchKitToolDescription * description = new mtsIntuitiveResearchKitToolDescription;
            try {
                cmnDataJSON<mtsIntuitiveResearchKitToolDescription>::DeSerializeText(*description, instruments[index]);
            } catch (std::runtime_error & e) {
                CMN_LOG_CLASS_INIT_ERROR  << "ToolList::Load: error found in file \""
                                          << fullFilename << "\": " << e.what() << std::endl;
                delete description;
                return false;
            }

            //
            // perform a bunch of sanity checks as it would be easy to
            // have errors in the tool index file
            //

            // make sure names at all upper case
            for (auto & name : description->names) {
                std::string upper_case_name = name;
                std::transform(upper_case_name.begin(), upper_case_name.end(), upper_case_name.begin(), ::toupper);
                if (name != upper_case_name) {
                    CMN_LOG_CLASS_INIT_WARNING << "ToolList::Load: issue found in file \""
                                               << fullFilename << "\": all names should be upper case, found \""
                                               << name << "\"" << std::endl;
                    name = upper_case_name;
                }
            }
            // make sure generation is S or Classic
            if ((description->generation != "Classic")
                && (description->generation != "S")) {
                CMN_LOG_CLASS_INIT_ERROR  << "ToolList::Load: error found in file \""
                                          << fullFilename << "\": generation must be either \"Classic\" or \"S\", not \""
                                          << description->generation << "\"" << std::endl;
                delete description;
                return false;
            }
            // make sure model number starts with 40 for Classic and 42 for S
            if (description->model.substr(0, 2) == "40") {
                if (description->generation != "Classic") {
                    CMN_LOG_CLASS_INIT_WARNING  << "ToolList::Load: issue found in file \""
                                                << fullFilename << "\": tool model \"" << description->model
                                                << "\" starts with 40 so it's generation should be \"Classic\", not \""
                                                << description->generation << "\"" << std::endl;
                }
            } else if (description->model.substr(0, 2) == "42") {
                if (description->generation != "S") {
                    CMN_LOG_CLASS_INIT_WARNING  << "ToolList::Load: issue found in file \""
                                                << fullFilename << "\": tool model \"" << description->model
                                                << "\" starts with 42 so it's generation should be \"S\", not \""
                                                << description->generation << "\"" << std::endl;
                }
            }
            // make sure file name contains one of the names
            bool nameFound = false;
            for (auto & name : description->names) {
                nameFound |= (description->file.find(name) != std::string::npos);
            }
            if (!nameFound) {
                CMN_LOG_CLASS_INIT_WARNING  << "ToolList::Load: issue found in file \""
                                            << fullFilename << "\": tool model \"" << description->model
                                            << "\" uses a file whose name doesn't contain any of the tool names: \""
                                            << description->file << "\"" << std::endl;
            }
            // make sure file name contains the model number
            std::size_t found = description->file.find(description->model);
            if (found == std::string::npos) {
                CMN_LOG_CLASS_INIT_WARNING  << "ToolList::Load: issue found in file \""
                                            << fullFilename << "\": tool model \"" << description->model
                                            << "\" uses a file whose name doesn't contain the model number: \""
                                            << description->file << "\"" << std::endl;
            }
            // finally, set index and add to all containers
            description->index = mTools.size();
            // main container
            mTools.push_back(description);
            // for faster searches
            typedef decltype(mToolsByModel)::value_type ModelValue;
            mToolsByModel.insert(ModelValue(description->model, description->index));
            typedef decltype(mToolsByNameModel)::value_type ModelNameValue;
            for (const auto & name : description->names) {
                mToolsByNameModel.insert(ModelNameValue(name + ":" + description->model, description->index));
            }
        }
    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "ToolList::Load: make sure the file \""
                                 << fullFilename << "\" is in JSON format" << std::endl;
        return false;
    }

    return true;
}

bool mtsToolList::Find(const std::string & toolName, size_t & index) const
{
    // remove the version number to search name + model
    bool hasVersion;
    int version;
    std::string nameModel;
    // search for [
    std::string::size_type versionStart = toolName.find('[');
    if (versionStart != std::string::npos) {
        hasVersion = true;
        nameModel = toolName.substr(0, versionStart);
        versionStart++; // to skip the [
        std::string::size_type versionEnd = toolName.find(']', versionStart);
        // make sure ] is there
        if (versionEnd == std::string::npos) {
            CMN_LOG_CLASS_INIT_ERROR << "ToolList::Find: tool name \""
                                     << toolName << "\" is missing the matching ] after the version number" << std::endl;
            return false;
        }
        // check for ".." in version, if just .., no version specified
        if (toolName.substr(versionStart, versionEnd - versionStart) == "..") {
            hasVersion = false;
            nameModel = toolName.substr(0, versionStart - 1);
        } else {
            // search for ".." to see if a range is specified
             std::string::size_type rangeStart = toolName.find("..", versionStart);
             if (rangeStart != std::string::npos) {
                 // now, 3 cases  m..M, m.. or ..M where m is minimum, M is maximum
                 // if there is a m version, just use it
                 if (versionStart != rangeStart) {
                     const auto versionString = toolName.substr(versionStart, rangeStart - versionStart);
                     version = std::stoi(versionString);
                 } else {
                     // use string from end or .. to ]
                     const size_t rangeEnd = rangeStart + 2;
                     const auto versionString = toolName.substr(rangeEnd, versionEnd - rangeEnd);
                     version = std::stoi(versionString);
                 }
             } else {
                 // last case, no ".." in version
                 const auto versionString = toolName.substr(versionStart, versionEnd - versionStart);
                 version = std::stoi(versionString);
             }
        }
    } else {
        hasVersion = false;
        nameModel = toolName;
    }

    // then look for the model and use version if and only if we have multiple options
    auto nb_found =  mToolsByNameModel.count(nameModel);
    // not found
    if (nb_found == 0) {
        return false;
    }

    // we have at least one definition
    auto range = mToolsByNameModel.equal_range(nameModel);

    // if there's only one, let's use it
    if (nb_found == 1) {
        index = (range.first)->second;
        return true;
    } else if (nb_found > 1) {
        if (!hasVersion) {
            CMN_LOG_CLASS_INIT_ERROR << "ToolList::Find: found multiple tools matching the name \""
                                     << toolName << "\", no way to determine which one to pick without a version" << std::endl;
            return false;
        } else {
            for (auto & it = range.first;
                 it != range.second; ++it) {
                if ((version >= mTools.at(it->second)->version_min)
                    && (version <= mTools.at(it->second)->version_max)) {
                    index = it->second;
                    return true;
                }
            }
        }
    }

    // other cases
    return false;
}

std::string mtsToolList::File(const size_t & index) const
{
    return mTools.at(index)->file;
}

std::string mtsToolList::Name(const size_t & index) const
{
    return mTools.at(index)->names.at(0)
        + ":" + mTools.at(index)->model + VersionDescription(index);
}

std::string mtsToolList::VersionDescription(const size_t & index) const
{
    std::string result = "[";
    if (mTools.at(index)->version_min != mtsIntuitiveResearchKitToolDescription::VERSION_MIN_DEFAULT) {
        result += std::to_string(mTools.at(index)->version_min);
    }
    result += "..";
    if (mTools.at(index)->version_max != mtsIntuitiveResearchKitToolDescription::VERSION_MAX_DEFAULT) {
        result += std::to_string(mTools.at(index)->version_max);
    }
    result += "]";
    return result;
}

std::string mtsToolList::Description(const size_t & index) const
{
    return mTools.at(index)->description;
}

std::string mtsToolList::FullDescription(const size_t & index) const
{
    return mTools.at(index)->description
        + " for da Vinci " + mTools.at(index)->generation
        + " (" + mTools.at(index)->model + VersionDescription(index) + ")";
}

std::string mtsToolList::PossibleNames(const std::string & divider) const
{
    std::string result;
    for (size_t index = 0;
         index < mTools.size();
         ++index) {
        if (index != 0) {
            result.append(divider);
        }
        result.append(Name(index));
    }
    return result;
}
