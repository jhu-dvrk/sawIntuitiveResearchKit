/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2021-07-02

  (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsToolList_h
#define _mtsToolList_h

#include <cisstCommon/cmnGenericObject.h>
#include <cisstCommon/cmnLogger.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitToolTypes.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class CISST_EXPORT mtsToolList
{
 public:
    inline mtsToolList(const cmnGenericObject & owner):
        OwnerServices(owner.Services())
    {}

    inline ~mtsToolList() {}

    bool Load(const cmnPath & path,
              const std::string & file);

    bool Find(const std::string & toolName, size_t & index) const;

    std::string File(const size_t & index) const;

    std::string Name(const size_t & index) const;

    std::string Description(const size_t & index) const;

    std::string FullDescription(const size_t & index) const;

    inline size_t size(void) const {
        return mTools.size();
    }

 protected:

    // for logs
    const cmnClassServicesBase * OwnerServices;

    inline const cmnClassServicesBase * Services(void) const {
        return this->OwnerServices;
    }

    inline cmnLogger::StreamBufType * GetLogMultiplexer(void) const {
        return cmnLogger::GetMultiplexer();
    }

    // actual list of tools by index
    std::vector<mtsIntuitiveResearchKitToolDescription *> mTools;

    // for faster searches and checks when adding tools
    std::multimap<std::string, size_t> mToolsByModel; // e.g. 400006
    std::multimap<std::string, size_t> mToolsByNameModel; // e.g. LARGE_NEEDLE_DRIVER_400006
};

#endif // _mtsToolList_h
