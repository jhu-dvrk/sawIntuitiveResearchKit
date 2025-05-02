/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-17

  (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _dvrk_IO_proxy_h
#define _dvrk_IO_proxy_h

#include <cisstMultiTask/mtsForwardDeclarations.h>
#include <cisstMultiTask/mtsFunctionVoid.h>

#include <sawIntuitiveResearchKit/IO_proxy_configuration.h>

// Always include last!
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class mtsRobotIO1394;
class mtsIntuitiveResearchKitConsole;

namespace dvrk {
    
    class CISST_EXPORT IO_proxy {
     public:
    
        friend class ::mtsIntuitiveResearchKitConsole;
        friend class arm_proxy;
    
        std::string m_name;
        mtsIntuitiveResearchKitConsole * m_console = nullptr;
        dvrk::IO_proxy_configuration * m_config = nullptr;
        
        IO_proxy(const std::string & name,
                 mtsIntuitiveResearchKitConsole * console,
                 dvrk::IO_proxy_configuration * config);
        
        NOT_COPYABLE(IO_proxy);
        NOT_MOVEABLE(IO_proxy);
    
        /*! Configure, i.e. load json and validate. */
        void post_configure(void);
        
        /*! Create and configure IO component. */
        void create_IO(void);
         
     protected:
        std::shared_ptr<mtsRobotIO1394> m_IO = nullptr;
        mtsFunctionVoid close_all_relays;
        mtsInterfaceRequired * m_interface_required;
    };
}

#endif // _dvrk_IO_proxy_h
