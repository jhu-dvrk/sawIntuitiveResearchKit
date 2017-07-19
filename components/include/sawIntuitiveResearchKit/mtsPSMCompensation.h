/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Grace Chrysilla
  Created on: 2017-07-18

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#ifndef _mtsCompensation_h
#define _mtsCompensation_h

// cisst include
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmStateJoint.h>

class mtsPSMCompensation: public mtsTaskPeriodic {
	CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

protected:
	prmStateJoint mJointStateEncoder; 
	prmStateJoint mJointStateCompensated;

	// functions used in the interface required to send commands to PID component
	mtsFunctionRead GetJointState;

	// json value for joint 1
	double complianceA1, complianceB1, complianceC1, complianceD1;
	double torqueOffsetA1;
	double backlash1;

	// json value for joint 2
	double complianceA2, complianceB2, complianceC2, complianceD2;
	double torqueOffsetA2, torqueOffsetB2;
	double backlash2;

	void SetupInterfaces(void); // setting up both requiredInterface and providedInterface
	void ComputeCompensation();
	prmStateJoint GetCorrectedJointState() const;

public:
	mtsPSMCompensation(const std::string & componentName, double periodInSecond);
	~mtsPSMCompensation(){};

	// pure virtual methods in mtsTask
	void Configure(const std::string & filename); // to parse JSON file
	void Startup(void); 
	void Run(void);
	void Cleanup(void) {};
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsPSMCompensation);

#endif