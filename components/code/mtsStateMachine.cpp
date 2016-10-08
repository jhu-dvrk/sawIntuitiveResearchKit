/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-02-05

  (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/mtsStateMachine.h>

void mtsStateMachine::AddState(const StateType state)
{
    if (StateExists(state)) {
        cmnThrow("mtsStateMachine::AddState: "
                 + mName + ", state already exists");
    }
    mStates[state] = false;
}

void mtsStateMachine::AddStates(const std::vector<StateType> & states)
{
    typedef std::vector<StateType> VectorType;
    const VectorType::const_iterator end = states.end();
    VectorType::const_iterator iter = states.begin();
    for (; iter != end; ++iter) {
        AddState(*iter);
    }
}

bool mtsStateMachine::StateExists(const StateType state) const
{
    const typename StateMap::const_iterator found
        = mStates.find(state);
    return (found != mStates.end());
}

void mtsStateMachine::AddAllowedDesiredState(const StateType allowedState)
{
    if (StateExists(allowedState)) {
        mStates[allowedState] = true;
    } else {
        cmnThrow("mtsStateMachine::AddAllowedDesiredState: "
                 + mName + ", state needs to be added first");
    }
}

void mtsStateMachine::Run(void)
{
    // on first run, call enter callback for initial state
    if (mFirstRun) {
        // update callbacks
        UpdateCurrentCallbacks();
        // find new state enter callback
        typename CallbackMap::iterator found;
        found = mEnterCallbacks.find(mCurrentState);
        if (found != mEnterCallbacks.end()) {
            found->second->Execute();
        }
        mFirstRun = false;
    }
    // run current state method
    if (mRunCallback) {
        mRunCallback->Execute();
    }
    if (mCurrentRunCallback) {
        mCurrentRunCallback->Execute();
    }
    // check if a transition should happen
    if (mCurrentTransitionCallback) {
        mCurrentTransitionCallback->Execute();
    }
}

bool mtsStateMachine::SetDesiredState(const StateType & desiredState)
{
    const typename StateMap::const_iterator found
        = mStates.find(desiredState);
    if ((found != mStates.end()) // state exists
        && (found->second)) {  // can be set as desired
        mDesiredState = desiredState;
        return true;
    }
    return false;
}

void mtsStateMachine::SetCurrentState(const StateType newState)
{
    typename CallbackMap::iterator found;

    // find current state leave callback
    found = mLeaveCallbacks.find(mCurrentState);
    if (found != mLeaveCallbacks.end()) {
        found->second->Execute();
    }
    // set the new state and update current callbacks
    mCurrentState = newState;
    // find new state enter callback
    found = mEnterCallbacks.find(mCurrentState);
    if (found != mEnterCallbacks.end()) {
        found->second->Execute();
    }
    // update current callbacks
    UpdateCurrentCallbacks();
    // user callback if provided
    if (mStateChangeCallback) {
        mStateChangeCallback->Execute();
    }
}

void mtsStateMachine::UpdateCurrentCallbacks(void)
{
    typename CallbackMap::iterator found;
    // find state run callback
    found = mRunCallbacks.find(mCurrentState);
    if (found != mRunCallbacks.end()) {
        mCurrentRunCallback = found->second;
    } else {
        mCurrentRunCallback = 0;
    }
    // find transition callback
    found = mTransitionCallbacks.find(mCurrentState);
    if (found != mTransitionCallbacks.end()) {
        mCurrentTransitionCallback = found->second;
    } else {
        mCurrentTransitionCallback = 0;
    }
}
