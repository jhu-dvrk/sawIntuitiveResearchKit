/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-02-05

  (C) Copyright 2016-2018 Johns Hopkins University (JHU), All Rights Reserved.

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
                 + mName + ", state " + state + " already exists");
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
    const StateMap::const_iterator found
        = mStates.find(state);
    return (found != mStates.end());
}

void mtsStateMachine::AddAllowedDesiredState(const StateType allowedState)
{
    if (StateExists(allowedState)) {
        mStates[allowedState] = true;
    } else {
        cmnThrow("mtsStateMachine::AddAllowedDesiredState: "
                 + mName + ", state " + allowedState + " needs to be added first");
    }
}

void mtsStateMachine::Run(void)
{
    // on first run, call enter callback for initial state
    if (mFirstRun) {
        // update callbacks
        UpdateCurrentCallbacks();

        // find new state enter callback
        CallbackMap::iterator callback;
        callback = mEnterCallbacks.find(mCurrentState);
        if (callback != mEnterCallbacks.end()) {
            callback->second->Execute();
        }

        // user callback if provided
        if (mStateChangeCallback) {
            mStateChangeCallback->Execute();
        }

        mFirstRun = false;
    }
    // check if a transition should happen
    if (mCurrentTransitionCallback) {
        mCurrentTransitionCallback->Execute();
    }
    // run current state method
    if (mRunCallback) {
        mRunCallback->Execute();
    }
    if (mCurrentRunCallback) {
        mCurrentRunCallback->Execute();
    }
}

void mtsStateMachine::SetDesiredState(const StateType & desiredState)
{
    const StateMap::const_iterator state
        = mStates.find(desiredState);
    if ((state != mStates.end()) // state exists
        && (state->second)) {  // can be set as desired
        mPreviousDesiredState = mDesiredState;
        mDesiredState = desiredState;
        mDesiredStateIsNotCurrent = (mDesiredState != mCurrentState);
        return;
    }
    cmnThrow("mtsStateMachine::SetDesiredState: "
             + desiredState + ", doesn't exists or is not allowed as a desired state");
}

void mtsStateMachine::SetCurrentState(const StateType & newState)
{
    // check if this state exists
    const StateMap::const_iterator state = mStates.find(newState);
    if (state == mStates.end()) {
        cmnThrow("mtsStateMachine::SetCurrentState: "
                 + newState + ", doesn't exists");
        return;
    }

    CallbackMap::iterator callback;

    // find current state leave callback
    callback = mLeaveCallbacks.find(mCurrentState);
    if (callback != mLeaveCallbacks.end()) {
        callback->second->Execute();
    }
    // set the new state and update current callbacks
    mPreviousState = mCurrentState;
    mCurrentState = newState;
    mDesiredStateIsNotCurrent = (mDesiredState != mCurrentState);

    // find new state enter callback
    callback = mEnterCallbacks.find(mCurrentState);
    if (callback != mEnterCallbacks.end()) {
        callback->second->Execute();
    }

    // user callback if provided
    if (mStateChangeCallback) {
        mStateChangeCallback->Execute();
    }

    // update current callbacks
    UpdateCurrentCallbacks();
}

void mtsStateMachine::UpdateCurrentCallbacks(void)
{
    CallbackMap::iterator callback;
    // find state run callback
    callback = mRunCallbacks.find(mCurrentState);
    if (callback != mRunCallbacks.end()) {
        mCurrentRunCallback = callback->second;
    } else {
        mCurrentRunCallback = 0;
    }
    // find transition callback
    callback = mTransitionCallbacks.find(mCurrentState);
    if (callback != mTransitionCallbacks.end()) {
        mCurrentTransitionCallback = callback->second;
    } else {
        mCurrentTransitionCallback = 0;
    }
}
