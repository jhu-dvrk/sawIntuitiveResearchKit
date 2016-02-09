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

#ifndef _mtsStateMachine_h
#define _mtsStateMachine_h

#include <cisstMultiTask/mtsCallableVoidMethod.h>

/*
  \todo
  - Add name in ctor so messages can include name
  - Add mtsFunction(std::string) for error and status (compatible dVRK), maybe a change state callback?
  - For SetDesiredState, add list of allowed transition
    map<currentState, list<allowedDesired>>, maybe special syntax for all states allowed?
  - Add SetDesiredState(std::string) 
  - Add ErrorCallback(std::string)
 */


template <typename _enumType, typename _containerClass>
class mtsStateMachine
{
public:
    typedef _enumType StateType;
    typedef _containerClass ClassType;

    inline mtsStateMachine(const std::string & name,
                           const StateType initialState):
        mName(name),
        mFirstRun(true)
    {
        typename CallbackMap::iterator found;
        // find state run callback
        found = mRunCallbacks.find(initialState);
        if (found != mRunCallbacks.end()) {
            mCurrentRunCallback = found->second;
        } else {
            mCurrentRunCallback = 0;
        }
        // find state leave trigger callback
        found = mLeaveTriggerCallbacks.find(initialState);
        if (found != mLeaveTriggerCallbacks.end()) {
            mCurrentLeaveTriggerCallback = found->second;
        } else {
            mCurrentLeaveTriggerCallback = 0;
        }
        // and finally set the new state
        mCurrentState = initialState;
        std::cerr << mName << ": initial state is "
                  << ClassType::StateTypeToString(mCurrentState) << std::endl;
    }

    void AddAllowedDesiredStates(const StateType allowedState) {
        mAllowedDesiredStates[allowedState] = true;
    }

    void SetRunCallback(const StateType state, const mtsCallableVoidBase * callback);
    template <class __classType>
    inline void SetRunCallback(const StateType state,
                               void (__classType::*method)(void),
                               __classType * classInstantiation) {
        this->SetRunCallback(state, new mtsCallableVoidMethod<__classType>(method, classInstantiation));
    }

    void SetEnterCallback(const StateType state, mtsCallableVoidBase & callback);
    template <class __classType>
    inline void SetEnterCallback(const StateType state,
                                 void (__classType::*method)(void),
                                 __classType * classInstantiation) {
        this->SetEnterCallback(state, new mtsCallableVoidMethod<__classType>(method, classInstantiation));
    }

    void SetLeaveCallback(const StateType state, mtsCallableVoidBase & callback);
    template <class __classType>
    inline void SetLEaveCallback(const StateType state,
                                 void (__classType::*method)(void),
                                 __classType * classInstantiation) {
        this->SetLeaveCallback(state, new mtsCallableVoidMethod<__classType>(method, classInstantiation));
    }

    void SetLeaveTriggerCallback(const StateType state, mtsCallableVoidBase & callback);
    template <class __classType>
    inline void SetLeaveTriggerCallback(const StateType state,
                                        void (__classType::*method)(void),
                                        __classType * classInstantiation) {
        this->SetLeaveTriggerCallback(state, new mtsCallableVoidMethod<__classType>(method, classInstantiation));
    }

    inline void Run(void) {
        // on first run, call enter callback for initial state
        if (mFirstRun) {
            typename CallbackMap::iterator found;
            // find new state enter callback
            found = mEnterCallbacks.find(mCurrentState);
            if (found != mEnterCallbacks.end()) {
                found->second->Execute();
            }
            mFirstRun = false;
        }
        // check with current state to see if ready to leave
        // the callback might call SetCurrentState
        if (mCurrentLeaveTriggerCallback) {
            mCurrentLeaveTriggerCallback->Execute();
        }
        if (mCurrentRunCallback) {
            mCurrentRunCallback->Execute();
        }
    }

    inline const StateType & CurrentState(void) const {
        return mCurrentState;
    }

    inline const StateType & DesiredState(void) const {
        return mDesiredState;
    }

    inline bool SetDesiredState(const StateType & desiredState) {
        const typename AllowedStateMap::const_iterator found
            = mAllowedDesiredStates.find(desiredState);
        if (found != mAllowedDesiredStates.end()) {
            mDesiredState = desiredState;
            std::cerr << mName << ": desired state is "
                      << ClassType::StateTypeToString(mDesiredState) << std::endl;
            return true;
        }
        std::cerr << mName << ": "
                  << ClassType::StateTypeToString(desiredState)
                  << " is not allowed as a desired state" << std::endl;
        return false;
    }

protected:

    inline void SetCurrentState(const StateType newState) {
        typename CallbackMap::iterator found;
        
        // find current state leave callback
        found = mLeaveCallbacks.find(mCurrentState);
        if (found != mLeaveCallbacks.end()) {
            found->second->Execute();
        }
        // find new state enter callback
        found = mEnterCallbacks.find(newState);
        if (found != mEnterCallbacks.end()) {
            found->second->Execute();
        }
        // find state run callback
        found = mRunCallbacks.find(newState);
        if (found != mRunCallbacks.end()) {
            mCurrentRunCallback = found->second;
        } else {
            mCurrentRunCallback = 0;
        }
        // find state leave trigger callback
        found = mLeaveTriggerCallbacks.find(newState);
        if (found != mLeaveTriggerCallbacks.end()) {
            mCurrentLeaveTriggerCallback = found->second;
        } else {
            mCurrentLeaveTriggerCallback = 0;
        }
        // and finally set the new state
        mCurrentState = newState;

        std::cerr << mName << ": current state is "
                  << ClassType::StateTypeToString(mCurrentState) << std::endl;
    }

    std::string mName;
    bool mFirstRun;

    typedef std::map<StateType, mtsCallableVoidBase *> CallbackMap;

    CallbackMap mEnterCallbacks,
        mRunCallbacks,
        mLeaveCallbacks,
        mLeaveTriggerCallbacks;

    mtsCallableVoidBase * mCurrentRunCallback;
    mtsCallableVoidBase * mCurrentLeaveTriggerCallback;

    StateType mCurrentState;
    StateType mDesiredState;
    typedef std::map<StateType, bool> AllowedStateMap;
    AllowedStateMap mAllowedDesiredStates;

private:
    // default constructor disabled
    mtsStateMachine(void);
};

#endif // _mtsStateMachine_h
