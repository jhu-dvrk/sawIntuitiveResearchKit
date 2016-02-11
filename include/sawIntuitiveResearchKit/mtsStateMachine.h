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

  General architecture:
  - use any type for state (using int allows adding more "states" in derived classes)
  - callbacks for enter/leave should take current state as value
  - remove container class, user has to convert state to string
  - add AddAllowedState method
  - add GetAllStates method so users can add their own 
 */


template <typename _enumType, typename _containerClass>
class mtsStateMachine
{
public:
    typedef _enumType StateType;
    typedef _containerClass ClassType;

    inline mtsStateMachine(const StateType initialState):
        mFirstRun(true),
        mCurrentState(initialState),
        mStateChangeCallback(0)
    {
    }

    void AddAllowedDesiredStates(const StateType allowedState) {
        mAllowedDesiredStates[allowedState] = true;
    }

    inline void SetRunCallback(const StateType state, mtsCallableVoidBase * callback) {
        mRunCallbacks[state] = callback;
    }
    template <class __classType>
    inline void SetRunCallback(const StateType state,
                               void (__classType::*method)(void),
                               __classType * classInstantiation) {
        this->SetRunCallback(state, new mtsCallableVoidMethod<__classType>(method, classInstantiation));
    }

    inline void SetEnterCallback(const StateType state, mtsCallableVoidBase * callback) {
        mEnterCallbacks[state] = callback;
    }
    template <class __classType>
    inline void SetEnterCallback(const StateType state,
                                 void (__classType::*method)(void),
                                 __classType * classInstantiation) {
        this->SetEnterCallback(state, new mtsCallableVoidMethod<__classType>(method, classInstantiation));
    }

    inline void SetLeaveCallback(const StateType state, mtsCallableVoidBase * callback) {
        mLeaveCallbacks[state] = callback;
    }
    template <class __classType>
    inline void SetLeaveCallback(const StateType state,
                                 void (__classType::*method)(void),
                                 __classType * classInstantiation) {
        this->SetLeaveCallback(state, new mtsCallableVoidMethod<__classType>(method, classInstantiation));
    }

    inline void SetTransitionCallback(const StateType state, mtsCallableVoidBase * callback) {
        mTransitionCallbacks[state] = callback;
    }
    template <class __classType>
    inline void SetTransitionCallback(const StateType state,
                                      void (__classType::*method)(void),
                                      __classType * classInstantiation) {
        this->SetTransitionCallback(state, new mtsCallableVoidMethod<__classType>(method, classInstantiation));
    }

    inline void SetStateChangedCallback(mtsCallableVoidBase * callback) {
        mStateChangeCallback = callback;
    }
    template <class __classType>
    inline void SetStateChangedCallback(void (__classType::*method)(void),
                                        __classType * classInstantiation) {
        this->SetStateChangedCallback(new mtsCallableVoidMethod<__classType>(method, classInstantiation));
    }

    inline void Run(void) {
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
        if (mCurrentRunCallback) {
            mCurrentRunCallback->Execute();
        }
        // check if a transition should happen
        if (mCurrentTransitionCallback) {
            mCurrentTransitionCallback->Execute();
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
            std::cerr << "desired state is "
                      << ClassType::StateTypeToString(mDesiredState) << std::endl;
            return true;
        }
        std::cerr << ClassType::StateTypeToString(desiredState)
                  << " is not allowed as a desired state" << std::endl;
        return false;
    }

    inline void SetCurrentState(const StateType newState) {
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

protected:

    void UpdateCurrentCallbacks(void)
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

    std::string mName;
    bool mFirstRun;

    typedef std::map<StateType, mtsCallableVoidBase *> CallbackMap;

    CallbackMap mEnterCallbacks,
                mRunCallbacks,
                mLeaveCallbacks,
                mTransitionCallbacks;
                     

    mtsCallableVoidBase * mCurrentRunCallback;
    mtsCallableVoidBase * mCurrentTransitionCallback;
    mtsCallableVoidBase * mStateChangeCallback;

    StateType mCurrentState;
    StateType mDesiredState;
    typedef std::map<StateType, bool> AllowedStateMap;
    AllowedStateMap mAllowedDesiredStates;

private:
    // default constructor disabled
    mtsStateMachine(void);
};

#endif // _mtsStateMachine_h
