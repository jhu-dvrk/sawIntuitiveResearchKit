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
    map<currentState, list<allowedDesired>>, maybe special syntax for all states allowed?
  - Add ErrorCallback(std::string)

  General architecture:
  - callbacks for enter/leave should take current state as value
  - add AddAllowedState method
  - add GetAllStates method so users can add their own
 */


class mtsStateMachine
{
public:
    typedef std::string StateType;

    inline mtsStateMachine(const std::string & name, const StateType initialState):
        mName(name),
        mFirstRun(true),
        mRunCallback(0),
        mStateChangeCallback(0),
        mCurrentState(initialState),
        mDesiredState(initialState)
    {
        AddState(initialState);
    }

    /*! Add a state. */
    void AddState(const StateType state);

    void AddStates(const std::vector<StateType> & states);

    bool StateExists(const StateType state) const;

    /*! Add an allowed desired state.  One can only use
      SetDesiredState with allowed states. */
    void AddAllowedDesiredState(const StateType allowedState);

    /*! Set the Run callback for a given state. */
    //@{
    inline void SetRunCallback(const StateType state, mtsCallableVoidBase * callback) {
        if (!StateExists(state)) {
            cmnThrow("mtsStateMachine::SetRunCallback: "
                     + mName + ", state [" + state + "] doesn't exist.  Use AddState first.");
        }
        mRunCallbacks[state] = callback;
    }
    template <class __classType>
    inline void SetRunCallback(const StateType state,
                               void (__classType::*method)(void),
                               __classType * classInstantiation) {
        this->SetRunCallback(state, new mtsCallableVoidMethod<__classType>(method, classInstantiation));
    }
    //@}

    /*! Set the Run callback called for all states, this method is
      called before the state specific Run callback. */
    //@{
    inline void SetRunCallback(mtsCallableVoidBase * callback) {
        mRunCallback = callback;
    }
    template <class __classType>
    inline void SetRunCallback(void (__classType::*method)(void),
                               __classType * classInstantiation) {
        this->SetRunCallback(new mtsCallableVoidMethod<__classType>(method, classInstantiation));
    }
    //@}

    /*! Set the Enter callback for a given state.  This method is
      called only once, before the Run callback. */
    //@{
    inline void SetEnterCallback(const StateType state, mtsCallableVoidBase * callback) {
        if (!StateExists(state)) {
            cmnThrow("mtsStateMachine::SetEnterCallback: "
                     + mName + ", state [" + state + "] doesn't exist.  Use AddState first.");
        }
        mEnterCallbacks[state] = callback;
    }
    template <class __classType>
    inline void SetEnterCallback(const StateType state,
                                 void (__classType::*method)(void),
                                 __classType * classInstantiation) {
        this->SetEnterCallback(state, new mtsCallableVoidMethod<__classType>(method, classInstantiation));
    }
    //@}

    /*! Set the Leave callback for a given state.  Called once when
      leaving the current state. */
    //@{
    inline void SetLeaveCallback(const StateType state, mtsCallableVoidBase * callback) {
        if (!StateExists(state)) {
            cmnThrow("mtsStateMachine::SetLeaveCallback: "
                     + mName + ", state [" + state + "] doesn't exist.  Use AddState first.");
        }
        mLeaveCallbacks[state] = callback;
    }
    template <class __classType>
    inline void SetLeaveCallback(const StateType state,
                                 void (__classType::*method)(void),
                                 __classType * classInstantiation) {
        this->SetLeaveCallback(state, new mtsCallableVoidMethod<__classType>(method, classInstantiation));
    }
    //@}

    /*! Set the Transition callback for a given state.  This callback
      is called after the Run callback for the current state. */
    //@{
    inline void SetTransitionCallback(const StateType state, mtsCallableVoidBase * callback) {
        if (!StateExists(state)) {
            cmnThrow("mtsStateMachine::SetTransitionCallback: "
                     + mName + ", state [" + state + "] doesn't exist.  Use AddState first.");
        }
        mTransitionCallbacks[state] = callback;
    }
    template <class __classType>
    inline void SetTransitionCallback(const StateType state,
                                      void (__classType::*method)(void),
                                      __classType * classInstantiation) {
        this->SetTransitionCallback(state, new mtsCallableVoidMethod<__classType>(method, classInstantiation));
    }
    //@}

    /*! Set the state changed callback. */
    //@{
    inline void SetStateChangedCallback(mtsCallableVoidBase * callback) {
        mStateChangeCallback = callback;
    }
    template <class __classType>
    inline void SetStateChangedCallback(void (__classType::*method)(void),
                                        __classType * classInstantiation) {
        this->SetStateChangedCallback(new mtsCallableVoidMethod<__classType>(method, classInstantiation));
    }
    //@}

    void Run(void);

    inline const StateType & CurrentState(void) const {
        return mCurrentState;
    }

    inline const StateType & DesiredState(void) const {
        return mDesiredState;
    }

    /*! Set the desired state.  This will check if the state is a
      possible desired state. */
    void SetDesiredState(const StateType & desiredState);

    /*! Set the current state.  This will check if the state is a
      valid state.  Leave and enter callbacks will also be called.
      Finally all callback pointers for the current state (run and
      transition) will be updated to avoid a callback lookup by state
      name in the Run method. */
    void SetCurrentState(const StateType & newState);

    /*! Check if the desired and current states are different.  This
        allows to avoid a string compare to determine if a transition
        is desired. */
    inline bool DesiredStateIsNotCurrent(void) const {
        return mDesiredStateIsNotCurrent;
    }

protected:

    void UpdateCurrentCallbacks(void);

    std::string mName;
    bool mFirstRun;
    bool mDesiredStateIsNotCurrent;

    typedef std::map<StateType, mtsCallableVoidBase *> CallbackMap;

    CallbackMap mEnterCallbacks,
                mRunCallbacks,
                mLeaveCallbacks,
                mTransitionCallbacks;

    mtsCallableVoidBase * mRunCallback,
                        * mCurrentRunCallback,
                        * mCurrentTransitionCallback,
                        * mStateChangeCallback;

    StateType mCurrentState;
    StateType mDesiredState;
    // if true, can be used set desired state
    typedef std::map<StateType, bool> StateMap;
    StateMap mStates;

private:
    // default constructor disabled
    mtsStateMachine(void);
};

#endif // _mtsStateMachine_h
