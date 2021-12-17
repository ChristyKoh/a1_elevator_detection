#!/usr/bin/env python
"""
Authors(s): Christy Koh
A1 control states for elevator navigation state machine.

References:
    https://dev.to/karn/building-a-simple-state-machine-in-python
    https://barrgroup.com/embedded-systems/how-to/coding-state-machines
"""

from enum import IntEnum


class ControlState(IntEnum):
    OUTSIDE_WAITING = 1
    ENTERING = 2
    PRESSING_BUTTON = 3  # not implemented yet
    TURNING_TO_DOOR = 4
    INSIDE_WAITING = 5
    EXITING = 6
    COMPLETE = 7

    def next(self):
        next_state = ControlState(self.value + 1)
        print_state(next_state)
        return next_state

    def __str__(self):
        return self.name


def print_state(state):
    if state == ControlState.OUTSIDE_WAITING:
        print("\n---------------------------------------")
        print("State: OUTSIDE_WAITING for door to open")
        print("---------------------------------------")
        print("DOOR CLOSED")
    elif state == ControlState.ENTERING:
        print("\n---------------------------------------")
        print("State: ENTERING elevator")
        print("---------------------------------------")
    elif state == ControlState.PRESSING_BUTTON:
        print("\n---------------------------------------")
        print("State: PRESSING_BUTTON for target floor")
        print("---------------------------------------")
    elif state == ControlState.TURNING_TO_DOOR:
        print("\n---------------------------------------")
        print("State: TURNING_TO_DOOR")
        print("---------------------------------------")
    elif state == ControlState.INSIDE_WAITING:
        print("\n---------------------------------------")
        print("State: WAITING to arrive at target floor")
        print("---------------------------------------")
    elif state == ControlState.EXITING:
        print("\n---------------------------------------")
        print("State 4: EXITING elevator")
        print("---------------------------------------")
    elif state == ControlState.COMPLETE:
        print("\n---------------------------------------")
        print("State: COMPLETE elevator operation")
        print("---------------------------------------")

