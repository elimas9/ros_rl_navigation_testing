from collections import namedtuple
import random


glob_num_states = None
glob_num_actions = None

SetParamsResponse = namedtuple('SetParamsResponse', ['success', 'error_msg'])

def set_params(num_states, num_actions, alpha, beta, gamma, initial_state):
    global glob_num_states
    global glob_num_actions
    if num_states < 0:
        return SetParamsResponse(False, "Invalid number of states")
    if num_actions < 0:
        return SetParamsResponse(False, "Invalid number of actions")
    if initial_state < 0 or initial_state >= num_states:
        return SetParamsResponse(False, "Invalid initial state")
    glob_num_states = num_states
    glob_num_actions = num_actions
    return SetParamsResponse(True, "")


GetNextActionResponse = namedtuple('GetNextActionResponse', ['action', 'success', 'error_msg'])

def get_next_action(state):
    if state < 0 or state >= glob_num_states:
        return GetNextActionResponse(0, False, "Invalid state")
    return GetNextActionResponse(random.choice(range(glob_num_actions)), True, "")

UpdateResponse = namedtuple('UpdateResponse', ['success', 'error_msg'])

def update(state):
    if state < 0 or state >= glob_num_states:
        return UpdateResponse(False, "Invalid state")
    return UpdateResponse(True, "")