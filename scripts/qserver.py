import rospy
import numpy as np
from qlearning.srv import SetParams, SetParamsResponse, GetNextAction, GetNextActionResponse, Update, UpdateResponse
from qlearning.msg import QValues
import argparse


num_states = None
num_actions = None
alpha = None
beta = None
gamma = None
qvalues = None
last_state = None
last_action = None


def set_params(req):
    # check parameters
    if req.num_states < 1:
        return SetParamsResponse(False, "Invalid number of states")
    if req.num_actions < 1:
        return SetParamsResponse(False, "Invalid number of actions")

    if len(req.initial_q) > 0:
        if len(req.initial_q) != req.num_states * req.num_actions:
            return SetParamsResponse(False, "Invalid Q matrix")
        tmp_q = np.array(req.initial_q)
        tmp_q = tmp_q.reshape((req.num_states, req.num_actions))
    else:
        tmp_q = np.zeros((req.num_states, req.num_actions))

    # if everything is ok we set the parameters
    global num_states
    global num_actions
    global alpha
    global beta
    global gamma
    global qvalues
    global last_state
    global last_action
    num_states = req.num_states
    num_actions = req.num_actions
    alpha = req.alpha
    beta = req.beta
    gamma = req.gamma
    qvalues = tmp_q
    last_state = None
    last_action = None
    return SetParamsResponse(True, "")


def get_next_action(req):
    # check state
    s = req.state
    if not 0 <= s < num_states:
        return GetNextActionResponse(0, False, "Invalid state")

    # softmax
    exp_prob = np.exp(beta * qvalues[s])
    softmax = np.exp(beta * qvalues[s]) / np.sum(exp_prob)

    # choose action
    next_action = np.random.choice(range(num_actions), p=softmax)

    # save last starting state and action
    global last_state
    global last_action
    last_state = s
    last_action = next_action
    return GetNextActionResponse(next_action, True, "")


def update(req):
    # check state and get values for update
    s = last_state
    a = last_action
    s1 = req.arrival_state
    r = req.reward
    if not 0 <= s1 < num_states:
        return UpdateResponse(False, "Invalid state")
    if s is None:
        return UpdateResponse(False, "Invalid update")

    # learning
    prediction_error = r + gamma * np.max(qvalues[s1]) - qvalues[s, a]
    qvalues[s, a] = qvalues[s, a] + alpha * prediction_error

    print(qvalues)

    return UpdateResponse(True, "")


def main():

    parser = argparse.ArgumentParser(description='Q-learning server')
    parser.add_argument('-r', '--rate', type=float, default=10.0,
                        help='rate for publishing q-values (Hz)')

    args = parser.parse_args()


    rospy.init_node('qlearning_server')
    rospy.Service('qlearning/set_params', SetParams, set_params)
    rospy.Service('qlearning/get_next_action', GetNextAction, get_next_action)
    rospy.Service('qlearning/update', Update, update)
    pub = rospy.Publisher("qlearning/qvalues", QValues, queue_size=10)
    rate = rospy.Rate(args.rate)
    while not rospy.is_shutdown():
        q = QValues()
        q.num_states = num_states if num_states is not None else 0
        q.num_actions = num_actions if num_actions is not None else 0
        if qvalues is None:
            q.q_values = []
        else:
            q.q_values = np.reshape(qvalues, num_actions * num_states)
        pub.publish(q)
        rate.sleep()


if __name__ == '__main__':
    main()
