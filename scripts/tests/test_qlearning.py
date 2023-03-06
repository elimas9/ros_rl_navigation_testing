import rospy
from qlearning.srv import SetParams, GetNextAction, Update


def check_result(res, success, msg):
    assert res.success == success, "expected success {}, got {}".format(success, res.success)
    assert res.error_msg == msg, "expected msg '{}', got '{}'".format(msg, res.error_msg)


def main():

    # test set params
    rospy.wait_for_service('qlearning/set_params', 2.)
    set_params = rospy.ServiceProxy('qlearning/set_params', SetParams)
    res = set_params(-4, 2, 0.8, 5.0, 0.9, [])
    check_result(res, False, "Invalid number of states")
    res = set_params(4, -2, 0.8, 5.0, 0.9, [])
    check_result(res, False, "Invalid number of actions")
    res = set_params(4, 2, 0.8, 5.0, 0.9, [1.] * 4 * 3)
    check_result(res, False, "Invalid Q matrix")
    res = set_params(4, 2, 0.8, 5.0, 0.9, [1.] * 4 * 2)
    check_result(res, True, "")
    res = set_params(4, 2, 0.8, 5.0, 0.9, [])
    check_result(res, True, "")

    # test get_next_action
    rospy.wait_for_service('qlearning/get_next_action', 2.)
    get_next_action = rospy.ServiceProxy('qlearning/get_next_action', GetNextAction)
    res = get_next_action(-2)
    check_result(res, False, "Invalid state")
    res = get_next_action(4)
    check_result(res, False, "Invalid state")
    res = get_next_action(0)
    check_result(res, True, "")
    assert(0 <= res.action < 4)

    # test update
    rospy.wait_for_service('qlearning/update', 2.)
    update = rospy.ServiceProxy('qlearning/update', Update)
    res = update(-2, 0.0)
    check_result(res, False, "Invalid state")
    res = update(4, 0.0)
    check_result(res, False, "Invalid state")
    res = update(1, 0.0)
    check_result(res, True, "")


if __name__ == '__main__':
    main()