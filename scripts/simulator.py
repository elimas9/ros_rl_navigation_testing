import rospy
from qlearning.srv import SetParams, GetNextAction, Update


def main():

    # create proxies
    rospy.wait_for_service('qlearning/set_params')
    set_params = rospy.ServiceProxy('qlearning/set_params', SetParams)
    rospy.wait_for_service('qlearning/get_next_action')
    get_next_action = rospy.ServiceProxy('qlearning/get_next_action', GetNextAction)
    rospy.wait_for_service('qlearning/update')
    update = rospy.ServiceProxy('qlearning/update', Update)

    res = set_params(4, 2, 0.8, 5.0, 0.9, [])

    for e in range(100):

        print("Trial {}".format(e))

        # trial
        s = 0
        action_count = 0
        while s != 3:
            # choose action and move
            a = get_next_action(s).action
            if a == 1:
                s1 = min(s+1, 4-1)
            else:
                s1 = max(s-1, 0)
            # update q
            r = 0.0
            if s1 == 3:
                r = 1.0
            update(s1, r)
            # set next state
            s = s1
            action_count += 1

        # wait for keypress after each trial
        print("Finished trial after {} actions, press any key to continue".format(action_count))
        raw_input()


if __name__ == '__main__':
    main()
