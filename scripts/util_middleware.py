from typing import List


def extractRobotIdFromNamespace(_ego_robot_ns):
    if _ego_robot_ns[0] == "/":
        if _ego_robot_ns[-1] == "/":
            return int(_ego_robot_ns[-2])
        else:
            return int(_ego_robot_ns[-1])
    else:
        return -999
    
def identifyOtherRobotNamespaces(_robot_ns_list, _ego_robot_ns):
    _other_robots_nss = []
    for ns in _robot_ns_list:
        if ns != _ego_robot_ns:
            _other_robots_nss.append(ns)
    return _other_robots_nss