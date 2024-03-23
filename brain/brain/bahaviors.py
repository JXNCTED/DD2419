import py_trees as pt
import py_trees_ros as ptr


class SimpleBehavior(pt.behaviour.Behaviour):
    def __init__(self, name="SimpleBehavior"):
        super(SimpleBehavior, self).__init__(name=name)

    def update(self):
        print("SimpleBehavior: Running")
        return pt.common.Status.SUCCESS


class SimpleBehavior2(pt.behaviour.Behaviour):
    def __init__(self, name="SimpleBehavior2"):
        super(SimpleBehavior2, self).__init__(name=name)

    def update(self):
        print("SimpleBehavior2: Running")
        return pt.common.Status.RUNNING



