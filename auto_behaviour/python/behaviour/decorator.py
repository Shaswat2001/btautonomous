
from py_trees.decorators import Decorator
from py_trees import behaviour,common

class OneShot(Decorator):
    """
    A decorator that implements the oneshot pattern.

    This decorator ensures that the underlying child is ticked through
    to completion just once and while doing so, will return
    with the same status as it's child. Thereafter it will return
    with the final status of the underlying child.

    Completion status is determined by the policy given on construction.

    * With policy :data:`~py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION`, the oneshot will activate
      only when the underlying child returns :data:`~py_trees.common.Status.SUCCESS` (i.e. it permits retries).
    * With policy :data:`~py_trees.common.OneShotPolicy.ON_COMPLETION`, the oneshot will activate when the child
      returns :data:`~py_trees.common.Status.SUCCESS` || :data:`~py_trees.common.Status.FAILURE`.

    .. seealso:: :meth:`py_trees.idioms.oneshot`
    """

    def __init__(self, name, child, completion_value):
        """
        Init with the decorated child.

        Args:
            child: behaviour to shoot
            name: the decorator name
            policy: policy determining when the oneshot should activate
        """
        super(OneShot, self).__init__(name=name, child=child)
        self.final_status = None
        if completion_value:
            self.policy = [common.Status.FAILURE]
        else:
            self.policy = [common.Status.SUCCESS,common.Status.FAILURE]

    def update(self):
        """
        Bounce if the child has already successfully completed.

        Returns:
            the behaviour's new status :class:`~py_trees.common.Status`
        """
        if self.final_status:
            self.logger.debug("{}.update()[bouncing]".format(self.__class__.__name__))
            return self.final_status
        return self.decorated.status

    def tick(self):
        """
        Tick the child or bounce back with the original status if already completed.

        Yields:
            a reference to itself or a behaviour in it's child subtree
        """
        if self.final_status:
            # ignore the child
            for node in behaviour.Behaviour.tick(self):
                yield node
        else:
            # tick the child
            for node in Decorator.tick(self):
                yield node


    def terminate(self, new_status) :
        """
        Prevent further entry if finishing with :data:`~py_trees.common.Status.SUCCESS`.

        This uses a flag to register that the behaviour has gone through to completion.
        In future ticks, it will block entry to the child and just return the original
        status result.
        """
        if not self.final_status and new_status in self.policy:
            self.logger.debug(
                "{}.terminate({})[oneshot completed]".format(
                    self.__class__.__name__, new_status
                )
            )
            self.feedback_message = "oneshot completed"
            self.final_status = new_status
        else:
            self.logger.debug(
                "{}.terminate({})".format(self.__class__.__name__, new_status)
            )