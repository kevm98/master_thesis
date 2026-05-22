class ControlPolicy:
    """A simple policy interface for Kevin integration."""

    def __init__(self, config=None):
        self.config = config or {}

    def compute_action(self, observation):
        """Compute an action from a robot observation.

        Replace this stub logic with your actual controller.
        """
        # Example: return the observation unchanged.
        return observation
