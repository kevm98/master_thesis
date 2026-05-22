"""Starter script for running the Kevin integration control policy."""

from kevin_integration.control_policy import ControlPolicy


def main():
    policy = ControlPolicy(config={})
    print("ControlPolicy loaded:", policy)

    # TODO: Replace this with your actual robot observation retrieval.
    observation = [0.0, 0.0, 0.0]
    action = policy.compute_action(observation)
    print("Computed action:", action)


if __name__ == "__main__":
    main()
