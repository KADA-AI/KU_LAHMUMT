"""
Configuration for the Monitoring CSC.
"""

# Messages to be sent from this module
PUSH_MESSAGES = (
    ("0102", "Set Operating Mode"),
    # Add other push messages as needed
)

# Messages to be received by this module
RECEIVE_MESSAGES = (
    ("0401", "Manned/Unmanned Aircraft Status"),
    ("0402", "Battlefield Situation Awareness"),
    # Add other receive messages as needed
)
