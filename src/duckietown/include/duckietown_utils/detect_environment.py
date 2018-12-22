
def on_duckiebot():
    """ True if we are on a Duckiebot. """
    import platform
    proc = platform.processor()
    on_the_duckiebot = not('x86' in proc)
    # armv7l
    return on_the_duckiebot

def on_laptop():
    """ True if we are on a laptop """
    return not on_circle() and not on_duckiebot()

def on_circle():
    """ True if we are running tests on the cloud. """
    import os
    return 'CIRCLECI' in os.environ