from .detect_environment import on_duckiebot

using_fake_contracts = False

if on_duckiebot():
    using_fake_contracts = True
else:
    try:
        # use PyContracts if installed 
        from contracts import contract  # @UnresolvedImport @UnusedImport
    except ImportError:
        using_fake_contracts = True
    
if using_fake_contracts:
    def contract(**kwargs):  # @UnusedVariable
        def phi(f):
            return f
        return phi
