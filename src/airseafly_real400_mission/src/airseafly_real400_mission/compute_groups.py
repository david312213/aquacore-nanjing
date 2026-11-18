"""Flight planning and optional red-point observation use separate runners."""
GROUPS = {'NONE': frozenset(), 'EGO': frozenset(('EGO',)), 'DOWN': frozenset(('DOWN',))}
def members(group):
    return GROUPS[group]
def group_name(workers):
    return next(name for name, value in GROUPS.items() if value == frozenset(workers))
