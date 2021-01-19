import json


def iter_bag(path):
    with open(path) as file:
        bag = json.load(file)

    for row in bag:
        yield row


def get_key(tree, key, default=None):
    key = key.split(".")
    try:
        return get_key_recurse(tree, key, 0)
    except KeyError:
        return default


def get_key_recurse(tree, key, index):
    subfield = key[index]
    if index + 1 == len(key):
        return tree[subfield]
    else:
        return get_key_recurse(tree[subfield], key, index + 1)
